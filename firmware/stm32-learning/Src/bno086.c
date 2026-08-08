#include "bno086.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

#include <math.h>
#include <string.h>

/*
 * BNO086 driver: CEVA's sh2 stack over an SPI transport supplied here.
 *
 * This module owns the wires -- chip select, reset and the SPI transfers --
 * and nothing above them.  SHTP framing, the SH-2 command set and report
 * decoding all belong to Drivers/sh2.
 *
 * Game Rotation Vector is used rather than Rotation Vector
 * because the latter folds in the magnetometer.  Twelve STS3215 servos put
 * permanent magnets a few centimetres from the sensor, so a magnetic heading
 * would be worse than none.  The balance loop only consumes roll and pitch,
 * which the gyro/accel fusion supplies without magnetometer help; the yaw this
 * report produces is relative to power-on rather than to north.
 */


/*
 * Set when the breakout selects SPI with a soldered PS0 jumper rather than
 * leaving the pin for the host.  PS0 doubles as WAKE, so a strapped board has
 * no wake line: the firmware must not drive IMU_WAKE, and the D4 wire should
 * be removed so nothing sits across the strap.
 */
#define BNO086_PS0_STRAPPED      1

#define BNO086_INT_TIMEOUT_MS    200U
#define BNO086_RESET_DRAIN_MS    600U


/*
 * Sign conventions, matching what the README documents for this robot:
 *
 *   Pitch +  nose down, tipping toward the front of the robot
 *   Roll  +  tipping toward the right of the robot
 *
 * The BNO086 is mounted like the BNO055 it replaced, so these start at +1.
 * Verify with 'imu on' on the bench and flip a constant here if a physical
 * tilt reports the opposite sign -- do not compensate in the balance gains.
 */
#define BNO086_ROLL_SIGN   (+1)
#define BNO086_PITCH_SIGN  (+1)
#define BNO086_YAW_SIGN    (+1)

/* Q14 fixed point: the unit quaternion arrives scaled by 2^14. */
#define BNO086_QUAT_SCALE  (1.0f / 16384.0f)

#define RAD_TO_TENTHS_DEG  (1800.0f / 3.14159265358979f)

static void gpio_write(GPIO_TypeDef *port, uint16_t pin, bool high)
{
    HAL_GPIO_WritePin(port, pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool interrupt_asserted(void)
{
    /* H_INTN is active low. */
    return HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET;
}

static bool wait_for_interrupt(uint32_t timeout_ms)
{
    const uint32_t started_at = HAL_GetTick();
    while (!interrupt_asserted()) {
        if ((uint32_t)(HAL_GetTick() - started_at) >= timeout_ms) {
            return false;
        }
    }
    return true;
}

/*
 * Clock bytes in while holding MOSI at zero.
 *
 * SPI here is genuinely full duplex: whatever the host clocks out is parsed by
 * the sensor as a host packet.  A run of zeros decodes as a zero-length header
 * and is discarded, so it is the only safe filler.  HAL_SPI_Receive would send
 * undefined bytes instead.
 */
static Bno086Result spi_read(Bno086 *imu, uint8_t *destination, size_t length)
{
    static const uint8_t zeros[32] = {0};
    uint8_t scratch[32];

    while (length != 0U) {
        const size_t chunk = length < sizeof(zeros) ? length : sizeof(zeros);
        uint8_t *target = destination != NULL ? destination : scratch;

        if (HAL_SPI_TransmitReceive(imu->spi,
                                    (uint8_t *)zeros,
                                    target,
                                    (uint16_t)chunk,
                                    100U) != HAL_OK) {
            return BNO086_SPI_ERROR;
        }
        if (destination != NULL) {
            destination += chunk;
        }
        length -= chunk;
    }
    return BNO086_OK;
}

/*
 * sh2 is a singleton, and its HAL callbacks only carry an sh2_Hal_t.  Keep the
 * instance here so the transport can reach the SPI handle.
 */
static Bno086 *active_imu;
static sh2_Hal_t sh2_hal;

/*
 * H_INTN says a packet is waiting, and honouring it keeps the steady-state
 * path cheap.  During bring-up it is also the one signal that has never been
 * seen to move, so read regardless there: an empty header is unambiguous, and
 * gating on a pin that may not work would hide a link that does.
 */
static bool hal_ignore_interrupt;

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    /* sh2_open() expects the part to come up from a known state. */
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, false);
    HAL_Delay(20);
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, true);
    HAL_Delay(300);
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, false);
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    /* Millisecond resolution is coarse but monotonic, which is what sh2 uses
     * it for: report timestamps and command timeouts, not control timing. */
    return HAL_GetTick() * 1000U;
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                    uint32_t *t_us)
{
    uint8_t header[4];

    (void)self;
    if (active_imu == NULL) {
        return 0;
    }
    if (!hal_ignore_interrupt && !interrupt_asserted()) {
        return 0;
    }

    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
    if (spi_read(active_imu, header, sizeof(header)) != BNO086_OK) {
        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
        return 0;
    }

    /* Bit 15 flags a continuation of an oversized transfer, not length. */
    const uint16_t total =
        (uint16_t)(((uint16_t)header[1] << 8) | header[0]) & 0x7FFFU;
    if (total < sizeof(header) || total > len) {
        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
        return 0;
    }

    memcpy(pBuffer, header, sizeof(header));
    Bno086Result result = BNO086_OK;
    if (total > sizeof(header)) {
        result = spi_read(active_imu, &pBuffer[sizeof(header)],
                          (size_t)total - sizeof(header));
    }
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);

    if (result != BNO086_OK) {
        return 0;
    }
    if (t_us != NULL) {
        *t_us = hal_get_time_us(self);
    }
    return (int)total;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;
    if (active_imu == NULL || len == 0U) {
        return 0;
    }

    /*
     * Prefer the window the sensor offers by asserting H_INTN, but do not
     * require it: a part with nothing queued never asserts, and refusing to
     * write then means the configuration that would give it something to say
     * can never be sent.
     */
    (void)wait_for_interrupt(BNO086_INT_TIMEOUT_MS);

    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
    const HAL_StatusTypeDef status =
        HAL_SPI_Transmit(active_imu->spi, pBuffer, (uint16_t)len, 100U);
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);

    return status == HAL_OK ? (int)len : 0;
}

static void update_angles(Bno086 *imu)
{
    const float i = imu->quat_i;
    const float j = imu->quat_j;
    const float k = imu->quat_k;
    const float w = imu->quat_real;

    const float roll = atan2f(2.0f * (w * i + j * k),
                              1.0f - 2.0f * (i * i + j * j));

    float sin_pitch = 2.0f * (w * j - k * i);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    const float pitch = asinf(sin_pitch);

    const float yaw = atan2f(2.0f * (w * k + i * j),
                             1.0f - 2.0f * (j * j + k * k));

    imu->roll_tenths =
        (int16_t)(BNO086_ROLL_SIGN * (int)(roll * RAD_TO_TENTHS_DEG));
    imu->pitch_tenths =
        (int16_t)(BNO086_PITCH_SIGN * (int)(pitch * RAD_TO_TENTHS_DEG));
    imu->yaw_tenths =
        (int16_t)(BNO086_YAW_SIGN * (int)(yaw * RAD_TO_TENTHS_DEG));
    imu->has_attitude = true;
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event)
{
    Bno086 *imu = (Bno086 *)cookie;
    sh2_SensorValue_t value;

    if (imu == NULL || sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        if (imu != NULL) {
            imu->protocol_errors++;
        }
        return;
    }
    if (value.sensorId != SH2_GAME_ROTATION_VECTOR) {
        return;
    }

    imu->quat_i = value.un.gameRotationVector.i;
    imu->quat_j = value.un.gameRotationVector.j;
    imu->quat_k = value.un.gameRotationVector.k;
    imu->quat_real = value.un.gameRotationVector.real;
    update_angles(imu);
    imu->report_count++;
    imu->last_report_tick = HAL_GetTick();
}

static void event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    Bno086 *imu = (Bno086 *)cookie;

    if (imu == NULL || event->eventId != SH2_RESET) {
        return;
    }
    /*
     * A reset clears every sensor configuration, so the subscription has to be
     * reissued.  Do it from bno086_service() rather than here: this runs
     * inside sh2_service() and reconfiguring re-enters the driver.
     */
    imu->resets_seen++;
    imu->resubscribe = true;
}

static int subscribe(Bno086 *imu)
{
    sh2_SensorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.reportInterval_us = imu->report_interval_us;
    return sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
}

void bno086_service(Bno086 *imu)
{
    if (imu == NULL || !imu->present) {
        return;
    }

    sh2_service();
    if (imu->resubscribe) {
        imu->resubscribe = false;
        (void)subscribe(imu);
    }
}

Bno086Result bno086_init(Bno086 *imu,
                         SPI_HandleTypeDef *spi,
                         uint32_t report_interval_us)
{
    if (imu == NULL || spi == NULL || report_interval_us == 0U) {
        return BNO086_PROTOCOL_ERROR;
    }

    memset(imu, 0, sizeof(*imu));
    imu->spi = spi;
    imu->report_interval_us = report_interval_us;
    active_imu = imu;
    hal_ignore_interrupt = true;

    sh2_hal.open = hal_open;
    sh2_hal.close = hal_close;
    sh2_hal.read = hal_read;
    sh2_hal.write = hal_write;
    sh2_hal.getTimeUs = hal_get_time_us;

    if (sh2_open(&sh2_hal, event_handler, imu) != SH2_OK) {
        active_imu = NULL;
        return BNO086_NOT_PRESENT;
    }
    if (sh2_setSensorCallback(sensor_handler, imu) != SH2_OK) {
        sh2_close();
        active_imu = NULL;
        return BNO086_PROTOCOL_ERROR;
    }

    /*
     * Ask for the product IDs before anything else.  Unlike setSensorConfig,
     * which only writes a command, this one waits for a reply, so its result
     * is the first hard evidence that the part is talking at all.
     */
    imu->product_id_status = sh2_getProdIds(&imu->product_ids);

    imu->present = true;
    if (subscribe(imu) != SH2_OK) {
        imu->present = false;
        sh2_close();
        active_imu = NULL;
        return BNO086_PROTOCOL_ERROR;
    }

    /*
     * Give the first reports a chance to land so callers can trust the cache.
     * A second is generous next to a 5 ms subscription, but a part that has
     * just been reset needs the slack.
     */
    const uint32_t subscribed_at = HAL_GetTick();
    while (!imu->has_attitude &&
           (uint32_t)(HAL_GetTick() - subscribed_at) < 1000U) {
        bno086_service(imu);
    }

    if (!imu->has_attitude) {
        imu->present = false;
        return BNO086_TIMEOUT;
    }

    /* Reports are flowing, so trust the interrupt from here on. */
    hal_ignore_interrupt = false;
    return BNO086_OK;
}

/* Sample H_INTN as a plain input, mirroring what linestate does for PA9/PA10. */
static uint32_t probe_int_percent(uint32_t pull)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = IMU_INT_Pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IMU_INT_GPIO_Port, &init);
    HAL_Delay(2);

    uint32_t high = 0U;
    uint32_t samples = 0U;
    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < 20U) {
        if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_SET) {
            ++high;
        }
        ++samples;
    }
    return samples == 0U ? 0U : (high * 100U) / samples;
}

void bno086_probe(Bno086 *imu, Bno086Probe *probe)
{
    if (imu == NULL || probe == NULL) {
        return;
    }
    memset(probe, 0, sizeof(*probe));

    /*
     * An idle H_INTN and a disconnected pin both read high, so pull it down:
     * only a pin something is actually driving stays high.
     */
    probe->int_float_percent = probe_int_percent(GPIO_NOPULL);
    probe->int_pulldown_percent = probe_int_percent(GPIO_PULLDOWN);

    /*
     * Sample again with the sensor held in reset.  If the sensor is the thing
     * driving H_INTN, its output goes high-Z here and the pull-down wins.  If
     * the pin stays high, something independent of the sensor holds it: a
     * board pull-up, or a reset line that never reaches the part.
     */
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, false);
    HAL_Delay(5);
    probe->int_in_reset_percent = probe_int_percent(GPIO_PULLDOWN);

    GPIO_InitTypeDef init = {0};
    init.Pin = IMU_INT_Pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IMU_INT_GPIO_Port, &init);

    HAL_Delay(20);
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, true);

    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < BNO086_RESET_DRAIN_MS) {
        if (interrupt_asserted()) {
            probe->int_asserted = true;
            probe->assert_delay_ms = (uint32_t)(HAL_GetTick() - started_at);
            break;
        }
    }

    /*
     * Read the header without consulting H_INTN.  A sensor that came up in
     * SPI mode has its advertisement waiting, so this returns real bytes even
     * when the interrupt wire is broken -- and stays blank when the sensor is
     * not speaking SPI, whatever the interrupt line happens to read.
     */
    for (uint32_t attempt = 0U; attempt < 24U && !probe->blind_data_seen;
         ++attempt) {
        uint8_t header[4] = {0};

        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
        const Bno086Result read = spi_read(imu, header, sizeof(header));
        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
        probe->blind_attempts = attempt + 1U;

        const bool blank =
            (header[0] == 0x00U && header[1] == 0x00U &&
             header[2] == 0x00U && header[3] == 0x00U) ||
            (header[0] == 0xFFU && header[1] == 0xFFU &&
             header[2] == 0xFFU && header[3] == 0xFFU);
        if (read == BNO086_OK && !blank) {
            memcpy(probe->blind_header, header, sizeof(header));
            probe->blind_data_seen = true;
        }
        HAL_Delay(20);
    }

    /* Control read with the sensor deselected; see Bno086Probe. */
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
    (void)spi_read(imu, probe->deselected_header,
                   sizeof(probe->deselected_header));

    /* Same read, selected, but with the part held in reset. */
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, false);
    HAL_Delay(5);
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
    (void)spi_read(imu, probe->in_reset_header,
                   sizeof(probe->in_reset_header));
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
    gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, true);
    HAL_Delay(100);

    /*
     * Sweep the four clock modes, resetting the part before each so a fresh
     * advertisement is waiting, and record what each one reads back.
     */
    const uint32_t polarity[4] = {SPI_POLARITY_LOW, SPI_POLARITY_LOW,
                                  SPI_POLARITY_HIGH, SPI_POLARITY_HIGH};
    const uint32_t phase[4] = {SPI_PHASE_1EDGE, SPI_PHASE_2EDGE,
                               SPI_PHASE_1EDGE, SPI_PHASE_2EDGE};
    for (uint32_t mode = 0U; mode < 4U; ++mode) {
        imu->spi->Init.CLKPolarity = polarity[mode];
        imu->spi->Init.CLKPhase = phase[mode];
        imu->spi->State = HAL_SPI_STATE_RESET;
        if (HAL_SPI_Init(imu->spi) != HAL_OK) {
            continue;
        }

        gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, false);
        HAL_Delay(20);
        gpio_write(IMU_RST_GPIO_Port, IMU_RST_Pin, true);
        HAL_Delay(250);

        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
        (void)spi_read(imu, probe->mode_header[mode], 4U);
        gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
    }

    /* Leave the bus on the mode the driver actually uses. */
    imu->spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    imu->spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    imu->spi->State = HAL_SPI_STATE_RESET;
    (void)HAL_SPI_Init(imu->spi);

    if (!probe->int_asserted) {
        return;
    }

    /*
     * Read the header only.  Its bytes tell wiring apart from protocol: all
     * 0x00 or all 0xFF means MISO is not carrying data, while a sane length
     * and channel means the link is up and the fault is further along.
     */
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, false);
    const Bno086Result result =
        spi_read(imu, probe->header, sizeof(probe->header));
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);
    probe->header_read = result == BNO086_OK;
}

static void loopback_set_gpio_mode(void)
{
    GPIO_InitTypeDef init = {0};

    init.Pin = GPIO_PIN_7;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &init);

    init.Pin = GPIO_PIN_6;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &init);
}

static void loopback_restore_spi_mode(void)
{
    GPIO_InitTypeDef init = {0};

    init.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &init);
}

void bno086_loopback_test(Bno086 *imu, Bno086Loopback *result)
{
    static const uint8_t pattern[] = {
        0x00U, 0xFFU, 0x55U, 0xAAU, 0x01U, 0x7EU, 0x81U, 0x42U
    };

    if (imu == NULL || result == NULL) {
        return;
    }
    memset(result, 0, sizeof(*result));
    result->failed_byte = -1;

    /* Keep the sensor deselected so a still-attached part cannot answer. */
    gpio_write(IMU_CS_GPIO_Port, IMU_CS_Pin, true);

    /*
     * DC continuity first.  Drive PA7 as a plain output and read PA6 against
     * an internal pull-down: a fitted jumper makes PA6 track PA7, an unseated
     * one leaves it on the pull-down, and a pin held high with PA7 low means
     * something else is still attached to D12.
     */
    loopback_set_gpio_mode();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(2);
    const bool high_follows =
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(2);
    const bool low_follows =
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET;
    result->continuity = high_follows && low_follows;
    result->miso_driven = !low_follows;
    loopback_restore_spi_mode();

    if (!result->continuity) {
        return;
    }

    for (size_t index = 0U; index < sizeof(pattern); ++index) {
        uint8_t out = pattern[index];
        uint8_t in = 0U;

        if (HAL_SPI_TransmitReceive(imu->spi, &out, &in, 1U, 100U) != HAL_OK) {
            result->failed_byte = (int)index;
            result->sent = out;
            return;
        }
        if (in != out) {
            result->failed_byte = (int)index;
            result->sent = out;
            result->received = in;
            return;
        }
    }
}

bool bno086_read_attitude(void *context,
                          int16_t *roll_tenths,
                          int16_t *pitch_tenths)
{
    Bno086 *imu = (Bno086 *)context;

    if (imu == NULL || roll_tenths == NULL || pitch_tenths == NULL) {
        return false;
    }

    /*
     * Refresh here as well as from the main loop.  Gait moves block the main
     * loop for seconds at a time, and a balance step acting on a stale angle
     * is worse than one that pays ~100us for a pending 21-byte report.  With
     * nothing pending this costs a single GPIO read.
     */
    bno086_service(imu);

    if (!imu->has_attitude) {
        return false;
    }

    *roll_tenths = imu->roll_tenths;
    *pitch_tenths = imu->pitch_tenths;
    return true;
}

const char *bno086_result_string(Bno086Result result)
{
    switch (result) {
    case BNO086_OK:
        return "ok";
    case BNO086_NOT_PRESENT:
        return "not present";
    case BNO086_TIMEOUT:
        return "timeout";
    case BNO086_SPI_ERROR:
        return "SPI error";
    case BNO086_PROTOCOL_ERROR:
        return "protocol error";
    default:
        return "unknown";
    }
}
