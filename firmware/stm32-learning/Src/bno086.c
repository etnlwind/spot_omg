#include "bno086.h"

#include <math.h>
#include <string.h>

/*
 * Minimal SHTP/SH-2 client for the BNO086 over SPI.
 *
 * Only what the balance loop needs is implemented: reset the part, subscribe
 * to one rotation report, and decode its quaternion.  CEVA's full sh2 driver
 * covers dozens of reports this robot does not use, so it is not vendored in.
 *
 * Game Rotation Vector (0x08) is used rather than Rotation Vector (0x05)
 * because the latter folds in the magnetometer.  Twelve STS3215 servos put
 * permanent magnets a few centimetres from the sensor, so a magnetic heading
 * would be worse than none.  The balance loop only consumes roll and pitch,
 * which the gyro/accel fusion supplies without magnetometer help; the yaw this
 * report produces is relative to power-on rather than to north.
 */

/* SHTP channels. */
#define SHTP_CHANNEL_COMMAND     0U
#define SHTP_CHANNEL_EXECUTABLE  1U
#define SHTP_CHANNEL_CONTROL     2U
#define SHTP_CHANNEL_REPORTS     3U

#define SHTP_HEADER_SIZE         4U
#define SHTP_MAX_PACKET          384U

/* SH-2 report identifiers. */
#define SH2_SET_FEATURE_COMMAND  0xFDU
#define SH2_BASE_TIMESTAMP       0xFBU
#define SH2_GAME_ROTATION_VECTOR 0x08U

/* Length of the timestamp prologue that opens every channel 3 packet. */
#define SH2_TIMESTAMP_SIZE       5U
#define SH2_GRV_REPORT_SIZE      12U

#define BNO086_INT_TIMEOUT_MS    200U
#define BNO086_RESET_DRAIN_MS    600U

/*
 * Board wiring.  Kept here rather than in main.h so the whole IMU bring-up is
 * one file and a CubeMX regeneration cannot drop it.
 */
#define BNO086_CS_PORT    GPIOB
#define BNO086_CS_PIN     GPIO_PIN_6
#define BNO086_INT_PORT   GPIOA
#define BNO086_INT_PIN    GPIO_PIN_8
#define BNO086_RST_PORT   GPIOB
#define BNO086_RST_PIN    GPIO_PIN_3
#define BNO086_WAKE_PORT  GPIOB
#define BNO086_WAKE_PIN   GPIO_PIN_5

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
    return HAL_GPIO_ReadPin(BNO086_INT_PORT, BNO086_INT_PIN) == GPIO_PIN_RESET;
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

static void configure_pins(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* Park the outputs before they are driven so the part is not half woken. */
    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
    gpio_write(BNO086_RST_PORT, BNO086_RST_PIN, false);
    gpio_write(BNO086_WAKE_PORT, BNO086_WAKE_PIN, true);

    /* PA5 SCK, PA6 MISO, PA7 MOSI. */
    init.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &init);

    init.Alternate = 0;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;

    init.Pin = BNO086_CS_PIN;
    HAL_GPIO_Init(BNO086_CS_PORT, &init);
    init.Pin = BNO086_RST_PIN;
    HAL_GPIO_Init(BNO086_RST_PORT, &init);
    init.Pin = BNO086_WAKE_PIN;
    HAL_GPIO_Init(BNO086_WAKE_PORT, &init);

    /*
     * H_INTN is sampled rather than wired to EXTI.  The main loop and the
     * attitude reader both call bno086_service(), so a level test costs one
     * GPIO read and needs no shared state with an ISR.
     */
    init.Pin = BNO086_INT_PIN;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BNO086_INT_PORT, &init);
}

static Bno086Result configure_spi(Bno086 *imu)
{
    imu->spi.Instance = SPI1;
    imu->spi.Init.Mode = SPI_MODE_MASTER;
    imu->spi.Init.Direction = SPI_DIRECTION_2LINES;
    imu->spi.Init.DataSize = SPI_DATASIZE_8BIT;
    /* The BNO086 samples on the trailing edge of an idle-high clock: mode 3. */
    imu->spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    imu->spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    imu->spi.Init.NSS = SPI_NSS_SOFT;
    /* PCLK2 is 16 MHz, so /8 gives 2 MHz against the part's 3 MHz ceiling. */
    imu->spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    imu->spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    imu->spi.Init.TIMode = SPI_TIMODE_DISABLE;
    imu->spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    imu->spi.Init.CRCPolynomial = 10;

    return HAL_SPI_Init(&imu->spi) == HAL_OK ? BNO086_OK : BNO086_SPI_ERROR;
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

        if (HAL_SPI_TransmitReceive(&imu->spi,
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
 * Read one SHTP packet.  Returns BNO086_OK with *payload_length set to zero
 * when the sensor has nothing queued.
 */
static Bno086Result shtp_receive(Bno086 *imu,
                                 uint8_t *payload,
                                 size_t payload_capacity,
                                 size_t *payload_length,
                                 uint8_t *channel)
{
    uint8_t header[SHTP_HEADER_SIZE];

    *payload_length = 0U;
    *channel = 0U;

    if (!interrupt_asserted()) {
        return BNO086_OK;
    }

    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, false);

    Bno086Result result = spi_read(imu, header, sizeof(header));
    if (result != BNO086_OK) {
        gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
        return result;
    }

    /* Bit 15 flags a continuation of an oversized transfer, not length. */
    const uint16_t total =
        (uint16_t)(((uint16_t)header[1] << 8) | header[0]) & 0x7FFFU;
    *channel = header[2];

    if (total <= SHTP_HEADER_SIZE || total > SHTP_MAX_PACKET) {
        gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
        /* A zero-length header is the idle answer, not a fault. */
        return total == 0U ? BNO086_OK : BNO086_PROTOCOL_ERROR;
    }

    const size_t body = (size_t)total - SHTP_HEADER_SIZE;
    const size_t wanted = body < payload_capacity ? body : payload_capacity;

    result = spi_read(imu, payload, wanted);
    if (result == BNO086_OK && body > wanted) {
        /* Advertisement packets are far longer than anything we decode. */
        result = spi_read(imu, NULL, body - wanted);
    }

    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
    if (result != BNO086_OK) {
        return result;
    }

    *payload_length = wanted;
    return BNO086_OK;
}

static Bno086Result shtp_send(Bno086 *imu,
                              uint8_t channel,
                              const uint8_t *payload,
                              size_t payload_length)
{
    uint8_t packet[SHTP_HEADER_SIZE + 24U];
    const size_t total = SHTP_HEADER_SIZE + payload_length;

    if (channel >= (uint8_t)(sizeof(imu->sequence) / sizeof(imu->sequence[0])) ||
        total > sizeof(packet)) {
        return BNO086_PROTOCOL_ERROR;
    }

    packet[0] = (uint8_t)(total & 0xFFU);
    packet[1] = (uint8_t)(total >> 8);
    packet[2] = channel;
    packet[3] = imu->sequence[channel]++;
    memcpy(&packet[SHTP_HEADER_SIZE], payload, payload_length);

    /*
     * WAKE (PS0 in SPI mode) asks the sensor for a transfer window; it answers
     * by asserting H_INTN.  Writing before that window is dropped.
     */
    gpio_write(BNO086_WAKE_PORT, BNO086_WAKE_PIN, false);
    const bool ready = wait_for_interrupt(BNO086_INT_TIMEOUT_MS);
    if (!ready) {
        gpio_write(BNO086_WAKE_PORT, BNO086_WAKE_PIN, true);
        return BNO086_TIMEOUT;
    }

    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, false);
    const HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&imu->spi, packet, (uint16_t)total, 100U);
    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
    gpio_write(BNO086_WAKE_PORT, BNO086_WAKE_PIN, true);

    return status == HAL_OK ? BNO086_OK : BNO086_SPI_ERROR;
}

static Bno086Result enable_game_rotation_vector(Bno086 *imu,
                                                uint32_t interval_us)
{
    uint8_t command[17] = {0};

    command[0] = SH2_SET_FEATURE_COMMAND;
    command[1] = SH2_GAME_ROTATION_VECTOR;
    /* [2] feature flags, [3..4] change sensitivity: defaults of zero. */
    command[5] = (uint8_t)(interval_us & 0xFFU);
    command[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    command[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    command[8] = (uint8_t)((interval_us >> 24) & 0xFFU);
    /* [9..12] batch interval, [13..16] sensor-specific config: zero. */

    return shtp_send(imu, SHTP_CHANNEL_CONTROL, command, sizeof(command));
}

static void update_angles(Bno086 *imu)
{
    const float i = (float)imu->quat_i * BNO086_QUAT_SCALE;
    const float j = (float)imu->quat_j * BNO086_QUAT_SCALE;
    const float k = (float)imu->quat_k * BNO086_QUAT_SCALE;
    const float w = (float)imu->quat_real * BNO086_QUAT_SCALE;

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

static void parse_sensor_reports(Bno086 *imu,
                                 const uint8_t *payload,
                                 size_t length)
{
    if (length < SH2_TIMESTAMP_SIZE || payload[0] != SH2_BASE_TIMESTAMP) {
        return;
    }

    size_t offset = SH2_TIMESTAMP_SIZE;
    while (offset < length) {
        if (payload[offset] != SH2_GAME_ROTATION_VECTOR) {
            /* Only one report is subscribed, so anything else ends the scan. */
            return;
        }
        if (length - offset < SH2_GRV_REPORT_SIZE) {
            return;
        }

        imu->quat_i = (int16_t)((uint16_t)payload[offset + 4] |
                                ((uint16_t)payload[offset + 5] << 8));
        imu->quat_j = (int16_t)((uint16_t)payload[offset + 6] |
                                ((uint16_t)payload[offset + 7] << 8));
        imu->quat_k = (int16_t)((uint16_t)payload[offset + 8] |
                                ((uint16_t)payload[offset + 9] << 8));
        imu->quat_real = (int16_t)((uint16_t)payload[offset + 10] |
                                   ((uint16_t)payload[offset + 11] << 8));

        update_angles(imu);
        imu->report_count++;
        imu->last_report_tick = HAL_GetTick();
        offset += SH2_GRV_REPORT_SIZE;
    }
}

void bno086_service(Bno086 *imu)
{
    uint8_t payload[64];

    if (imu == NULL || !imu->present) {
        return;
    }

    /* Drain rather than read once, so a burst cannot leave H_INTN asserted. */
    for (unsigned int guard = 0U; guard < 8U; ++guard) {
        size_t length = 0U;
        uint8_t channel = 0U;

        if (!interrupt_asserted()) {
            return;
        }
        if (shtp_receive(imu, payload, sizeof(payload), &length, &channel) !=
            BNO086_OK) {
            imu->protocol_errors++;
            return;
        }
        if (length == 0U) {
            return;
        }
        if (channel == SHTP_CHANNEL_REPORTS) {
            parse_sensor_reports(imu, payload, length);
        }
    }
}

Bno086Result bno086_init(Bno086 *imu, uint32_t report_interval_us)
{
    uint8_t payload[64];

    if (imu == NULL || report_interval_us == 0U) {
        return BNO086_PROTOCOL_ERROR;
    }

    memset(imu, 0, sizeof(*imu));
    configure_pins();

    Bno086Result result = configure_spi(imu);
    if (result != BNO086_OK) {
        return result;
    }

    /* Hold reset low well past the datasheet minimum, then let it boot. */
    gpio_write(BNO086_RST_PORT, BNO086_RST_PIN, false);
    HAL_Delay(20);
    gpio_write(BNO086_RST_PORT, BNO086_RST_PIN, true);

    /*
     * A healthy part announces itself unprompted: an advertisement on channel
     * 0 and a reset-complete on channel 1.  Seeing any packet is what tells us
     * the part is wired and in SPI mode, so drain until one arrives.
     */
    bool saw_packet = false;
    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < BNO086_RESET_DRAIN_MS) {
        size_t length = 0U;
        uint8_t channel = 0U;

        if (!interrupt_asserted()) {
            continue;
        }
        if (shtp_receive(imu, payload, sizeof(payload), &length, &channel) !=
            BNO086_OK) {
            imu->protocol_errors++;
            continue;
        }
        if (length != 0U) {
            saw_packet = true;
        }
    }

    if (!saw_packet) {
        return BNO086_NOT_PRESENT;
    }

    imu->present = true;
    result = enable_game_rotation_vector(imu, report_interval_us);
    if (result != BNO086_OK) {
        imu->present = false;
        return result;
    }

    /* Give the first reports a chance to land so callers can trust the cache. */
    const uint32_t subscribed_at = HAL_GetTick();
    while (!imu->has_attitude &&
           (uint32_t)(HAL_GetTick() - subscribed_at) < BNO086_INT_TIMEOUT_MS) {
        bno086_service(imu);
    }

    return imu->has_attitude ? BNO086_OK : BNO086_TIMEOUT;
}

/* Sample H_INTN as a plain input, mirroring what linestate does for PA9/PA10. */
static uint32_t probe_int_percent(uint32_t pull)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = BNO086_INT_PIN;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BNO086_INT_PORT, &init);
    HAL_Delay(2);

    uint32_t high = 0U;
    uint32_t samples = 0U;
    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < 20U) {
        if (HAL_GPIO_ReadPin(BNO086_INT_PORT, BNO086_INT_PIN) == GPIO_PIN_SET) {
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
    gpio_write(BNO086_RST_PORT, BNO086_RST_PIN, false);
    HAL_Delay(5);
    probe->int_in_reset_percent = probe_int_percent(GPIO_PULLDOWN);

    GPIO_InitTypeDef init = {0};
    init.Pin = BNO086_INT_PIN;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BNO086_INT_PORT, &init);

    HAL_Delay(20);
    gpio_write(BNO086_RST_PORT, BNO086_RST_PIN, true);

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

        gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, false);
        const Bno086Result read = spi_read(imu, header, sizeof(header));
        gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
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

    if (!probe->int_asserted) {
        return;
    }

    /*
     * Read the header only.  Its bytes tell wiring apart from protocol: all
     * 0x00 or all 0xFF means MISO is not carrying data, while a sane length
     * and channel means the link is up and the fault is further along.
     */
    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, false);
    const Bno086Result result =
        spi_read(imu, probe->header, sizeof(probe->header));
    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);
    probe->header_read = result == BNO086_OK;
}

int bno086_loopback_test(Bno086 *imu, uint8_t *sent, uint8_t *received)
{
    static const uint8_t pattern[] = {
        0x00U, 0xFFU, 0x55U, 0xAAU, 0x01U, 0x7EU, 0x81U, 0x42U
    };

    if (imu == NULL || sent == NULL || received == NULL) {
        return 0;
    }

    /* Keep the sensor deselected so a still-attached part cannot answer. */
    gpio_write(BNO086_CS_PORT, BNO086_CS_PIN, true);

    for (size_t index = 0U; index < sizeof(pattern); ++index) {
        uint8_t out = pattern[index];
        uint8_t in = 0U;

        if (HAL_SPI_TransmitReceive(&imu->spi, &out, &in, 1U, 100U) != HAL_OK) {
            *sent = out;
            *received = 0U;
            return (int)index;
        }
        if (in != out) {
            *sent = out;
            *received = in;
            return (int)index;
        }
    }

    return -1;
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
