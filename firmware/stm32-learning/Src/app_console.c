#include "app_console.h"

#include "feetech_protocol.h"
#include "gait_policy.h"
#include "robot_config.h"
#include "safety.h"
#include "sts3215.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_SAFE_MOVE_DELTA 256U
#define UARTTEST_TIMEOUT_MS     10U
#define BUSPROBE_WINDOW_MS      50U
#define LINESTATE_WINDOW_MS     20U
#define LINESTATE_SETTLE_MS      2U

static void write_text(AppConsole *console, const char *text)
{
    if (console == NULL || console->uart == NULL || text == NULL) {
        return;
    }

    (void)HAL_UART_Transmit(console->uart,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HAL_MAX_DELAY);
}

static void echo_byte(AppConsole *console, uint8_t byte)
{
    if (console == NULL || console->uart == NULL || !console->echo_enabled) {
        return;
    }

    while (__HAL_UART_GET_FLAG(console->uart, UART_FLAG_TXE) == RESET) {
    }
    console->uart->Instance->DR = byte;
}

static void echo_text(AppConsole *console, const char *text)
{
    while (text != NULL && *text != '\0') {
        echo_byte(console, (uint8_t)*text++);
    }
}

static bool parse_u32(const char *text,
                      uint32_t minimum,
                      uint32_t maximum,
                      uint32_t *value)
{
    char *end = NULL;

    if (text == NULL || value == NULL || text[0] == '\0') {
        return false;
    }

    unsigned long parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

/* Defined below with the other safety helpers; used by print_robot_result. */
static void print_safety_fault(AppConsole *console);

static void print_bus_result(AppConsole *console,
                             uint8_t servo_id,
                             ServoBusResult result)
{
    char message[96];
    const uint8_t error = console->robot->bus->last_servo_error;

    (void)snprintf(message,
                   sizeof(message),
                   "ID %u: %s, servo_error=0x%02X\r\n",
                   (unsigned int)servo_id,
                   servo_bus_result_string(result),
                   (unsigned int)error);
    write_text(console, message);
}

static void print_robot_result(AppConsole *console, RobotResult result)
{
    char message[128];

    if (result == ROBOT_OK) {
        write_text(console, "OK\r\n");
        return;
    }
    if (result == ROBOT_SAFETY_FAULT) {
        print_safety_fault(console);
        write_text(console,
                   "ERROR: safety fault; torque off, run recover\r\n");
        return;
    }
    if (result == ROBOT_SERVO_POWER_LOST) {
        write_text(console,
                   "ERROR: servo power lost; restore the 12V supply then "
                   "run recover\r\n");
        return;
    }
    if (result == ROBOT_IMU_ERROR) {
        write_text(console,
                   "ERROR: IMU balance error; stand target requested\r\n");
        return;
    }
    if (result == ROBOT_ACTUATOR_PROFILE_ERROR) {
        write_text(console,
                   "ERROR: trot3 requires profile 3400 254; no motion started\r\n");
        return;
    }
    if (result == ROBOT_TROT3_PERIOD_ERROR) {
        write_text(console,
                   "ERROR: trot3 period must be 600..1800ms\r\n");
        return;
    }
    if (result == ROBOT_MOTION_ABORTED) {
        write_text(console, "STOPPED: stand target requested\r\n");
        return;
    }
    if (result == ROBOT_TILT_LIMIT) {
        write_text(console,
                   "ERROR: motion tilt safety limit reached; stand target "
                   "requested\r\n");
        return;
    }

    (void)snprintf(
        message,
        sizeof(message),
        "ERROR: %s; servo=%u, bus=%s, servo_error=0x%02X\r\n",
        robot_result_string(result),
        (unsigned int)console->robot->last_failed_servo_id,
        servo_bus_result_string(console->robot->last_bus_result),
        (unsigned int)console->robot->bus->last_servo_error);
    write_text(console, message);
}

static void command_ping(AppConsole *console, char *id_text)
{
    uint32_t id = 0U;
    if (!parse_u32(id_text, 1U, 253U, &id)) {
        write_text(console, "usage: ping ID\r\n");
        return;
    }

    ServoBusResult result = sts3215_ping(console->robot->bus, (uint8_t)id);
    print_bus_result(console, (uint8_t)id, result);
}

static void command_scan(AppConsole *console)
{
    write_text(console, "Scanning configured IDs 1..12\r\n");
    for (uint8_t id = 1U; id <= ROBOT_JOINT_COUNT; ++id) {
        ServoBusResult result = sts3215_ping(console->robot->bus, id);
        if (result == SERVO_BUS_OK) {
            char message[24];
            (void)snprintf(message,
                           sizeof(message),
                           "  ID %u OK\r\n",
                           (unsigned int)id);
            write_text(console, message);
        } else {
            print_bus_result(console, id, result);
        }
    }
}

static void command_uarttest(AppConsole *console)
{
    static const uint8_t test_bytes[] = {
        0x00U, 0xFFU, 0x55U, 0xAAU, 0x01U, 0x7EU, 0x81U, 0x42U
    };
    ServoBus *bus = console->robot->bus;
    UART_HandleTypeDef *uart = bus->uart;

    write_text(console,
               "USART1 loopback test: disconnect URT-2 and connect "
               "PA9 directly to PA10\r\n");

    __HAL_UART_CLEAR_OREFLAG(uart);
    while (__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE) != RESET) {
        volatile uint32_t discarded = uart->Instance->DR;
        (void)discarded;
    }

    for (size_t index = 0U; index < sizeof(test_bytes); ++index) {
        const uint8_t transmitted = test_bytes[index];
        uint8_t received = 0U;
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            uart, (uint8_t *)&test_bytes[index], 1U, UARTTEST_TIMEOUT_MS);
        if (status != HAL_OK) {
            char message[80];
            (void)snprintf(message,
                           sizeof(message),
                           "UARTTEST FAIL: TX status=%u at byte %u\r\n",
                           (unsigned int)status,
                           (unsigned int)index);
            write_text(console, message);
            return;
        }

        status = HAL_UART_Receive(uart,
                                  &received,
                                  1U,
                                  UARTTEST_TIMEOUT_MS);
        if (status != HAL_OK) {
            char message[80];
            (void)snprintf(message,
                           sizeof(message),
                           "UARTTEST FAIL: RX status=%u at byte %u\r\n",
                           (unsigned int)status,
                           (unsigned int)index);
            write_text(console, message);
            return;
        }
        if (received != transmitted) {
            char message[96];
            (void)snprintf(message,
                           sizeof(message),
                           "UARTTEST FAIL: byte %u sent=0x%02X received=0x%02X\r\n",
                           (unsigned int)index,
                           (unsigned int)transmitted,
                           (unsigned int)received);
            write_text(console, message);
            return;
        }
    }

    write_text(console, "UARTTEST PASS: USART1 PA9/PA10 loopback OK\r\n");
}

static void command_busprobe(AppConsole *console, char *id_text)
{
    uint32_t id = 0U;
    uint8_t packet[FEETECH_PACKET_CAPACITY];
    uint8_t received[FEETECH_PACKET_CAPACITY];
    size_t received_count = 0U;

    if (!parse_u32(id_text, 1U, 253U, &id)) {
        write_text(console, "usage: busprobe ID\r\n");
        return;
    }

    const size_t packet_length = feetech_encode_instruction(
        (uint8_t)id,
        FEETECH_INST_PING,
        NULL,
        0U,
        packet,
        sizeof(packet));
    ServoBus *bus = console->robot->bus;
    UART_HandleTypeDef *uart = bus->uart;

    __HAL_UART_CLEAR_OREFLAG(uart);
    while (__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE) != RESET) {
        volatile uint32_t discarded = uart->Instance->DR;
        (void)discarded;
    }

    CLEAR_BIT(uart->Instance->CR1, USART_CR1_RE);
    HAL_StatusTypeDef status = HAL_UART_Transmit(uart,
                                                packet,
                                                (uint16_t)packet_length,
                                                BUSPROBE_WINDOW_MS);
    SET_BIT(uart->Instance->CR1, USART_CR1_RE);
    if (status != HAL_OK) {
        char message[64];
        (void)snprintf(message,
                       sizeof(message),
                       "BUSPROBE FAIL: TX status=%u\r\n",
                       (unsigned int)status);
        write_text(console, message);
        return;
    }

    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < BUSPROBE_WINDOW_MS &&
           received_count < sizeof(received)) {
        const uint32_t uart_status = uart->Instance->SR;
        if ((uart_status & USART_SR_RXNE) != 0U) {
            received[received_count++] = (uint8_t)uart->Instance->DR;
        } else if ((uart_status & (USART_SR_ORE | USART_SR_NE |
                                  USART_SR_FE | USART_SR_PE)) != 0U) {
            volatile uint32_t discarded_status = uart->Instance->SR;
            volatile uint32_t discarded_data = uart->Instance->DR;
            (void)discarded_status;
            (void)discarded_data;
            write_text(console, "BUSPROBE FAIL: USART1 receive error\r\n");
            return;
        }
    }

    if (received_count == 0U) {
        write_text(console, "BUSPROBE RX: no bytes\r\n");
        return;
    }

    char message[3U * FEETECH_PACKET_CAPACITY + 32U];
    int used = snprintf(message,
                        sizeof(message),
                        "BUSPROBE RX (%u):",
                        (unsigned int)received_count);
    for (size_t index = 0U;
         index < received_count && used > 0 && (size_t)used < sizeof(message);
         ++index) {
        used += snprintf(&message[used],
                         sizeof(message) - (size_t)used,
                         " %02X",
                         (unsigned int)received[index]);
    }
    if (used > 0 && (size_t)used < sizeof(message)) {
        (void)snprintf(&message[used], sizeof(message) - (size_t)used, "\r\n");
    }
    write_text(console, message);
}

/* Sample one pin as a plain input with the requested pull setting. */
static uint32_t linestate_sample_percent(uint16_t pin, uint32_t pull)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = pull;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &init);
    HAL_Delay(LINESTATE_SETTLE_MS);

    uint32_t high = 0U;
    uint32_t samples = 0U;
    const uint32_t started_at = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_at) < LINESTATE_WINDOW_MS) {
        if (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET) {
            ++high;
        }
        ++samples;
    }
    return samples == 0U ? 0U : (high * 100U) / samples;
}

static void linestate_restore_usart1_pins(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &init);
}

/*
 * Probe the USART1 pins to tell a driven wire from a disconnected one.
 *
 * An idle UART line sits HIGH, but so does a floating input, so reading the
 * pad alone proves nothing.  Applying a pull-down settles it: a pin that
 * something is actively driving stays HIGH, while an open one follows the
 * pull-down to LOW.
 *
 * What this command cannot do is separate correct wiring from swapped wiring.
 * The URT-2 output idles HIGH and its input carries a pull-up, so whichever
 * way round the two wires go, both pins read HIGH under the pull-down.  An
 * unpowered URT-2 parasitically pulled high through PA9 and its input ESD
 * diode reads the same way again.  Three states, one measurement.
 *
 * Measured on 2026-08-08, both pins high is what a HEALTHY bus reads:
 *
 *   correct wiring, scan 12/12 OK -> PA10 float=100% pulldown=100% | PA9 100%
 *   crossed wiring, scan 12 fail  -> PA10 float=100% pulldown=100% | PA9 100%
 *
 * Identical.  Do not read this branch as a fault; the earlier verdict that
 * called it an unpowered URT-2 was firing on a normal bus.
 *
 * So when both pins read high, do not trust a verdict from here -- run the
 * two free hardware checks instead, in this order:
 *
 *   1. Watch the URT-2 TX1 LED during "scan".  If it never blinks the URT-2
 *      is not receiving the request at all, which rules the return path out.
 *   2. Swap the two signal wires at the URT-2 header and rerun "scan".
 *
 * That swap is what actually fixed the 2026-08-08 silent bus: PA9 and PA10
 * were crossed, so the STM32 was driving the URT-2 output pin while the
 * URT-2 input sat unconnected on the MCU receive pin.  Swapping cannot make
 * things worse, because a crossed pair already has two outputs fighting.
 *
 * The pins are returned to USART1 alternate function before returning.  The
 * servo bus is idle while a console command runs, so nothing is interrupted.
 */
static void command_linestate(AppConsole *console)
{
    char message[128];

    const uint32_t rx_float = linestate_sample_percent(
        GPIO_PIN_10, GPIO_NOPULL);
    const uint32_t rx_down = linestate_sample_percent(
        GPIO_PIN_10, GPIO_PULLDOWN);
    const uint32_t tx_down = linestate_sample_percent(
        GPIO_PIN_9, GPIO_PULLDOWN);
    linestate_restore_usart1_pins();

    (void)snprintf(
        message,
        sizeof(message),
        "LINESTATE PA10 float=%lu%% pulldown=%lu%% | PA9 pulldown=%lu%%\r\n",
        (unsigned long)rx_float,
        (unsigned long)rx_down,
        (unsigned long)tx_down);
    write_text(console, message);

    if (rx_down >= 90U && tx_down >= 90U) {
        /*
         * Correct wiring, swapped wiring and an unpowered URT-2 all read this
         * way, so this branch reports the tie rather than picking a winner.
         */
        write_text(console,
                   "Both pins held high: this is also what a working bus "
                   "reads, so it is not a fault on its own. Run 'scan'; if it "
                   "fails, watch the URT-2 TX1 LED, then swap the two signal "
                   "wires at the URT-2 header and rerun 'scan'\r\n");
    } else if (rx_down >= 90U) {
        write_text(console,
                   "PA10 driven high: URT-2 TX reaches the MCU; the request "
                   "path PA9 -> URT-2 is the remaining suspect\r\n");
    } else if (rx_down <= 10U) {
        write_text(console,
                   "PA10 follows the pull-down: nothing drives it; the RX "
                   "wire, GND or URT-2 logic power is open\r\n");
    } else {
        write_text(console,
                   "PA10 unstable under pull-down: weak drive or noise; "
                   "check GND and the 3.3V level switch\r\n");
    }

    if (tx_down >= 90U && rx_down < 90U) {
        write_text(console,
                   "PA9 is being driven by something: suspect TX and RX "
                   "swapped, or a URT-2 pull-up on that pin\r\n");
    }
}

static void command_read(AppConsole *console, char *id_text)
{
    uint32_t id = 0U;
    if (!parse_u32(id_text, 1U, 253U, &id)) {
        write_text(console, "usage: read ID\r\n");
        return;
    }

    Sts3215State state;
    ServoBusResult result = sts3215_read_state(console->robot->bus,
                                               (uint8_t)id,
                                               &state);
    if (result != SERVO_BUS_OK) {
        print_bus_result(console, (uint8_t)id, result);
        return;
    }

    char message[160];
    (void)snprintf(message,
                   sizeof(message),
                   "ID %lu pos=%u speed=%d load=%d voltage=%umV temp=%uC "
                   "current=%d moving=%u hw=0x%02X\r\n",
                   (unsigned long)id,
                   (unsigned int)state.position,
                   (int)state.speed,
                   (int)state.load,
                   (unsigned int)state.voltage_mv,
                   (unsigned int)state.temperature_c,
                   (int)state.current,
                   state.moving ? 1U : 0U,
                   (unsigned int)state.hardware_error);
    write_text(console, message);
}

static void command_move(AppConsole *console,
                         char *id_text,
                         char *position_text)
{
    uint32_t id = 0U;
    uint32_t position = 0U;
    if (!parse_u32(id_text, 1U, 12U, &id) ||
        !parse_u32(position_text, 0U, STS3215_MAX_POSITION, &position)) {
        write_text(console, "usage: move ID POSITION; ID=1..12\r\n");
        return;
    }

    RobotResult result = robot_move_single_safe(
        console->robot,
        (uint8_t)id,
        (uint16_t)position,
        CONSOLE_SAFE_MOVE_DELTA);
    print_robot_result(console, result);
}

static void command_targets(AppConsole *console)
{
    uint16_t targets[ROBOT_JOINT_COUNT];
    if (!robot_stand_targets(targets)) {
        write_text(console, "ERROR: stand target configuration\r\n");
        return;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        char message[48];
        (void)snprintf(message,
                       sizeof(message),
                       "ID %u target=%u\r\n",
                       (unsigned int)g_robot_servo_ids[index],
                       (unsigned int)targets[index]);
        write_text(console, message);
    }
}

/*
 * Bring-up probe for the BNO086, the IMU counterpart of busprobe/linestate.
 *
 * bno086_init() reports every failure as "not present", which does not say
 * whether the part is dead, mis-wired or answering on I2C because PS1 was
 * left low.  Pulsing reset and watching H_INTN separates those.
 */
static void command_i2cscan(AppConsole *console)
{
    uint8_t found[8];
    char message[64];

    if (console->imu055 == NULL || console->imu055->i2c == NULL) {
        write_text(console, "ERROR: I2C scan unavailable\r\n");
        return;
    }

    const uint8_t count =
        bno055_scan(console->imu055->i2c, found, (uint8_t)sizeof(found));
    if (count == 0U) {
        write_text(console,
                   "I2CSCAN: no devices. Check SDA on D14/PB9, SCL on "
                   "D15/PB8, 3V3 and GND\r\n");
        return;
    }

    (void)snprintf(message, sizeof(message), "I2CSCAN found %u:",
                   (unsigned int)count);
    write_text(console, message);
    for (uint8_t index = 0U; index < count && index < sizeof(found); ++index) {
        (void)snprintf(message, sizeof(message), " 0x%02X",
                       (unsigned int)found[index]);
        write_text(console, message);
    }
    write_text(console, "\r\n");
    write_text(console,
               "  A BNO055 answers at 0x28 or 0x29 with chip id 0xA0\r\n");
}

static void command_spitest(AppConsole *console)
{
    Bno086Loopback result;
    char message[128];

    if (console->imu == NULL) {
        write_text(console, "ERROR: SPI loopback unavailable\r\n");
        return;
    }

    write_text(console,
               "SPI1 loopback test: disconnect the BNO086 and connect "
               "D11/PA7 directly to D12/PA6\r\n");

    bno086_loopback_test(console->imu, &result);

    if (!result.continuity) {
        /*
         * Say nothing about SPI1 here.  An unseated jumper fails the byte
         * pattern exactly like a broken peripheral, so a verdict at this
         * point would point at the wrong side of the link.
         */
        write_text(console,
                   "SPITEST FAIL: D11 and D12 are not connected. PA6 did not "
                   "follow PA7 driven as plain GPIO, so the jumper is not "
                   "conducting and SPI1 is untested\r\n");
        if (result.miso_driven) {
            write_text(console,
                       "  PA6 also held high against a pull-down: something "
                       "is still attached to D12. Remove the sensor's SO wire "
                       "before fitting the jumper\r\n");
        }
        return;
    }

    if (result.failed_byte < 0) {
        write_text(console,
                   "SPITEST PASS: SPI1 PA5/PA6/PA7 loopback OK; the MCU side "
                   "is good, so suspect the sensor or its wiring\r\n");
        return;
    }

    (void)snprintf(message,
                   sizeof(message),
                   "SPITEST FAIL: byte %d sent=0x%02X received=0x%02X\r\n",
                   result.failed_byte,
                   (unsigned int)result.sent,
                   (unsigned int)result.received);
    write_text(console, message);
    write_text(console,
               "  The jumper conducts, so this is SPI1 itself: check that PA5 "
               "is not still driven as the LD2 output\r\n");
}

static const char *leg_name(uint8_t leg_index)
{
    static const char *const names[4] = {"FL", "FR", "RL", "RR"};
    return leg_index < 4U ? names[leg_index] : "??";
}

static uint32_t ticks_to_tenths_degrees(uint32_t ticks)
{
    return (ticks * 3600U + 2048U) / 4096U;
}

static void support_mask_text(uint8_t mask, char text[16])
{
    size_t used = 0U;
    text[0] = '\0';
    for (uint8_t leg = 0U; leg < 4U; ++leg) {
        if ((mask & (uint8_t)(1U << leg)) == 0U) {
            continue;
        }
        const int written = snprintf(text + used,
                                     16U - used,
                                     "%s%s",
                                     used == 0U ? "" : "+",
                                     leg_name(leg));
        if (written < 0 || (size_t)written >= 16U - used) {
            break;
        }
        used += (size_t)written;
    }
    if (used == 0U) {
        (void)snprintf(text, 16U, "none");
    }
}

/*
 * Print everything about the joint that faulted.
 *
 * A stall is over by the time anyone reads this, and the servo has usually
 * been powered down since, so the report is the only evidence left: which
 * joint, how far it was from its target, how hard it was pushing, how long it
 * held, and where in the stride it happened.
 */
static void print_safety_fault(AppConsole *console)
{
    const SafetyMonitor *safety = &console->robot->safety;
    const SafetyFaultRecord *record = &safety->record;
    char message[128];

    (void)snprintf(message,
                   sizeof(message),
                   "SAFETY %s\r\n  leg=%s joint=J%u id=%u\r\n",
                   safety_fault_string(safety->fault),
                   leg_name(record->leg_index),
                   (unsigned int)record->joint_index,
                   (unsigned int)record->servo_id);
    write_text(console, message);

    (void)snprintf(message,
                   sizeof(message),
                   "  target=%u actual=%u error=%u\r\n",
                   (unsigned int)record->target,
                   (unsigned int)record->actual,
                   (unsigned int)record->position_error);
    write_text(console, message);

    (void)snprintf(message,
                   sizeof(message),
                   "  load=%d current=%d temp=%uC hw_error=0x%02X\r\n",
                   (int)record->load,
                   (int)record->current,
                   (unsigned int)record->temperature_c,
                   (unsigned int)record->hardware_error);
    write_text(console, message);

    (void)snprintf(message,
                   sizeof(message),
                   "  duration=%ums gait_phase=%u action=TORQUE_OFF_ALL\r\n",
                   (unsigned int)record->duration_ms,
                   (unsigned int)record->gait_phase);
    write_text(console, message);
}

static void command_safety(AppConsole *console)
{
    const SafetyMonitor *safety = &console->robot->safety;
    char message[128];

    (void)snprintf(
        message,
        sizeof(message),
        "Safety: %s, peak_error=%uticks, candidates=%u, bad_samples=%u\r\n",
        safety_fault_string(safety->fault),
        (unsigned int)safety->peak_position_error,
        (unsigned int)safety->stall_candidates_seen,
        (unsigned int)safety->implausible_samples);
    write_text(console, message);

    (void)snprintf(
        message,
        sizeof(message),
        "Limits: error>=%uticks load>=%u current>=%u for %ums, temp>=%uC\r\n",
        (unsigned int)safety->limits.position_error_ticks,
        (unsigned int)safety->limits.load_magnitude,
        (unsigned int)safety->limits.current_magnitude,
        (unsigned int)safety->limits.sustain_ms,
        (unsigned int)safety->limits.temperature_limit_c);
    write_text(console, message);

    if (safety_is_faulted(safety)) {
        print_safety_fault(console);
        write_text(console,
                   "Latched: motion refused until 'recover' succeeds\r\n");
    }
}

static void command_gait_diagnostics(AppConsole *console)
{
    const ActuatorDiagnostics *diagnostics =
        robot_gait_diagnostics(console->robot);
    char message[320];

    if (diagnostics == NULL || diagnostics->total_samples == 0U) {
        write_text(console, "Gait diagnostics: no samples yet\r\n");
        return;
    }

    (void)snprintf(
        message,
        sizeof(message),
        "Gait diagnostics: samples=%lu min_voltage=%umV lag=%u "
        "lag+droop=%u derate=%s fall=%s late_frames=%u "
        "limited_frames=%lu\r\n",
        (unsigned long)diagnostics->total_samples,
        (unsigned int)diagnostics->minimum_voltage_mv,
        (unsigned int)diagnostics->lag_samples,
        (unsigned int)diagnostics->lag_with_voltage_droop_samples,
        diagnostics->derate_recommended ? "recommended" : "no",
        console->robot->tilt_snapshot.valid ? "tilt" : "no",
        (unsigned int)console->robot->balance_late_frames,
        (unsigned long)console->robot->trot3_limited_frames);
    write_text(console, message);

    (void)snprintf(
        message,
        sizeof(message),
        "Gait timing: balance=%s nominal=%lums elapsed=%lums "
        "step_sync_blocking_wait=%ums max_barrier=%ums "
        "max_balance_gap=%ums\r\n",
        console->robot->gait_balance_was_enabled ? "on" : "off",
        (unsigned long)console->robot->gait_nominal_duration_ms,
        (unsigned long)console->robot->gait_elapsed_ms,
        (unsigned int)console->robot->trot_step_sync_wait_ms,
        (unsigned int)console->robot->trot_step_sync_max_wait_ms,
        (unsigned int)console->robot->balance_max_update_gap_ms);
    write_text(console, message);

    (void)snprintf(
        message,
        sizeof(message),
        "Step sync monitor: transitions=%u misses=%u "
        "peak_recent_error=%uticks\r\n",
        (unsigned int)console->robot->trot_step_sync_count,
        (unsigned int)console->robot->trot_step_sync_miss_count,
        (unsigned int)console->robot->trot_step_sync_peak_error_ticks);
    write_text(console, message);

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const ActuatorJointDiagnostics *joint = &diagnostics->joints[index];
        if (joint->sample_count == 0U) {
            continue;
        }
        const ActuatorTrackingSample *sample = &joint->latest;
        const uint32_t mean_error_ticks =
            (joint->absolute_error_sum_ticks + joint->sample_count / 2U) /
            joint->sample_count;
        (void)snprintf(
            message,
            sizeof(message),
            "  ID%u %s J%u phase=%u cmd=%u pos=%u err=%d speed_raw=%d "
            "current=%d load=%d voltage=%umV peak_err=%u(%lu.%ludeg)@%u "
            "mean_err=%lu(%lu.%ludeg) "
            "peak_current=%u peak_load=%u min_v=%umV lag=%u samples=%lu\r\n",
            (unsigned int)sample->servo_id,
            leg_name(sample->leg_index),
            (unsigned int)sample->joint_index,
            (unsigned int)sample->gait_phase,
            (unsigned int)sample->commanded_position,
            (unsigned int)sample->measured_position,
            (int)sample->position_error,
            (int)sample->measured_speed,
            (int)sample->current,
            (int)sample->load,
            (unsigned int)sample->voltage_mv,
            (unsigned int)joint->peak_position_error,
            (unsigned long)(ticks_to_tenths_degrees(
                joint->peak_position_error) / 10U),
            (unsigned long)(ticks_to_tenths_degrees(
                joint->peak_position_error) % 10U),
            (unsigned int)joint->peak_error_phase,
            (unsigned long)mean_error_ticks,
            (unsigned long)(ticks_to_tenths_degrees(mean_error_ticks) / 10U),
            (unsigned long)(ticks_to_tenths_degrees(mean_error_ticks) % 10U),
            (unsigned int)joint->peak_current_magnitude,
            (unsigned int)joint->peak_load_magnitude,
            (unsigned int)joint->minimum_voltage_mv,
            (unsigned int)joint->lag_samples,
            (unsigned long)joint->sample_count);
        write_text(console, message);
    }

    const RobotLimiterDiagnostics *limiter =
        &console->robot->limiter_diagnostics;
    if (limiter->frame_count > 0U) {
        write_text(console,
                   "Limiter distortion (policy/balance target -> command):\r\n");
        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            const uint32_t mean_millideg =
                limiter->joint_error_sum_millideg[index] /
                limiter->frame_count;
            (void)snprintf(
                message,
                sizeof(message),
                "  ID%u %s J%u peak=%u.%03udeg mean=%lu.%03ludeg\r\n",
                (unsigned int)g_robot_servo_ids[index],
                leg_name(g_robot_joints[index].leg_index),
                (unsigned int)g_robot_joints[index].joint_index,
                (unsigned int)(
                    limiter->joint_peak_error_millideg[index] / 1000U),
                (unsigned int)(
                    limiter->joint_peak_error_millideg[index] % 1000U),
                (unsigned long)(mean_millideg / 1000U),
                (unsigned long)(mean_millideg % 1000U));
            write_text(console, message);
        }
        for (uint8_t leg = 0U; leg < ROBOT_LEG_COUNT; ++leg) {
            const uint32_t mean_milli =
                limiter->foot_error_sum_milli[leg] / limiter->frame_count;
            (void)snprintf(
                message,
                sizeof(message),
                "  %s foot peak=%u.%03u mean=%lu.%03lu normalized-link\r\n",
                leg_name(leg),
                (unsigned int)(limiter->foot_peak_error_milli[leg] / 1000U),
                (unsigned int)(limiter->foot_peak_error_milli[leg] % 1000U),
                (unsigned long)(mean_milli / 1000U),
                (unsigned long)(mean_milli % 1000U));
            write_text(console, message);
        }
    }
}

static void command_balance_diagnostics(AppConsole *console)
{
    const RobotController *robot = console->robot;
    char message[320];
    char support[16];

    (void)snprintf(
        message,
        sizeof(message),
        "Balance trace: frames=%u applied=%s; angles/rates use 0.1deg, "
        "control/length/placement use milli-units\r\n",
        (unsigned int)robot->balance_trace.count,
        robot->gait_balance_was_enabled ? "on" : "off");
    write_text(console, message);

    if (robot->tilt_snapshot.valid) {
        const RobotTiltSnapshot *tilt = &robot->tilt_snapshot;
        support_mask_text(tilt->frame.support_mask, support);
        (void)snprintf(
            message,
            sizeof(message),
            "Tilt snapshot: phase=%u support=%s raw10=%d/%d "
            "filtered10=%d/%d "
            "rate10/s=%d/%d control_mrad=%d/%d j1_10=%d knee_10=%d "
            "len_milli=%d sat=0x%02X limited=0x%03X lag=%u "
            "worst_sampled=ID%u err=%d min_voltage=%umV\r\n",
            (unsigned int)tilt->frame.phase,
            support,
            (int)tilt->frame.raw_roll_tenths,
            (int)tilt->frame.raw_pitch_tenths,
            (int)tilt->frame.roll_tenths,
            (int)tilt->frame.pitch_tenths,
            (int)tilt->frame.roll_rate_tenths_s,
            (int)tilt->frame.pitch_rate_tenths_s,
            (int)tilt->frame.roll_control_millirad,
            (int)tilt->frame.pitch_control_millirad,
            (int)tilt->frame.j1_correction_tenths,
            (int)tilt->frame.knee_correction_tenths,
            (int)tilt->frame.leg_length_correction_milli,
            (unsigned int)tilt->frame.saturation_flags,
            (unsigned int)tilt->frame.limited_joint_mask,
            (unsigned int)tilt->frame.tracking_lag_samples,
            (unsigned int)tilt->worst_servo_id,
            (int)tilt->worst_position_error_ticks,
            (unsigned int)tilt->minimum_voltage_mv);
        write_text(console, message);
    } else {
        write_text(console, "Tilt snapshot: none\r\n");
    }

    const uint8_t count = robot->balance_trace.count;
    const uint8_t start = (uint8_t)(
        (robot->balance_trace.write_index + ROBOT_BALANCE_TRACE_CAPACITY -
         count) % ROBOT_BALANCE_TRACE_CAPACITY);
    for (uint8_t offset = 0U; offset < count; ++offset) {
        const uint8_t index = (uint8_t)(
            (start + offset) % ROBOT_BALANCE_TRACE_CAPACITY);
        const RobotBalanceTraceFrame *frame =
            &robot->balance_trace.frames[index];
        support_mask_text(frame->support_mask, support);
        (void)snprintf(
            message,
            sizeof(message),
            "  B phase=%u support=%s raw10=%d/%d filtered10=%d/%d "
            "rate10/s=%d/%d "
            "control_mrad=%d/%d j1_10=%d len_milli=%d knee_10=%d "
            "place_milli=%d sat=0x%02X limited=0x%03X lag=%u\r\n",
            (unsigned int)frame->phase,
            support,
            (int)frame->raw_roll_tenths,
            (int)frame->raw_pitch_tenths,
            (int)frame->roll_tenths,
            (int)frame->pitch_tenths,
            (int)frame->roll_rate_tenths_s,
            (int)frame->pitch_rate_tenths_s,
            (int)frame->roll_control_millirad,
            (int)frame->pitch_control_millirad,
            (int)frame->j1_correction_tenths,
            (int)frame->leg_length_correction_milli,
            (int)frame->knee_correction_tenths,
            (int)frame->foot_placement_correction_milli,
            (unsigned int)frame->saturation_flags,
            (unsigned int)frame->limited_joint_mask,
            (unsigned int)frame->tracking_lag_samples);
        write_text(console, message);
    }
}

static void command_recover(AppConsole *console)
{
    const RobotResult result = robot_recover(console->robot);

    if (result == ROBOT_SERVO_POWER_LOST) {
        /*
         * Say this plainly rather than as a bus timeout: with the rail down
         * there is nothing to relax or torque off, and the operator needs to
         * restore the supply before anything else is worth trying.
         */
        write_text(console,
                   "ERROR: servo power lost; no servo answered a ping. "
                   "Restore the 12V supply, then run recover again\r\n");
        return;
    }
    if (result == ROBOT_OK) {
        write_text(console,
                   "Recovered: holding at present positions, torque on\r\n");
        return;
    }
    print_robot_result(console, result);
}

static void command_imuprobe(AppConsole *console)
{
    Bno086Probe probe;
    char message[160];

    if (console->imu == NULL) {
        write_text(console, "ERROR: IMU probe unavailable\r\n");
        return;
    }

    bno086_probe(console->imu, &probe);

    (void)snprintf(message,
                   sizeof(message),
                   "IMUPROBE INT float=%lu%% pulldown=%lu%% in_reset=%lu%%\r\n",
                   (unsigned long)probe.int_float_percent,
                   (unsigned long)probe.int_pulldown_percent,
                   (unsigned long)probe.int_in_reset_percent);
    write_text(console, message);

    (void)snprintf(message,
                   sizeof(message),
                   "IMUPROBE blind read (%lu tries): %s %02X %02X %02X %02X\r\n",
                   (unsigned long)probe.blind_attempts,
                   probe.blind_data_seen ? "DATA" : "blank",
                   (unsigned int)probe.blind_header[0],
                   (unsigned int)probe.blind_header[1],
                   (unsigned int)probe.blind_header[2],
                   (unsigned int)probe.blind_header[3]);
    write_text(console, message);

    (void)snprintf(message,
                   sizeof(message),
                   "IMUPROBE in-reset read   : %02X %02X %02X %02X\r\n",
                   (unsigned int)probe.in_reset_header[0],
                   (unsigned int)probe.in_reset_header[1],
                   (unsigned int)probe.in_reset_header[2],
                   (unsigned int)probe.in_reset_header[3]);
    write_text(console, message);

    if (memcmp(probe.in_reset_header, probe.blind_header, 4) == 0) {
        write_text(console,
                   "  Holding RST low changes nothing, so the reset pulse "
                   "never reaches the part: check D3 against the sensor RST "
                   "pin\r\n");
    } else {
        write_text(console,
                   "  Holding RST low changes the answer, so D3 does reach "
                   "the part and reset works\r\n");
    }

    (void)snprintf(message,
                   sizeof(message),
                   "IMUPROBE deselected read: %02X %02X %02X %02X\r\n",
                   (unsigned int)probe.deselected_header[0],
                   (unsigned int)probe.deselected_header[1],
                   (unsigned int)probe.deselected_header[2],
                   (unsigned int)probe.deselected_header[3]);
    write_text(console, message);

    /*
     * A sensor that is present releases MISO when CS is high, so the two
     * reads differ.  Identical bytes mean nothing ever drives the line.
     */
    if (memcmp(probe.deselected_header, probe.blind_header, 4) == 0 &&
        !probe.blind_data_seen) {
        write_text(console,
                   "  CS makes no difference to MISO: the D12/PA6 wire is not "
                   "reaching a driven output. Suspect MISO wiring or sensor "
                   "power before anything protocol related\r\n");
    } else if (!probe.blind_data_seen) {
        write_text(console,
                   "  MISO follows CS, so the sensor is selected and driving: "
                   "it answers with a zero-length header, meaning it booted "
                   "but queued no advertisement. Suspect RST on D3\r\n");
    }

    for (uint32_t mode = 0U; mode < 4U; ++mode) {
        (void)snprintf(message,
                       sizeof(message),
                       "IMUPROBE mode %lu (CPOL=%lu CPHA=%lu): %02X %02X %02X %02X\r\n",
                       (unsigned long)mode,
                       (unsigned long)(mode >> 1),
                       (unsigned long)(mode & 1U),
                       (unsigned int)probe.mode_header[mode][0],
                       (unsigned int)probe.mode_header[mode][1],
                       (unsigned int)probe.mode_header[mode][2],
                       (unsigned int)probe.mode_header[mode][3]);
        write_text(console, message);
    }

    if (probe.blind_data_seen && !probe.int_asserted) {
        /*
         * The data path works and only the interrupt does not, so the sensor
         * itself, its supply and the SPI wiring are all fine.
         */
        write_text(console,
                   "IMUPROBE FAIL: SPI answers but H_INTN never asserts. The "
                   "sensor and SPI wiring are good; the INT wire to D7/PA8 is "
                   "the fault\r\n");
        return;
    }

    if (!probe.int_asserted) {
        write_text(console,
                   "IMUPROBE FAIL: H_INTN never went low after reset\r\n");
        write_text(console,
                   "  SPI returned no data either, so the sensor is not "
                   "speaking SPI at all\r\n");
        if (probe.int_pulldown_percent <= 10U) {
            write_text(console,
                       "  INT follows the pull-down even when running: "
                       "nothing drives it. Check the D7 wire, 3V3 and GND\r\n");
        } else if (probe.int_in_reset_percent <= 10U) {
            /*
             * The pin let go the moment RST went low, so the sensor is alive,
             * powered and holding it high on purpose.  It just never has SPI
             * traffic to announce, which is what a wrong interface looks like.
             */
            write_text(console,
                       "  INT released under reset, so the sensor drives it "
                       "and RST reaches the part. It is alive but not talking "
                       "SPI: check PS1/PS0 at reset (SPI needs both high)\r\n");
        } else {
            /*
             * Held high even in reset: whatever holds the pin is not the
             * sensor output, so this says nothing about the sensor itself.
             */
            write_text(console,
                       "  INT stays high even under reset: a board pull-up "
                       "holds it, or RST on D3 never reaches the part. Verify "
                       "the D3 wire before trusting any INT reading\r\n");
        }
        return;
    }

    (void)snprintf(message,
                   sizeof(message),
                   "IMUPROBE INT asserted after %lums; header %02X %02X %02X %02X\r\n",
                   (unsigned long)probe.assert_delay_ms,
                   (unsigned int)probe.header[0],
                   (unsigned int)probe.header[1],
                   (unsigned int)probe.header[2],
                   (unsigned int)probe.header[3]);
    write_text(console, message);

    const bool blank = (probe.header[0] == 0x00U && probe.header[1] == 0x00U &&
                        probe.header[2] == 0x00U && probe.header[3] == 0x00U) ||
                       (probe.header[0] == 0xFFU && probe.header[1] == 0xFFU &&
                        probe.header[2] == 0xFFU && probe.header[3] == 0xFFU);
    const uint16_t length =
        (uint16_t)(((uint16_t)probe.header[1] << 8) | probe.header[0]) & 0x7FFFU;

    if (!probe.header_read) {
        write_text(console, "IMUPROBE FAIL: SPI transfer error\r\n");
    } else if (blank) {
        write_text(console,
                   "IMUPROBE FAIL: MISO carried no data. Check D12/PA6 and "
                   "the SPI mode; the part uses CPOL=1 CPHA=1\r\n");
    } else if (length < 4U || length > 512U) {
        write_text(console,
                   "IMUPROBE FAIL: header length implausible. Suspect a bit "
                   "shift from the clock phase, or a CS/SCK wiring fault\r\n");
    } else {
        write_text(console,
                   "IMUPROBE PASS: SHTP link is up; rerun 'balance status' "
                   "after a reset\r\n");
    }
}

static void command_imu(AppConsole *console, char *mode)
{
    if (console->imu_log_enabled == NULL) {
        write_text(console, "ERROR: IMU logging is unavailable\r\n");
        return;
    }

    if (mode == NULL || strcmp(mode, "status") == 0) {
        write_text(console,
                   *console->imu_log_enabled ? "IMU log: on\r\n"
                                             : "IMU log: off\r\n");
    } else if (strcmp(mode, "on") == 0) {
        *console->imu_log_enabled = true;
        write_text(console, "IMU log: on\r\n");
    } else if (strcmp(mode, "off") == 0) {
        *console->imu_log_enabled = false;
        write_text(console, "IMU log: off\r\n");
    } else {
        write_text(console, "usage: imu on|off|status\r\n");
    }
}

static void command_balance(AppConsole *console, char *mode)
{
    RobotController *robot = console->robot;

    if (mode == NULL || strcmp(mode, "status") == 0) {
        char message[128];
        (void)snprintf(message,
                       sizeof(message),
                       "Control revision: %s\r\n",
                       ROBOT_CONTROL_REV);
        write_text(console, message);
        (void)snprintf(message,
                       sizeof(message),
                       "Balance: %s, mode: %s, target: %s, IMU: %s, policy: %s\r\n",
                       robot->balance_enabled ? "on" : "off",
                       robot_balance_mode_string(robot->balance_mode),
                       robot->balance_mode == ROBOT_BALANCE_FULL ?
                           "level" : "startup",
                       robot->attitude_reader != NULL ? "available"
                                                      : "unavailable",
                       robot->balance_required ? "required" : "open-loop allowed");
        write_text(console, message);
        if (robot->balance_reference_valid) {
            const int32_t roll = robot->balance_reference_roll_tenths;
            const int32_t pitch = robot->balance_reference_pitch_tenths;
            (void)snprintf(
                message,
                sizeof(message),
                "Last reference: Roll=%s%ld.%01ld Pitch=%s%ld.%01ld deg\r\n",
                roll < 0 ? "-" : "",
                labs(roll) / 10,
                labs(roll) % 10,
                pitch < 0 ? "-" : "",
                labs(pitch) / 10,
                labs(pitch) % 10);
            write_text(console, message);
            const int32_t last_roll = robot->balance_last_roll_error_tenths;
            const int32_t last_pitch = robot->balance_last_pitch_error_tenths;
            (void)snprintf(
                message,
                sizeof(message),
                "Last error: Roll=%s%ld.%01ld Pitch=%s%ld.%01ld deg\r\n",
                last_roll < 0 ? "-" : "",
                labs(last_roll) / 10,
                labs(last_roll) % 10,
                last_pitch < 0 ? "-" : "",
                labs(last_pitch) / 10,
                labs(last_pitch) % 10);
            write_text(console, message);
            (void)snprintf(
                message,
                sizeof(message),
                "Peak: Roll=%u.%u Pitch=%u.%u J1=%u.%u Knee=%u.%u late=%u\r\n",
                (unsigned int)(robot->balance_peak_roll_error_tenths / 10),
                (unsigned int)(robot->balance_peak_roll_error_tenths % 10),
                (unsigned int)(robot->balance_peak_pitch_error_tenths / 10),
                (unsigned int)(robot->balance_peak_pitch_error_tenths % 10),
                (unsigned int)(robot->balance_peak_j1_correction_tenths / 10),
                (unsigned int)(robot->balance_peak_j1_correction_tenths % 10),
                (unsigned int)(robot->balance_peak_knee_correction_tenths / 10),
                (unsigned int)(robot->balance_peak_knee_correction_tenths % 10),
                (unsigned int)robot->balance_late_frames);
            write_text(console, message);
        }
        (void)snprintf(
            message,
            sizeof(message),
            "Step sync monitor: transitions=%u misses=%u "
            "peak_recent_error=%uticks blocking_wait=%ums "
            "max_balance_gap=%ums\r\n",
            (unsigned int)robot->trot_step_sync_count,
            (unsigned int)robot->trot_step_sync_miss_count,
            (unsigned int)robot->trot_step_sync_peak_error_ticks,
            (unsigned int)robot->trot_step_sync_wait_ms,
            (unsigned int)robot->balance_max_update_gap_ms);
        write_text(console, message);
    } else if (strcmp(mode, "on") == 0) {
        if (!robot_set_balance_enabled(robot, true)) {
            write_text(console,
                       "ERROR: IMU unavailable; balance remains off\r\n");
            return;
        }
        write_text(console,
                   robot->balance_mode == ROBOT_BALANCE_FULL ?
                       "Balance: on, full (absolute level target)\r\n" :
                       "Balance: on, normal (startup attitude target)\r\n");
    } else if (strcmp(mode, "full") == 0) {
        if (!robot_set_balance_mode(robot, ROBOT_BALANCE_FULL)) {
            write_text(console,
                       "ERROR: IMU unavailable; balance mode unchanged\r\n");
            return;
        }
        write_text(console,
                   "Balance: on, full (absolute level, maximum bounded correction)\r\n");
    } else if (strcmp(mode, "normal") == 0) {
        if (!robot_set_balance_mode(robot, ROBOT_BALANCE_NORMAL)) {
            write_text(console,
                       "ERROR: IMU unavailable; balance mode unchanged\r\n");
            return;
        }
        write_text(console,
                   "Balance: on, normal (startup attitude target)\r\n");
    } else if (strcmp(mode, "off") == 0) {
        (void)robot_set_balance_enabled(robot, false);
        write_text(console,
                   "Balance: off (open-loop motion explicitly allowed)\r\n");
    } else {
        write_text(console,
                   "usage: balance full|normal|on|off|status\r\n");
    }
}

static void command_profile(AppConsole *console,
                            char *speed_text,
                            char *acceleration_text)
{
    if (speed_text == NULL && acceleration_text == NULL) {
        char message[64];
        (void)snprintf(message,
                       sizeof(message),
                       "Profile: speed=%u acceleration=%u\r\n",
                       (unsigned int)console->robot->profile_speed,
                       (unsigned int)console->robot->profile_acceleration);
        write_text(console, message);
        return;
    }

    uint32_t speed = 0U;
    uint32_t acceleration = 0U;
    if (!parse_u32(speed_text, 1U, 3400U, &speed) ||
        !parse_u32(acceleration_text, 0U, 254U, &acceleration) ||
        !robot_set_profile(console->robot,
                           (uint16_t)speed,
                           (uint8_t)acceleration)) {
        write_text(console, "usage: profile SPEED ACCEL; SPEED=1..3400 ACCEL=0..254\r\n");
        return;
    }

    char message[64];
    (void)snprintf(message,
                   sizeof(message),
                   "Profile set: speed=%lu acceleration=%lu\r\n",
                   (unsigned long)speed,
                   (unsigned long)acceleration);
    write_text(console, message);
}

static void command_trot(AppConsole *console,
                         char *cycles_text,
                         char *period_text)
{
    uint32_t cycles = 1U;
    uint32_t period_ms = GAIT_POLICY_SIM_TROT_PERIOD_MS;

    if ((cycles_text != NULL &&
         !parse_u32(cycles_text, 1U, 10U, &cycles)) ||
        (period_text != NULL &&
         !parse_u32(period_text, 600U, 5000U, &period_ms))) {
        write_text(console,
                   "usage: trot [CYCLES [PERIOD_MS]]; cycles=1..10 period=600..5000\r\n");
        return;
    }

    char message[96];
    (void)snprintf(message,
                   sizeof(message),
                   "Starting shared-C sim-trot: cycles=%lu period=%lums balance=%s/%s\r\n",
                   (unsigned long)cycles,
                   (unsigned long)period_ms,
                   console->robot->balance_enabled ? "on" : "off",
                   robot_balance_mode_string(console->robot->balance_mode));
    write_text(console, message);
    print_robot_result(console,
                       robot_trot(console->robot,
                                  (uint8_t)cycles,
                                  (uint16_t)period_ms));
}

static void command_trot_in_place(AppConsole *console,
                                  char *cycles_text,
                                  char *period_text)
{
    uint32_t cycles = 1U;
    uint32_t period_ms = GAIT_POLICY_SIM_TROT_PERIOD_MS;

    if ((cycles_text != NULL &&
         !parse_u32(cycles_text, 1U, 10U, &cycles)) ||
        (period_text != NULL &&
         !parse_u32(period_text, 600U, 5000U, &period_ms))) {
        write_text(console,
                   "usage: trotplace [CYCLES [PERIOD_MS]]; "
                   "cycles=1..10 period=600..5000\r\n");
        return;
    }

    char message[104];
    (void)snprintf(
        message,
        sizeof(message),
        "Starting in-place trot: cycles=%lu period=%lums balance=%s/%s\r\n",
        (unsigned long)cycles,
        (unsigned long)period_ms,
        console->robot->balance_enabled ? "on" : "off",
        robot_balance_mode_string(console->robot->balance_mode));
    write_text(console, message);
    print_robot_result(console,
                       robot_trot_in_place(console->robot,
                                           (uint8_t)cycles,
                                           (uint16_t)period_ms));
}

static void command_trot2(AppConsole *console,
                          char *cycles_text,
                          char *period_text)
{
    uint32_t cycles = 1U;
    uint32_t period_ms = GAIT_POLICY_TROT2_PERIOD_MS;

    if ((cycles_text != NULL &&
         !parse_u32(cycles_text, 1U, 10U, &cycles)) ||
        (period_text != NULL &&
         !parse_u32(period_text, 600U, 5000U, &period_ms))) {
        write_text(console,
                   "usage: trot2 [CYCLES [PERIOD_MS]]; "
                   "cycles=1..10 period=600..5000\r\n");
        return;
    }

    char message[112];
    (void)snprintf(
        message,
        sizeof(message),
        "Starting circular-foot trot2: cycles=%lu period=%lums "
        "fold=J2 78/J3 108 balance=%s/%s\r\n",
        (unsigned long)cycles,
        (unsigned long)period_ms,
        console->robot->balance_enabled ? "on" : "off",
        robot_balance_mode_string(console->robot->balance_mode));
    write_text(console, message);
    print_robot_result(console,
                       robot_trot2(console->robot,
                                   (uint8_t)cycles,
                                   (uint16_t)period_ms));
}

static void command_trot3(AppConsole *console,
                          char *cycles_text,
                          char *period_text)
{
    uint32_t cycles = 1U;
    uint32_t period_ms = GAIT_POLICY_TROT3_PERIOD_MS;

    if ((cycles_text != NULL &&
         !parse_u32(cycles_text, 1U, 10U, &cycles)) ||
        (period_text != NULL &&
         !parse_u32(period_text,
                    600U,
                    GAIT_POLICY_TROT3_MAX_PERIOD_MS,
                    &period_ms))) {
        write_text(console,
                   "usage: trot3 [CYCLES [PERIOD_MS]]; "
                   "cycles=1..10 period=600..1800 (default 1400)\r\n");
        return;
    }
    if (!actuator_profile_supports_trot3(
            console->robot->profile_speed,
            console->robot->profile_acceleration)) {
        write_text(console,
                   "ERROR: trot3 requires profile 3400 254; no motion started\r\n");
        return;
    }

    char message[192];
    (void)snprintf(
        message,
        sizeof(message),
        "Starting actuator-feasible trot3: cycles=%lu period=%lums "
        "limit=%udeg/s accel=%udeg/s2 shift=1.5deg sync=position "
        "balance=%s/%s rev=%s\r\n",
        (unsigned long)cycles,
        (unsigned long)period_ms,
        (unsigned int)MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S,
        (unsigned int)MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2,
        console->robot->balance_enabled ? "on" : "off",
        robot_balance_mode_string(console->robot->balance_mode),
        ROBOT_CONTROL_REV);
    write_text(console, message);
    const RobotResult result = robot_trot3(console->robot,
                                           (uint8_t)cycles,
                                           (uint16_t)period_ms);
    print_robot_result(console, result);
    command_gait_diagnostics(console);
    if (result == ROBOT_TILT_LIMIT) {
        command_balance_diagnostics(console);
    }
}

static void command_jump(AppConsole *console,
                         char *cycles_text,
                         char *period_text)
{
    uint32_t cycles = 0U;
    uint32_t period_ms = GAIT_POLICY_JUMP_PERIOD_MS;

    if ((cycles_text != NULL &&
         !parse_u32(cycles_text, 0U, 20U, &cycles)) ||
        (period_text != NULL &&
         !parse_u32(period_text, 800U, 5000U, &period_ms))) {
        write_text(console,
                   "usage: jump [CYCLES [PERIOD_MS]]; cycles=0..20 "
                   "(0=continuous) period=800..5000\r\n");
        return;
    }

    char message[112];
    if (cycles == 0U) {
        (void)snprintf(
            message,
            sizeof(message),
            "Starting in-place jump: continuous period=%lums; Ctrl+C to stop\r\n",
            (unsigned long)period_ms);
    } else {
        (void)snprintf(
            message,
            sizeof(message),
            "Starting in-place jump: cycles=%lu period=%lums; Ctrl+C to stop\r\n",
            (unsigned long)cycles,
            (unsigned long)period_ms);
    }
    write_text(console, message);
    print_robot_result(console,
                       robot_jump(console->robot,
                                  (uint8_t)cycles,
                                  (uint16_t)period_ms));
}

static void command_echo(AppConsole *console, char *mode)
{
    if (mode == NULL || strcmp(mode, "status") == 0) {
        write_text(console,
                   console->echo_enabled ? "Console echo: on\r\n"
                                         : "Console echo: off\r\n");
    } else if (strcmp(mode, "on") == 0) {
        console->echo_enabled = true;
        write_text(console, "Console echo: on\r\n");
    } else if (strcmp(mode, "off") == 0) {
        console->echo_enabled = false;
        write_text(console, "Console echo: off\r\n");
    } else {
        write_text(console, "usage: echo on|off|status\r\n");
    }
}

static void execute_line(AppConsole *console)
{
    char *command = strtok(console->line, " \t");
    if (command == NULL) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        app_console_print_help(console);
    } else if (strcmp(command, "ping") == 0) {
        command_ping(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "scan") == 0) {
        command_scan(console);
    } else if (strcmp(command, "uarttest") == 0) {
        command_uarttest(console);
    } else if (strcmp(command, "busprobe") == 0) {
        command_busprobe(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "linestate") == 0) {
        command_linestate(console);
    } else if (strcmp(command, "read") == 0) {
        command_read(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "move") == 0) {
        char *id = strtok(NULL, " \t");
        char *position = strtok(NULL, " \t");
        command_move(console, id, position);
    } else if (strcmp(command, "hold") == 0) {
        print_robot_result(console, robot_hold(console->robot));
    } else if (strcmp(command, "stand11") == 0) {
        write_text(console, "Straightening all four legs\r\n");
        print_robot_result(console, robot_stand_straight(console->robot));
    } else if (strcmp(command, "stand") == 0) {
        write_text(console, "Starting direct synchronized stand move\r\n");
        print_robot_result(console, robot_stand(console->robot));
    } else if (strcmp(command, "trot") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_trot(console, cycles, period);
    } else if (strcmp(command, "trotplace") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_trot_in_place(console, cycles, period);
    } else if (strcmp(command, "trot2") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_trot2(console, cycles, period);
    } else if (strcmp(command, "trot3") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_trot3(console, cycles, period);
    } else if (strcmp(command, "jump") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_jump(console, cycles, period);
    } else if (strcmp(command, "relax") == 0) {
        print_robot_result(console, robot_relax(console->robot));
    } else if (strcmp(command, "targets") == 0) {
        command_targets(console);
    } else if (strcmp(command, "profile") == 0) {
        char *speed = strtok(NULL, " \t");
        char *acceleration = strtok(NULL, " \t");
        command_profile(console, speed, acceleration);
    } else if (strcmp(command, "echo") == 0) {
        command_echo(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "safety") == 0) {
        command_safety(console);
    } else if (strcmp(command, "gaitdiag") == 0) {
        command_gait_diagnostics(console);
    } else if (strcmp(command, "baldiag") == 0) {
        command_balance_diagnostics(console);
    } else if (strcmp(command, "recover") == 0) {
        command_recover(console);
    } else if (strcmp(command, "i2cscan") == 0) {
        command_i2cscan(console);
    } else if (strcmp(command, "spitest") == 0) {
        command_spitest(console);
    } else if (strcmp(command, "imuprobe") == 0) {
        command_imuprobe(console);
    } else if (strcmp(command, "imu") == 0) {
        command_imu(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "balance") == 0) {
        command_balance(console, strtok(NULL, " \t"));
    } else {
        write_text(console, "unknown command; type help\r\n");
    }
}

void app_console_init(AppConsole *console,
                      UART_HandleTypeDef *uart,
                      RobotController *robot,
                      Bno055 *imu055,
                      Bno086 *imu,
                      bool *imu_log_enabled)
{
    if (console == NULL) {
        return;
    }

    console->uart = uart;
    console->robot = robot;
    console->imu055 = imu055;
    console->imu = imu;
    console->imu_log_enabled = imu_log_enabled;
    console->rx_byte = 0U;
    console->line[0] = '\0';
    console->line_length = 0U;
    console->line_ready = false;
    console->overflow = false;
    console->echo_enabled = false;

    if (uart != NULL) {
        (void)HAL_UART_Receive_IT(uart, &console->rx_byte, 1U);
    }
}

void app_console_on_rx_complete(AppConsole *console,
                                UART_HandleTypeDef *uart)
{
    if (console == NULL || uart == NULL || uart != console->uart) {
        return;
    }

    const uint8_t byte = console->rx_byte;
    if (byte == 0x03U) {
        robot_request_motion_abort(console->robot);
    } else if (!console->line_ready) {
        if (byte == '\r' || byte == '\n') {
            /*
             * Serial terminals may send CR, LF, or CRLF for Enter.  Ignore
             * an empty terminator so the LF half of CRLF cannot create a
             * second blank command after the CR command was consumed.
             */
            if (console->line_length != 0U || console->overflow) {
                console->line[console->line_length] = '\0';
                console->line_ready = true;
                echo_text(console, "\r\n");
            }
        } else if (byte == '\b' || byte == 0x7FU) {
            if (console->overflow) {
                console->overflow = false;
                echo_text(console, "\b \b");
            } else if (console->line_length != 0U) {
                --console->line_length;
                echo_text(console, "\b \b");
            }
        } else {
            if (console->line_length < APP_CONSOLE_LINE_CAPACITY - 1U) {
                console->line[console->line_length++] = (char)byte;
                echo_byte(console, byte);
            } else {
                console->overflow = true;
            }
        }
    }

    (void)HAL_UART_Receive_IT(console->uart, &console->rx_byte, 1U);
}

void app_console_poll(AppConsole *console)
{
    if (console == NULL || !console->line_ready) {
        return;
    }

    if (console->overflow) {
        write_text(console, "ERROR: command line too long\r\n");
    } else {
        execute_line(console);
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    console->line_length = 0U;
    console->line_ready = false;
    console->overflow = false;
    if (primask == 0U) {
        __enable_irq();
    }

    app_console_print_prompt(console);
}

void app_console_print_prompt(AppConsole *console)
{
    write_text(console, "# ");
}

void app_console_print_help(AppConsole *console)
{
    write_text(console,
               "\r\nSpot OMG STM32 servo console\r\n"
               "  ping ID          ping one servo\r\n"
               "  scan             ping configured IDs 1..12\r\n"
               "  uarttest         USART1 loopback test (PA9 connected to PA10)\r\n"
               "  busprobe ID      send ping and print raw USART1 response bytes\r\n"
               "  linestate        sample the idle level of PA9/PA10\r\n"
               "  read ID          read position/load/current/state\r\n"
               "  move ID RAW      safe single move, max delta 256 ticks\r\n"
               "  targets          print calibrated stand raw targets\r\n"
               "  profile [S A]   show/set speed 1..3400, acceleration 0..254\r\n"
               "  echo on|off     STM32 input echo control (default off)\r\n"
               "  hold             torque on at all current positions\r\n"
               "  stand            direct synchronized stand move\r\n"
               "  stand11          straighten every leg (J2=0, J3=0)\r\n"
               "  trot [C [MS]]    diagonal trot, cycles 1..10, period 600..5000ms\r\n"
               "  trotplace [C [MS]] in-place diagonal trot; Ctrl+C stop\r\n"
               "  trot2 [C [MS]]   circular-foot diagonal trot; Ctrl+C stop\r\n"
               "  trot3 [C [MS]]   overlap trot (default 1400ms, max 1800ms)\r\n"
               "  jump [C [MS]]    in-place repeat jump, C=0 continuous, Ctrl+C stop\r\n"
               "  relax            torque off all configured servos\r\n"
               "  safety           stall detector state and the latched fault\r\n"
               "  gaitdiag         last gait tracking/current/voltage report\r\n"
               "  baldiag          recent balance frames and tilt snapshot\r\n"
               "  recover          clear a safety fault and hold where the legs are\r\n"
               "  i2cscan          scan I2C1 for the BNO055\r\n"
               "  spitest          SPI1 loopback test (BNO086 removed, PA7 connected to PA6)\r\n"
               "  imuprobe         reset the BNO086 and report H_INTN and the SHTP header\r\n"
               "  imu on|off|status control 10 Hz IMU logging (default off)\r\n"
               "  balance full|normal|on|off|status IMU balance (default full/on)\r\n"
               "  help             show this help\r\n\r\n");
    write_text(console,
               console->echo_enabled ? "Console echo: on\r\n"
                                     : "Console echo: off\r\n");
}
