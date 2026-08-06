#include "app_console.h"

#include "feetech_protocol.h"
#include "robot_config.h"
#include "sts3215.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_SAFE_MOVE_DELTA 256U
#define UARTTEST_TIMEOUT_MS     10U
#define BUSPROBE_WINDOW_MS      50U

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
    if (result == ROBOT_IMU_ERROR) {
        write_text(console,
                   "ERROR: IMU balance error; stand target requested\r\n");
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
                       "Balance: %s, IMU: %s\r\n",
                       robot->balance_enabled ? "on" : "off",
                       robot->attitude_reader != NULL ? "available"
                                                      : "unavailable");
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
        }
    } else if (strcmp(mode, "on") == 0) {
        if (!robot_set_balance_enabled(robot, true)) {
            write_text(console,
                       "ERROR: IMU unavailable; balance remains off\r\n");
            return;
        }
        write_text(console, "Balance: on (Roll/Pitch, default off)\r\n");
    } else if (strcmp(mode, "off") == 0) {
        (void)robot_set_balance_enabled(robot, false);
        write_text(console, "Balance: off\r\n");
    } else {
        write_text(console, "usage: balance on|off|status\r\n");
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
    uint32_t period_ms = 1200U;

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
                   "Starting diagonal trot: cycles=%lu period=%lums balance=%s\r\n",
                   (unsigned long)cycles,
                   (unsigned long)period_ms,
                   console->robot->balance_enabled ? "on" : "off");
    write_text(console, message);
    print_robot_result(console,
                       robot_trot(console->robot,
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
    } else if (strcmp(command, "read") == 0) {
        command_read(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "move") == 0) {
        char *id = strtok(NULL, " \t");
        char *position = strtok(NULL, " \t");
        command_move(console, id, position);
    } else if (strcmp(command, "hold") == 0) {
        print_robot_result(console, robot_hold(console->robot));
    } else if (strcmp(command, "stand") == 0) {
        write_text(console, "Starting direct synchronized stand move\r\n");
        print_robot_result(console, robot_stand(console->robot));
    } else if (strcmp(command, "trot") == 0) {
        char *cycles = strtok(NULL, " \t");
        char *period = strtok(NULL, " \t");
        command_trot(console, cycles, period);
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
                      bool *imu_log_enabled)
{
    if (console == NULL) {
        return;
    }

    console->uart = uart;
    console->robot = robot;
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
    if (!console->line_ready) {
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
               "  read ID          read position/load/current/state\r\n"
               "  move ID RAW      safe single move, max delta 256 ticks\r\n"
               "  targets          print calibrated stand raw targets\r\n"
               "  profile [S A]   show/set speed 1..3400, acceleration 0..254\r\n"
               "  echo on|off     STM32 input echo control (default off)\r\n"
               "  hold             torque on at all current positions\r\n"
               "  stand            direct synchronized stand move\r\n"
               "  trot [C [MS]]    diagonal trot, cycles 1..10, period 600..5000ms\r\n"
               "  relax            torque off all configured servos\r\n"
               "  imu on|off|status control 10 Hz IMU logging (default off)\r\n"
               "  balance on|off|status Roll/Pitch trot feedback (default off)\r\n"
               "  help             show this help\r\n\r\n");
    write_text(console,
               console->echo_enabled ? "Console echo: on\r\n"
                                     : "Console echo: off\r\n");
}
