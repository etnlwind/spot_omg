#include "app_console.h"

#include "robot_config.h"
#include "sts3215.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_SAFE_MOVE_DELTA 256U

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
    } else if (strcmp(command, "read") == 0) {
        command_read(console, strtok(NULL, " \t"));
    } else if (strcmp(command, "move") == 0) {
        char *id = strtok(NULL, " \t");
        char *position = strtok(NULL, " \t");
        command_move(console, id, position);
    } else if (strcmp(command, "hold") == 0) {
        print_robot_result(console, robot_hold(console->robot));
    } else if (strcmp(command, "stand") == 0) {
        write_text(console, "Starting calibrated 2-second stand ramp\r\n");
        print_robot_result(console, robot_stand(console->robot));
    } else if (strcmp(command, "relax") == 0) {
        print_robot_result(console, robot_relax(console->robot));
    } else if (strcmp(command, "targets") == 0) {
        command_targets(console);
    } else if (strcmp(command, "imu") == 0) {
        command_imu(console, strtok(NULL, " \t"));
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
            }
        } else {
            if (console->line_length < APP_CONSOLE_LINE_CAPACITY - 1U) {
                console->line[console->line_length++] = (char)byte;
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
}

void app_console_print_help(AppConsole *console)
{
    write_text(console,
               "\r\nSpot OMG STM32 servo console\r\n"
               "  ping ID          ping one servo\r\n"
               "  scan             ping configured IDs 1..12\r\n"
               "  read ID          read position/load/current/state\r\n"
               "  move ID RAW      safe single move, max delta 256 ticks\r\n"
               "  targets          print calibrated stand raw targets\r\n"
               "  hold             torque on at all current positions\r\n"
               "  stand            2-second synchronized stand ramp\r\n"
               "  relax            torque off all configured servos\r\n"
               "  imu on|off|status control 10 Hz IMU logging (default off)\r\n"
               "  help             show this help\r\n\r\n");
}
