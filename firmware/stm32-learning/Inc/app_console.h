#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bno055.h"
#include "bno086.h"
#include "main.h"
#include "robot.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_CONSOLE_LINE_CAPACITY 96U

typedef struct
{
    UART_HandleTypeDef *uart;
    RobotController *robot;
    Bno055 *imu055;
    Bno086 *imu;
    bool *imu_log_enabled;
    uint8_t rx_byte;
    char line[APP_CONSOLE_LINE_CAPACITY];
    volatile size_t line_length;
    volatile bool line_ready;
    volatile bool overflow;
    volatile bool echo_enabled;
} AppConsole;

void app_console_init(AppConsole *console,
                      UART_HandleTypeDef *uart,
                      RobotController *robot,
                      Bno055 *imu055,
                      Bno086 *imu,
                      bool *imu_log_enabled);

void app_console_on_rx_complete(AppConsole *console,
                                UART_HandleTypeDef *uart);
void app_console_on_uart_error(AppConsole *console,
                               UART_HandleTypeDef *uart);

void app_console_poll(AppConsole *console);
void app_console_print_help(AppConsole *console);
void app_console_print_prompt(AppConsole *console);

#ifdef __cplusplus
}
#endif

#endif
