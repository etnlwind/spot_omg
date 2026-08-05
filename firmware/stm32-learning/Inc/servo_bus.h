#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    SERVO_BUS_OK = 0,
    SERVO_BUS_INVALID_ARGUMENT,
    SERVO_BUS_HAL_ERROR,
    SERVO_BUS_TIMEOUT,
    SERVO_BUS_PROTOCOL_ERROR,
    SERVO_BUS_SERVO_ERROR
} ServoBusResult;

typedef struct
{
    UART_HandleTypeDef *uart;
    uint32_t timeout_ms;
    uint8_t last_servo_error;
} ServoBus;

void servo_bus_init(ServoBus *bus,
                    UART_HandleTypeDef *uart,
                    uint32_t timeout_ms);

ServoBusResult servo_bus_request(ServoBus *bus,
                                 uint8_t servo_id,
                                 uint8_t instruction,
                                 const uint8_t *parameters,
                                 size_t parameter_count,
                                 uint8_t *response_parameters,
                                 size_t response_capacity,
                                 size_t *response_count,
                                 bool expect_response);

ServoBusResult servo_bus_ping(ServoBus *bus, uint8_t servo_id);

ServoBusResult servo_bus_read(ServoBus *bus,
                              uint8_t servo_id,
                              uint8_t address,
                              uint8_t *data,
                              size_t data_size);

ServoBusResult servo_bus_write(ServoBus *bus,
                               uint8_t servo_id,
                               uint8_t address,
                               const uint8_t *data,
                               size_t data_size);

ServoBusResult servo_bus_sync_write(ServoBus *bus,
                                    uint8_t address,
                                    uint8_t item_size,
                                    const uint8_t *servo_ids,
                                    const uint8_t *items,
                                    size_t servo_count);

const char *servo_bus_result_string(ServoBusResult result);

#ifdef __cplusplus
}
#endif

#endif
