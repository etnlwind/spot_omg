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
    SERVO_BUS_SERVO_ERROR,
    SERVO_BUS_BUSY
} ServoBusResult;

/* Gait-only asynchronous reads. One transaction owns the half-duplex bus.
 * Times are DWT cycle differences, so the 32-bit counter wrap is harmless. */
#define SERVO_FEEDBACK_WINDOW_US 4000U
#define SERVO_FEEDBACK_QUIET_US 1000U
#define SERVO_FEEDBACK_QUARANTINE_US 25000U
#define SERVO_FEEDBACK_TRACE_CAPACITY 32U
typedef struct {
    uint32_t start_ms, duration_us, first_rx_us, last_rx_us, uart_errors;
    uint8_t id, attempt, result, received, stage, sent;
} ServoReadTrace;
typedef struct {
    volatile uint8_t state, tx_index, rx_count;
    volatile uint32_t started, last_activity, first_rx, last_rx, uart_errors;
    uint32_t queued, quarantine, cycles_per_us, start_ms, end_ms;
    bool enabled, quarantined;
    volatile bool writing;
    uint8_t id, size, attempt, packet[8], rx[32], payload[15];
    ServoBusResult result;
    uint32_t ok, timeouts, errors, late_bytes, refused_writes, max_us, irq_peak_cycles;
    ServoReadTrace trace[SERVO_FEEDBACK_TRACE_CAPACITY];
    uint8_t trace_write, trace_count, trace_tail;
    bool trace_frozen, trace_triggered;
} ServoFeedback;

typedef struct
{
    UART_HandleTypeDef *uart;
    int32_t front_position_bias[2]; /* Live STS3250 turn origin; never EEPROM. */
    int32_t front_stow_origin[2];
    bool front_origin_valid[2];
    uint32_t timeout_ms;
    uint8_t last_servo_error;
    uint32_t read_retry_attempts;
    uint32_t read_retry_recoveries;
    uint32_t read_retry_failures;
    ServoFeedback feedback;
} ServoBus;

void servo_bus_feedback_begin(ServoBus *bus);
void servo_bus_feedback_end(ServoBus *bus);
ServoBusResult servo_bus_feedback_start(ServoBus *bus, uint8_t id,
                                       uint8_t address, uint8_t size, uint8_t attempt);
/* BUSY means no completed sample. A completed result is consumed exactly once. */
ServoBusResult servo_bus_feedback_take(ServoBus *bus, uint8_t *data,
                                      uint32_t *begin_ms, uint32_t *end_ms);
void servo_bus_feedback_irq(void);
void servo_bus_feedback_tick(void);
bool servo_bus_feedback_can_write(ServoBus *bus);
void servo_bus_feedback_written(ServoBus *bus);
void servo_bus_feedback_legacy_guard(ServoBus *bus);

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

void servo_bus_clear_retry_diagnostics(ServoBus *bus);

const char *servo_bus_result_string(ServoBusResult result);

#ifdef __cplusplus
}
#endif

#endif
