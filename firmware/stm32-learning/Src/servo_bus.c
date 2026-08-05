#include "servo_bus.h"

#include "feetech_protocol.h"

#include <stdbool.h>
#include <string.h>

static bool timeout_elapsed(uint32_t started_at, uint32_t timeout_ms)
{
    return (uint32_t)(HAL_GetTick() - started_at) >= timeout_ms;
}

static ServoBusResult read_byte_until(ServoBus *bus,
                                      uint32_t started_at,
                                      uint8_t *value)
{
    while (!timeout_elapsed(started_at, bus->timeout_ms)) {
        HAL_StatusTypeDef status = HAL_UART_Receive(bus->uart, value, 1U, 1U);
        if (status == HAL_OK) {
            return SERVO_BUS_OK;
        }
        if (status != HAL_TIMEOUT) {
            return SERVO_BUS_HAL_ERROR;
        }
    }

    return SERVO_BUS_TIMEOUT;
}

static ServoBusResult receive_packet(ServoBus *bus,
                                     uint32_t started_at,
                                     uint8_t *packet,
                                     size_t packet_capacity,
                                     size_t *packet_length)
{
    uint8_t header_count = 0U;
    uint8_t byte = 0U;

    if (packet == NULL || packet_length == NULL || packet_capacity < 6U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    while (header_count < 2U) {
        ServoBusResult result = read_byte_until(bus, started_at, &byte);
        if (result != SERVO_BUS_OK) {
            return result;
        }

        if (byte == FEETECH_HEADER_BYTE) {
            packet[header_count++] = byte;
        } else {
            header_count = 0U;
        }
    }

    for (size_t index = 2U; index < 4U; ++index) {
        ServoBusResult result = read_byte_until(bus, started_at, &packet[index]);
        if (result != SERVO_BUS_OK) {
            return result;
        }
    }

    if (packet[3] < 2U) {
        return SERVO_BUS_PROTOCOL_ERROR;
    }

    const size_t total = (size_t)packet[3] + 4U;
    if (total > packet_capacity) {
        return SERVO_BUS_PROTOCOL_ERROR;
    }

    for (size_t index = 4U; index < total; ++index) {
        ServoBusResult result = read_byte_until(bus, started_at, &packet[index]);
        if (result != SERVO_BUS_OK) {
            return result;
        }
    }

    *packet_length = total;
    return SERVO_BUS_OK;
}

static void flush_uart_rx(UART_HandleTypeDef *uart)
{
    __HAL_UART_CLEAR_OREFLAG(uart);
    while (__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE) != RESET) {
        volatile uint32_t discarded = uart->Instance->DR;
        (void)discarded;
    }
}

void servo_bus_init(ServoBus *bus,
                    UART_HandleTypeDef *uart,
                    uint32_t timeout_ms)
{
    if (bus == NULL) {
        return;
    }

    bus->uart = uart;
    bus->timeout_ms = timeout_ms;
    bus->last_servo_error = 0U;
}

ServoBusResult servo_bus_request(ServoBus *bus,
                                 uint8_t servo_id,
                                 uint8_t instruction,
                                 const uint8_t *parameters,
                                 size_t parameter_count,
                                 uint8_t *response_parameters,
                                 size_t response_capacity,
                                 size_t *response_count,
                                 bool expect_response)
{
    uint8_t tx_packet[FEETECH_PACKET_CAPACITY];
    uint8_t rx_packet[FEETECH_PACKET_CAPACITY];
    bool echo_discarded = false;

    if (bus == NULL || bus->uart == NULL || bus->timeout_ms == 0U ||
        servo_id > FEETECH_BROADCAST_ID ||
        (response_capacity != 0U && response_parameters == NULL)) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    const size_t tx_length = feetech_encode_instruction(
        servo_id,
        instruction,
        parameters,
        parameter_count,
        tx_packet,
        sizeof(tx_packet));
    if (tx_length == 0U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    if (response_count != NULL) {
        *response_count = 0U;
    }
    bus->last_servo_error = 0U;
    flush_uart_rx(bus->uart);

    /*
     * The URT-2 UART header may reflect the TTL bus onto RX while TX is
     * active. Disable the STM32 receiver during transmission so the echo
     * cannot overrun the one-byte UART receive register. Re-enable it as
     * soon as HAL has observed the final stop bit.
     */
    CLEAR_BIT(bus->uart->Instance->CR1, USART_CR1_RE);
    HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(
        bus->uart,
        tx_packet,
        (uint16_t)tx_length,
        bus->timeout_ms);
    SET_BIT(bus->uart->Instance->CR1, USART_CR1_RE);
    if (transmit_status != HAL_OK) {
        return SERVO_BUS_HAL_ERROR;
    }

    if (!expect_response || servo_id == FEETECH_BROADCAST_ID) {
        return SERVO_BUS_OK;
    }

    const uint32_t started_at = HAL_GetTick();
    while (!timeout_elapsed(started_at, bus->timeout_ms)) {
        size_t rx_length = 0U;
        ServoBusResult result = receive_packet(bus,
                                               started_at,
                                               rx_packet,
                                               sizeof(rx_packet),
                                               &rx_length);
        if (result != SERVO_BUS_OK) {
            return result;
        }

        if (!echo_discarded && rx_length == tx_length &&
            memcmp(rx_packet, tx_packet, tx_length) == 0) {
            echo_discarded = true;
            continue;
        }

        if (!feetech_packet_checksum_valid(rx_packet, rx_length) ||
            rx_packet[2] != servo_id) {
            return SERVO_BUS_PROTOCOL_ERROR;
        }

        bus->last_servo_error = rx_packet[4];
        if (bus->last_servo_error != 0U) {
            return SERVO_BUS_SERVO_ERROR;
        }

        const size_t parameter_bytes = rx_length - 6U;
        if (parameter_bytes > response_capacity) {
            return SERVO_BUS_PROTOCOL_ERROR;
        }
        if (parameter_bytes != 0U) {
            memcpy(response_parameters, &rx_packet[5], parameter_bytes);
        }
        if (response_count != NULL) {
            *response_count = parameter_bytes;
        }
        return SERVO_BUS_OK;
    }

    return SERVO_BUS_TIMEOUT;
}

ServoBusResult servo_bus_ping(ServoBus *bus, uint8_t servo_id)
{
    return servo_bus_request(bus,
                             servo_id,
                             FEETECH_INST_PING,
                             NULL,
                             0U,
                             NULL,
                             0U,
                             NULL,
                             true);
}

ServoBusResult servo_bus_read(ServoBus *bus,
                              uint8_t servo_id,
                              uint8_t address,
                              uint8_t *data,
                              size_t data_size)
{
    uint8_t parameters[2];
    size_t response_count = 0U;

    if (data == NULL || data_size == 0U || data_size > 255U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    parameters[0] = address;
    parameters[1] = (uint8_t)data_size;
    ServoBusResult result = servo_bus_request(bus,
                                              servo_id,
                                              FEETECH_INST_READ,
                                              parameters,
                                              sizeof(parameters),
                                              data,
                                              data_size,
                                              &response_count,
                                              true);
    if (result != SERVO_BUS_OK) {
        return result;
    }

    return response_count == data_size ? SERVO_BUS_OK
                                       : SERVO_BUS_PROTOCOL_ERROR;
}

ServoBusResult servo_bus_write(ServoBus *bus,
                               uint8_t servo_id,
                               uint8_t address,
                               const uint8_t *data,
                               size_t data_size)
{
    uint8_t parameters[FEETECH_PACKET_CAPACITY - 6U];

    if (data_size > sizeof(parameters) - 1U ||
        (data_size != 0U && data == NULL)) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    parameters[0] = address;
    if (data_size != 0U) {
        memcpy(&parameters[1], data, data_size);
    }

    return servo_bus_request(bus,
                             servo_id,
                             FEETECH_INST_WRITE,
                             parameters,
                             data_size + 1U,
                             NULL,
                             0U,
                             NULL,
                             true);
}

ServoBusResult servo_bus_sync_write(ServoBus *bus,
                                    uint8_t address,
                                    uint8_t item_size,
                                    const uint8_t *servo_ids,
                                    const uint8_t *items,
                                    size_t servo_count)
{
    uint8_t parameters[FEETECH_PACKET_CAPACITY - 6U];
    const size_t parameter_count = 2U + servo_count * (1U + item_size);

    if (item_size == 0U || servo_count == 0U || servo_ids == NULL ||
        items == NULL || parameter_count > sizeof(parameters)) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    parameters[0] = address;
    parameters[1] = item_size;
    size_t output = 2U;
    for (size_t servo = 0U; servo < servo_count; ++servo) {
        if (servo_ids[servo] >= FEETECH_BROADCAST_ID) {
            return SERVO_BUS_INVALID_ARGUMENT;
        }
        parameters[output++] = servo_ids[servo];
        memcpy(&parameters[output],
               &items[servo * item_size],
               item_size);
        output += item_size;
    }

    return servo_bus_request(bus,
                             FEETECH_BROADCAST_ID,
                             FEETECH_INST_SYNC_WRITE,
                             parameters,
                             parameter_count,
                             NULL,
                             0U,
                             NULL,
                             false);
}

const char *servo_bus_result_string(ServoBusResult result)
{
    switch (result) {
    case SERVO_BUS_OK:
        return "ok";
    case SERVO_BUS_INVALID_ARGUMENT:
        return "invalid argument";
    case SERVO_BUS_HAL_ERROR:
        return "UART error";
    case SERVO_BUS_TIMEOUT:
        return "timeout";
    case SERVO_BUS_PROTOCOL_ERROR:
        return "protocol error";
    case SERVO_BUS_SERVO_ERROR:
        return "servo error";
    default:
        return "unknown";
    }
}
