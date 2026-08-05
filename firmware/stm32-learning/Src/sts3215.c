#include "sts3215.h"

#include "feetech_protocol.h"

ServoBusResult sts3215_ping(ServoBus *bus, uint8_t servo_id)
{
    return servo_bus_ping(bus, servo_id);
}

ServoBusResult sts3215_set_torque(ServoBus *bus,
                                  uint8_t servo_id,
                                  bool enabled)
{
    const uint8_t value = enabled ? 1U : 0U;
    return servo_bus_write(bus,
                           servo_id,
                           STS3215_ADDR_TORQUE_ENABLE,
                           &value,
                           1U);
}

ServoBusResult sts3215_read_position(ServoBus *bus,
                                     uint8_t servo_id,
                                     uint16_t *position)
{
    uint8_t raw[2];

    if (position == NULL) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    ServoBusResult result = servo_bus_read(bus,
                                           servo_id,
                                           STS3215_ADDR_PRESENT_POSITION,
                                           raw,
                                           sizeof(raw));
    if (result == SERVO_BUS_OK) {
        *position = feetech_decode_u16(raw);
    }
    return result;
}

ServoBusResult sts3215_read_state(ServoBus *bus,
                                  uint8_t servo_id,
                                  Sts3215State *state)
{
    uint8_t raw[15];

    if (state == NULL) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    ServoBusResult result = servo_bus_read(bus,
                                           servo_id,
                                           STS3215_ADDR_PRESENT_POSITION,
                                           raw,
                                           sizeof(raw));
    if (result != SERVO_BUS_OK) {
        return result;
    }

    const uint16_t load_raw = feetech_decode_u16(&raw[4]);
    const int16_t load_magnitude = (int16_t)(load_raw & 0x03FFU);
    state->position = feetech_decode_u16(&raw[0]);
    state->speed = feetech_decode_sign_magnitude(&raw[2]);
    state->load = (load_raw & 0x0400U) != 0U
                      ? (int16_t)-load_magnitude
                      : load_magnitude;
    state->voltage_mv = (uint16_t)raw[6] * 100U;
    state->temperature_c = raw[7];
    state->hardware_error = raw[9];
    state->moving = raw[10] != 0U;
    state->current = feetech_decode_sign_magnitude(&raw[13]);
    return SERVO_BUS_OK;
}

ServoBusResult sts3215_write_position(ServoBus *bus,
                                      uint8_t servo_id,
                                      uint16_t position,
                                      uint16_t speed,
                                      uint8_t acceleration)
{
    uint8_t data[7];

    if (position > STS3215_MAX_POSITION || speed > 3400U ||
        acceleration > 254U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    data[0] = acceleration;
    feetech_encode_u16(position, &data[1]);
    feetech_encode_u16(0U, &data[3]);
    feetech_encode_u16(speed, &data[5]);
    return servo_bus_write(bus,
                           servo_id,
                           STS3215_ADDR_ACCELERATION,
                           data,
                           sizeof(data));
}

ServoBusResult sts3215_sync_move(ServoBus *bus,
                                 const uint8_t *servo_ids,
                                 const uint16_t *positions,
                                 size_t servo_count,
                                 uint16_t speed,
                                 uint8_t acceleration)
{
    uint8_t items[12U * 7U];

    if (servo_ids == NULL || positions == NULL || servo_count == 0U ||
        servo_count > 12U || speed > 3400U || acceleration > 254U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    for (size_t index = 0U; index < servo_count; ++index) {
        if (positions[index] > STS3215_MAX_POSITION) {
            return SERVO_BUS_INVALID_ARGUMENT;
        }
        uint8_t *item = &items[index * 7U];
        item[0] = acceleration;
        feetech_encode_u16(positions[index], &item[1]);
        feetech_encode_u16(0U, &item[3]);
        feetech_encode_u16(speed, &item[5]);
    }

    return servo_bus_sync_write(bus,
                                STS3215_ADDR_ACCELERATION,
                                7U,
                                servo_ids,
                                items,
                                servo_count);
}

ServoBusResult sts3215_sync_positions(ServoBus *bus,
                                      const uint8_t *servo_ids,
                                      const uint16_t *positions,
                                      size_t servo_count)
{
    uint8_t items[12U * 2U];

    if (servo_ids == NULL || positions == NULL || servo_count == 0U ||
        servo_count > 12U) {
        return SERVO_BUS_INVALID_ARGUMENT;
    }

    for (size_t index = 0U; index < servo_count; ++index) {
        if (positions[index] > STS3215_MAX_POSITION) {
            return SERVO_BUS_INVALID_ARGUMENT;
        }
        feetech_encode_u16(positions[index], &items[index * 2U]);
    }

    return servo_bus_sync_write(bus,
                                STS3215_ADDR_GOAL_POSITION,
                                2U,
                                servo_ids,
                                items,
                                servo_count);
}
