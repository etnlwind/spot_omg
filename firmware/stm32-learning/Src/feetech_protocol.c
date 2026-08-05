#include "feetech_protocol.h"

uint8_t feetech_checksum(const uint8_t *data, size_t length)
{
    uint32_t sum = 0U;

    if (data == NULL && length != 0U) {
        return 0U;
    }

    for (size_t index = 0U; index < length; ++index) {
        sum += data[index];
    }

    return (uint8_t)(~sum);
}

size_t feetech_encode_instruction(uint8_t servo_id,
                                  uint8_t instruction,
                                  const uint8_t *parameters,
                                  size_t parameter_count,
                                  uint8_t *packet,
                                  size_t packet_capacity)
{
    const size_t packet_length = parameter_count + 6U;

    if (packet == NULL || servo_id > FEETECH_BROADCAST_ID ||
        parameter_count > 251U || packet_capacity < packet_length ||
        (parameter_count != 0U && parameters == NULL)) {
        return 0U;
    }

    packet[0] = FEETECH_HEADER_BYTE;
    packet[1] = FEETECH_HEADER_BYTE;
    packet[2] = servo_id;
    packet[3] = (uint8_t)(parameter_count + 2U);
    packet[4] = instruction;

    for (size_t index = 0U; index < parameter_count; ++index) {
        packet[5U + index] = parameters[index];
    }

    packet[packet_length - 1U] =
        feetech_checksum(&packet[2], packet_length - 3U);
    return packet_length;
}

bool feetech_packet_checksum_valid(const uint8_t *packet,
                                   size_t packet_length)
{
    if (packet == NULL || packet_length < 6U ||
        packet[0] != FEETECH_HEADER_BYTE ||
        packet[1] != FEETECH_HEADER_BYTE ||
        packet_length != (size_t)packet[3] + 4U) {
        return false;
    }

    return feetech_checksum(&packet[2], packet_length - 3U) ==
           packet[packet_length - 1U];
}

void feetech_encode_u16(uint16_t value, uint8_t output[2])
{
    if (output == NULL) {
        return;
    }

    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)(value >> 8U);
}

uint16_t feetech_decode_u16(const uint8_t input[2])
{
    if (input == NULL) {
        return 0U;
    }

    return (uint16_t)input[0] | ((uint16_t)input[1] << 8U);
}

int16_t feetech_decode_sign_magnitude(const uint8_t input[2])
{
    const uint16_t raw = feetech_decode_u16(input);
    const int16_t magnitude = (int16_t)(raw & 0x7FFFU);

    return (raw & 0x8000U) != 0U ? (int16_t)-magnitude : magnitude;
}
