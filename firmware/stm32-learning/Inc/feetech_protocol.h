#ifndef FEETECH_PROTOCOL_H
#define FEETECH_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FEETECH_HEADER_BYTE          0xFFU
#define FEETECH_BROADCAST_ID         0xFEU
#define FEETECH_PACKET_CAPACITY      128U

#define FEETECH_INST_PING            0x01U
#define FEETECH_INST_READ            0x02U
#define FEETECH_INST_WRITE           0x03U
#define FEETECH_INST_REG_WRITE       0x04U
#define FEETECH_INST_ACTION          0x05U
#define FEETECH_INST_SYNC_WRITE      0x83U

uint8_t feetech_checksum(const uint8_t *data, size_t length);

size_t feetech_encode_instruction(uint8_t servo_id,
                                  uint8_t instruction,
                                  const uint8_t *parameters,
                                  size_t parameter_count,
                                  uint8_t *packet,
                                  size_t packet_capacity);

bool feetech_packet_checksum_valid(const uint8_t *packet,
                                   size_t packet_length);

void feetech_encode_u16(uint16_t value, uint8_t output[2]);
uint16_t feetech_decode_u16(const uint8_t input[2]);
int16_t feetech_decode_sign_magnitude(const uint8_t input[2]);

#ifdef __cplusplus
}
#endif

#endif
