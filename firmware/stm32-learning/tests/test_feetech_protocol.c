#include "feetech_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static void test_ping_packet(void)
{
    const uint8_t expected[] = {0xFFU, 0xFFU, 0x01U, 0x02U, 0x01U, 0xFBU};
    uint8_t packet[16];

    const size_t length = feetech_encode_instruction(1U,
                                                     FEETECH_INST_PING,
                                                     NULL,
                                                     0U,
                                                     packet,
                                                     sizeof(packet));
    assert(length == sizeof(expected));
    assert(memcmp(packet, expected, sizeof(expected)) == 0);
}

static void test_read_packet(void)
{
    const uint8_t parameters[] = {56U, 2U};
    const uint8_t expected[] = {
        0xFFU, 0xFFU, 0x01U, 0x04U, 0x02U, 0x38U, 0x02U, 0xBEU
    };
    uint8_t packet[16];

    const size_t length = feetech_encode_instruction(1U,
                                                     FEETECH_INST_READ,
                                                     parameters,
                                                     sizeof(parameters),
                                                     packet,
                                                     sizeof(packet));
    assert(length == sizeof(expected));
    assert(memcmp(packet, expected, sizeof(expected)) == 0);
}

static void test_status_and_integer_helpers(void)
{
    uint8_t status[] = {0xFFU, 0xFFU, 0x01U, 0x04U,
                        0x00U, 0x02U, 0x08U, 0xF0U};
    uint8_t encoded[2];

    assert(feetech_packet_checksum_valid(status, sizeof(status)));
    status[7] ^= 0x01U;
    assert(!feetech_packet_checksum_valid(status, sizeof(status)));

    feetech_encode_u16(2050U, encoded);
    assert(encoded[0] == 0x02U && encoded[1] == 0x08U);
    assert(feetech_decode_u16(encoded) == 2050U);

    encoded[0] = 100U;
    encoded[1] = 0x80U;
    assert(feetech_decode_sign_magnitude(encoded) == -100);
}

int main(void)
{
    test_ping_packet();
    test_read_packet();
    test_status_and_integer_helpers();
    return 0;
}
