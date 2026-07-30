"""SCServo/STS serial packet encoding and decoding."""

from __future__ import annotations

from dataclasses import dataclass

HEADER = b"\xff\xff"

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_SYNC_WRITE = 0x83


class ProtocolError(RuntimeError):
    """Raised when a malformed packet is received."""


class ServoError(RuntimeError):
    """Raised when a servo reports an error status."""

    ERROR_NAMES = {
        0x01: "input voltage",
        0x02: "angle limit",
        0x04: "overheating",
        0x08: "range",
        0x10: "checksum",
        0x20: "overload",
    }

    def __init__(self, servo_id: int, error: int) -> None:
        labels = [
            name for bit, name in self.ERROR_NAMES.items() if error & bit
        ]
        detail = ", ".join(labels) if labels else f"unknown error 0x{error:02x}"
        super().__init__(f"servo {servo_id}: {detail}")
        self.servo_id = servo_id
        self.error = error


@dataclass(frozen=True)
class StatusPacket:
    servo_id: int
    error: int
    parameters: bytes


def checksum(data: bytes) -> int:
    """Return the protocol checksum for bytes after the 0xff headers."""
    return (~sum(data)) & 0xFF


def encode_instruction(
    servo_id: int, instruction: int, parameters: bytes = b""
) -> bytes:
    if not 0 <= servo_id <= 0xFE:
        raise ValueError("servo id must be between 0 and 254")
    if len(parameters) > 251:
        raise ValueError("too many packet parameters")
    body = bytes((servo_id, len(parameters) + 2, instruction)) + parameters
    return HEADER + body + bytes((checksum(body),))


def decode_status(packet: bytes) -> StatusPacket:
    if len(packet) < 6 or packet[:2] != HEADER:
        raise ProtocolError("invalid status packet header")
    expected_length = packet[3] + 4
    if len(packet) != expected_length:
        raise ProtocolError(
            f"invalid status packet length: expected {expected_length}, got {len(packet)}"
        )
    if checksum(packet[2:-1]) != packet[-1]:
        raise ProtocolError("status packet checksum mismatch")
    return StatusPacket(packet[2], packet[4], packet[5:-1])


def encode_u16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("16-bit value must be between 0 and 65535")
    return value.to_bytes(2, "little")


def decode_u16(value: bytes) -> int:
    if len(value) != 2:
        raise ValueError("a 16-bit value requires exactly two bytes")
    return int.from_bytes(value, "little")


def decode_sign_magnitude(value: bytes) -> int:
    """Decode the 15-bit sign/magnitude values used for speed and load."""
    raw = decode_u16(value)
    magnitude = raw & 0x7FFF
    return -magnitude if raw & 0x8000 else magnitude
