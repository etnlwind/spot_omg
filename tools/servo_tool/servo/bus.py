"""Serial bus transport for Feetech SCServo-compatible servos."""

from __future__ import annotations

import threading
import time
from types import TracebackType
from typing import Iterable

try:
    import serial
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise ImportError(
        "pyserial is required; run: pip install -r requirements.txt"
    ) from exc

from .protocol import (
    HEADER,
    INST_PING,
    INST_READ,
    INST_SYNC_WRITE,
    INST_WRITE,
    ProtocolError,
    ServoError,
    StatusPacket,
    decode_status,
    encode_instruction,
)

BROADCAST_ID = 0xFE


class ServoBus:
    """Own a half-duplex serial connection and exchange servo packets."""

    def __init__(
        self,
        port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        *,
        serial_port=None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = serial_port
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> "ServoBus":
        if not self.is_open:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
        return self

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()

    def __enter__(self) -> "ServoBus":
        return self.open()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _ensure_open(self) -> None:
        if not self.is_open:
            raise RuntimeError("servo bus is not open")

    def _read_status(self, expected_id: int | None = None) -> StatusPacket:
        self._ensure_open()
        deadline = time.monotonic() + self.timeout
        header = bytearray()
        while time.monotonic() < deadline:
            byte = self._serial.read(1)
            if not byte:
                continue
            header = (header + byte)[-2:]
            if bytes(header) == HEADER:
                break
        else:
            raise TimeoutError("servo did not respond")

        prefix = self._serial.read(2)
        if len(prefix) != 2:
            raise TimeoutError("incomplete servo status packet")
        servo_id, length = prefix
        if length < 2:
            raise ProtocolError(f"invalid status length {length}")
        remainder = self._serial.read(length)
        if len(remainder) != length:
            raise TimeoutError("incomplete servo status packet")
        status = decode_status(HEADER + prefix + remainder)
        if expected_id is not None and status.servo_id != expected_id:
            raise ProtocolError(
                f"expected servo {expected_id}, received servo {status.servo_id}"
            )
        if status.error:
            raise ServoError(status.servo_id, status.error)
        return status

    def request(
        self,
        servo_id: int,
        instruction: int,
        parameters: bytes = b"",
        *,
        expect_response: bool = True,
    ) -> StatusPacket | None:
        with self._lock:
            self._ensure_open()
            self._serial.reset_input_buffer()
            packet = encode_instruction(servo_id, instruction, parameters)
            written = self._serial.write(packet)
            self._serial.flush()
            if written != len(packet):
                raise TimeoutError("could not write complete servo packet")
            if not expect_response or servo_id == BROADCAST_ID:
                return None
            return self._read_status(servo_id)

    def ping(self, servo_id: int) -> bool:
        try:
            self.request(servo_id, INST_PING)
            return True
        except (TimeoutError, ProtocolError, ServoError):
            return False

    def scan(self, ids: Iterable[int] = range(1, 254)) -> list[int]:
        return [servo_id for servo_id in ids if self.ping(servo_id)]

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        if not 0 <= address <= 0xFF or not 1 <= size <= 0xFF:
            raise ValueError("address and size must fit in one byte")
        status = self.request(servo_id, INST_READ, bytes((address, size)))
        assert status is not None
        if len(status.parameters) != size:
            raise ProtocolError(
                f"expected {size} data bytes, got {len(status.parameters)}"
            )
        return status.parameters

    def write(
        self,
        servo_id: int,
        address: int,
        data: bytes,
        *,
        expect_response: bool = True,
    ) -> None:
        if not 0 <= address <= 0xFF:
            raise ValueError("address must fit in one byte")
        self.request(
            servo_id,
            INST_WRITE,
            bytes((address,)) + data,
            expect_response=expect_response,
        )

    def sync_write(
        self, address: int, item_size: int, values: dict[int, bytes]
    ) -> None:
        if not values:
            return
        parameters = bytearray((address, item_size))
        for servo_id, data in values.items():
            if len(data) != item_size:
                raise ValueError(f"servo {servo_id}: expected {item_size} bytes")
            parameters.append(servo_id)
            parameters.extend(data)
        self.request(
            BROADCAST_ID,
            INST_SYNC_WRITE,
            bytes(parameters),
            expect_response=False,
        )
