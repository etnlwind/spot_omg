"""Byte-stream transports used by the STM32 text console."""

from __future__ import annotations

import socket
from types import TracebackType
from typing import Protocol, runtime_checkable

try:
    import serial
except ImportError:  # pragma: no cover - depends on local environment
    serial = None


@runtime_checkable
class Transport(Protocol):
    """Minimal serial-like stream required by :class:`Stm32Console`."""

    timeout: float

    @property
    def is_open(self) -> bool: ...
    @property
    def in_waiting(self) -> int: ...
    def open(self) -> "Transport": ...
    def write(self, data: bytes) -> int: ...
    def read(self, size: int = 1) -> bytes: ...
    def readline(self, size: int = -1) -> bytes: ...
    def flush(self) -> None: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


class SerialTransport:
    """Adapter around pyserial preserving the existing console behavior."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float = 0.2,
        *,
        serial_port=None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = serial_port

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def in_waiting(self) -> int:
        return self._serial.in_waiting if self._serial is not None else 0

    def open(self) -> "SerialTransport":
        if not self.is_open:
            if serial is None:
                raise ImportError("pyserial is required; run: pip install pyserial")
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=max(self.timeout, 1.0),
                exclusive=True,
            )
        return self

    def write(self, data: bytes) -> int:
        return self._serial.write(data)

    def read(self, size: int = 1) -> bytes:
        return self._serial.read(size)

    def readline(self, size: int = -1) -> bytes:
        return self._serial.readline(size)

    def flush(self) -> None:
        self._serial.flush()

    def reset_input_buffer(self) -> None:
        self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()

    def __enter__(self) -> "SerialTransport":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


class TcpTransport:
    """Serial-like TCP stream for an ESP32-C3 STM32 UART bridge."""

    def __init__(self, host: str, port: int = 3333, timeout: float = 0.2) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: socket.socket | None = None
        self._read_buffer = bytearray()

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    @property
    def in_waiting(self) -> int:
        return len(self._read_buffer)

    def open(self) -> "TcpTransport":
        if not self.is_open:
            try:
                self._socket = socket.create_connection(
                    (self.host, self.port), timeout=self.timeout
                )
                self._socket.settimeout(self.timeout)
            except OSError as exc:
                self._socket = None
                raise ConnectionError(
                    f"Unable to connect to STM32 bridge at {self.host}:{self.port}"
                ) from exc
        return self

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise ConnectionError("TCP transport is not open")
        return self._socket

    def write(self, data: bytes) -> int:
        sock = self._require_socket()
        sock.sendall(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if size <= 0:
            return b""
        if self._read_buffer:
            data = bytes(self._read_buffer[:size])
            del self._read_buffer[:size]
            return data
        try:
            return self._require_socket().recv(size)
        except socket.timeout:
            return b""

    def readline(self, size: int = -1) -> bytes:
        while size < 0 or len(self._read_buffer) < size:
            newline = self._read_buffer.find(b"\n")
            if newline >= 0:
                end = newline + 1
                data = bytes(self._read_buffer[:end])
                del self._read_buffer[:end]
                return data
            try:
                chunk = self._require_socket().recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            self._read_buffer.extend(chunk)
        end = len(self._read_buffer) if size < 0 else min(size, len(self._read_buffer))
        data = bytes(self._read_buffer[:end])
        del self._read_buffer[:end]
        return data

    def flush(self) -> None:
        # sendall() is synchronous; TCP has no userspace output buffer to flush.
        return None

    def reset_input_buffer(self) -> None:
        self._read_buffer.clear()
        sock = self._require_socket()
        previous_timeout = sock.gettimeout()
        try:
            sock.setblocking(False)
            while sock.recv(4096):
                pass
        except (BlockingIOError, socket.timeout):
            pass
        finally:
            sock.settimeout(previous_timeout)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None

    def __enter__(self) -> "TcpTransport":
        return self.open()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
