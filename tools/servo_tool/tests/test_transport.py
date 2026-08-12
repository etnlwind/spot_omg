from __future__ import annotations

import socket
import threading
import unittest
from unittest.mock import MagicMock, patch

from servo.console import Stm32Console
from servo.transport import SerialTransport, TcpTransport


class MockBridge:
    def __init__(self) -> None:
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.host, self.port = self.listener.getsockname()
        self.received: list[bytes] = []
        self.thread = threading.Thread(target=self._serve, daemon=True)

    def _serve(self) -> None:
        connection, _ = self.listener.accept()
        with connection:
            pending = b""
            while True:
                data = connection.recv(4096)
                if not data:
                    break
                pending += data
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    command = line.rstrip(b"\r")
                    self.received.append(command)
                    if command == b"echo off":
                        connection.sendall(b"Console echo: off\r\n# ")
                    elif command == b"stand":
                        connection.sendall(b"OK\r\n# ")

    def __enter__(self) -> "MockBridge":
        self.thread.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.listener.close()
        self.thread.join(timeout=1)


class TcpTransportTest(unittest.TestCase):
    def test_connect_read_write_and_stm32_response(self) -> None:
        with MockBridge() as bridge:
            transport = TcpTransport(bridge.host, bridge.port, timeout=0.05)
            with Stm32Console("tcp-test", transport=transport) as console:
                console.sync()
                response = console.send("stand")
            self.assertEqual(bridge.received, [b"echo off", b"stand"])
            self.assertEqual(response.lines, ("OK",))
            self.assertTrue(response.ok)

    def test_read_returns_empty_bytes_on_timeout(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        host, port = listener.getsockname()
        thread = threading.Thread(target=lambda: listener.accept(), daemon=True)
        thread.start()
        transport = TcpTransport(host, port, timeout=0.02).open()
        try:
            self.assertEqual(transport.read(1), b"")
        finally:
            transport.close()
            listener.close()

    def test_connect_failure_has_bridge_address(self) -> None:
        with patch("socket.create_connection", side_effect=OSError("refused")):
            with self.assertRaisesRegex(
                ConnectionError,
                r"Unable to connect to STM32 bridge at example\.invalid:3333",
            ):
                TcpTransport("example.invalid").open()


class SerialTransportRegressionTest(unittest.TestCase):
    def test_serial_options_and_stream_methods_are_preserved(self) -> None:
        port = MagicMock()
        port.is_open = True
        port.in_waiting = 4
        port.write.return_value = 3
        port.read.return_value = b"abc"
        port.readline.return_value = b"line\n"
        transport = SerialTransport("COM7", 115200, serial_port=port)
        self.assertTrue(transport.is_open)
        self.assertEqual(transport.in_waiting, 4)
        self.assertEqual(transport.write(b"abc"), 3)
        self.assertEqual(transport.read(3), b"abc")
        self.assertEqual(transport.readline(), b"line\n")
        transport.flush()
        transport.reset_input_buffer()
        transport.close()
        port.flush.assert_called_once()
        port.reset_input_buffer.assert_called_once()
        port.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
