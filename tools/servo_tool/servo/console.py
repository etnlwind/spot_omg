"""Client for the STM32 text console on the ST-LINK virtual COM port.

This is the second, separate path to the servos.  ``ServoBus`` speaks the
Feetech protocol to a URT-2 plugged straight into the host at 1 Mbps.  This
module instead drives the 115200 bps text console that ``app_console.c``
exposes on USART2, and lets the STM32 keep ownership of the servo bus, the
50 Hz IMU balance loop and the step barrier.

The firmware protocol this mirrors:

* Every completed command is followed by the ``"# "`` prompt, written without
  a trailing newline.  That prompt is the only reliable end-of-response mark.
* A bare newline is discarded, so an empty line does *not* produce a prompt.
  :meth:`Stm32Console.sync` therefore sends a real command to resynchronize.
* Input echo defaults to off, which is what a programmatic client wants.
* Byte ``0x03`` (Ctrl+C) is handled in the USART2 RX interrupt and aborts a
  running motion, which then reports ``STOPPED:`` and returns to stand.
* With ``imu on`` the main loop writes ``Yaw=...`` telemetry at 10 Hz,
  asynchronously to any command response.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import threading
import time
from types import TracebackType
from typing import Callable, Iterable

try:
    import serial
except ImportError:  # pragma: no cover - depends on local environment
    serial = None

PROMPT = "# "
CONSOLE_BAUDRATE = 115_200
ABORT_BYTE = b"\x03"

#: Written by the 10 Hz IMU logger, not by any command.
TELEMETRY_PATTERN = re.compile(r"^Yaw=")

#: print_bus_result() writes "ID 3: timeout, servo_error=0x00" for a servo
#: that did not answer, and "ID 3: ok, ..." when it did.  Neither carries an
#: ERROR: prefix, so match the result word directly.  "ID 3 pos=..." and
#: "  ID 3 OK" have no colon and are not failures.
BUS_FAILURE_PATTERN = re.compile(r"^\s*ID \d+: (?!ok,)")

#: Commands whose runtime depends on cycle count and period.
_TIMED_COMMANDS = {
    # command: (default cycles, default period ms)
    "trot": (1, 800),
    "trotplace": (1, 800),
    "trot2": (1, 800),
    "jump": (0, 1200),
}

#: Fixed allowance for the stand move, bus retries and the final report.
MOTION_MARGIN_SECONDS = 20.0
DEFAULT_TIMEOUT_SECONDS = 15.0


class ConsoleError(RuntimeError):
    """The console did not answer, or answered with a failure."""


@dataclass(frozen=True)
class ConsoleResponse:
    """One command and everything the firmware printed before the prompt."""

    command: str
    lines: tuple[str, ...]
    telemetry: tuple[str, ...]
    status: str

    @property
    def ok(self) -> bool:
        return self.status in {"ok", "info"}

    @property
    def text(self) -> str:
        return "\n".join(self.lines)


def classify(lines: Iterable[str]) -> str:
    """Map firmware output to ok / error / stopped / info.

    ``info`` covers the commands that report values instead of a result, such
    as ``read``, ``targets``, ``profile`` and ``balance status``.
    """
    seen_ok = False
    stopped = False
    for line in lines:
        if (
            line.startswith("ERROR:")
            or line.startswith("usage:")
            or line.startswith("unknown command")
            or "FAIL" in line
            or BUS_FAILURE_PATTERN.match(line)
        ):
            return "error"
        if line.startswith("STOPPED:"):
            stopped = True
        elif line == "OK":
            seen_ok = True
    if stopped:
        return "stopped"
    return "ok" if seen_ok else "info"


def estimate_timeout(command: str) -> float | None:
    """Return a generous timeout for ``command``, or None if unbounded.

    ``jump 0`` repeats until Ctrl+C, so it has no finite completion time and
    the caller must supply its own timeout or interrupt it.
    """
    tokens = command.split()
    if not tokens:
        return DEFAULT_TIMEOUT_SECONDS
    name = tokens[0]
    if name == "scan":
        return 30.0
    if name not in _TIMED_COMMANDS:
        return DEFAULT_TIMEOUT_SECONDS

    default_cycles, default_period_ms = _TIMED_COMMANDS[name]
    try:
        cycles = int(tokens[1]) if len(tokens) > 1 else default_cycles
        period_ms = int(tokens[2]) if len(tokens) > 2 else default_period_ms
    except ValueError:
        # Let the firmware reject the arguments and print its usage line.
        return DEFAULT_TIMEOUT_SECONDS
    if cycles == 0:
        return None
    return cycles * period_ms / 1000.0 + MOTION_MARGIN_SECONDS


class Stm32Console:
    """Own the ST-LINK virtual COM port and exchange console lines."""

    def __init__(
        self,
        port: str,
        baudrate: int = CONSOLE_BAUDRATE,
        timeout: float = 0.2,
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

    def open(self) -> "Stm32Console":
        if not self.is_open:
            if serial is None:
                raise ImportError(
                    "pyserial is required; run: pip install -r requirements.txt"
                )
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=max(self.timeout, 1.0),
            )
        return self

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()

    def __enter__(self) -> "Stm32Console":
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
            raise ConsoleError("STM32 console is not open")

    def _write_line(self, command: str) -> None:
        payload = (command + "\n").encode("ascii", errors="strict")
        written = self._serial.write(payload)
        self._serial.flush()
        if written not in (None, len(payload)):
            raise ConsoleError("could not write the complete console command")

    def _read_until_prompt(
        self,
        deadline: float | None,
        on_line: Callable[[str], None] | None = None,
    ) -> list[str]:
        """Collect lines until the firmware writes its ``"# "`` prompt."""
        lines: list[str] = []
        buffer = ""
        while deadline is None or time.monotonic() < deadline:
            chunk = self._serial.read(max(1, self._serial.in_waiting or 1))
            if not chunk:
                continue
            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                raw, _, buffer = buffer.partition("\n")
                line = raw.rstrip("\r")
                # The prompt has no newline, so a line may still carry one
                # from a previous command whose output we joined.
                if line.startswith(PROMPT):
                    line = line[len(PROMPT):]
                if line:
                    lines.append(line)
                    if on_line is not None:
                        on_line(line)
            if buffer.endswith(PROMPT):
                return lines
        raise ConsoleError(
            "timed out waiting for the STM32 console prompt; "
            f"received {len(lines)} line(s)"
        )

    def abort(self) -> None:
        """Send Ctrl+C so a running motion returns to stand."""
        self._ensure_open()
        self._serial.write(ABORT_BYTE)
        self._serial.flush()

    def sync(self, timeout: float = 5.0) -> None:
        """Drop any boot banner and leave the console at a known prompt.

        Sends ``echo off`` because a bare newline is discarded by the
        firmware and would never produce a prompt to synchronize on.
        """
        self._ensure_open()
        with self._lock:
            self._serial.reset_input_buffer()
            self._write_line("echo off")
            self._read_until_prompt(time.monotonic() + timeout)

    def send(
        self,
        command: str,
        *,
        timeout: float | None = -1.0,
        on_line: Callable[[str], None] | None = None,
    ) -> ConsoleResponse:
        """Run one console command and return everything it printed.

        ``timeout`` defaults to :func:`estimate_timeout`; pass ``None`` to
        wait indefinitely.  A :class:`KeyboardInterrupt` while waiting sends
        Ctrl+C to the firmware instead of propagating, so the robot returns to
        stand rather than being left mid-stride.
        """
        command = command.strip()
        if not command:
            raise ValueError("console command must not be empty")
        if "\n" in command or "\r" in command:
            raise ValueError("console command must be a single line")
        self._ensure_open()

        if timeout == -1.0:
            timeout = estimate_timeout(command)
        deadline = None if timeout is None else time.monotonic() + timeout

        with self._lock:
            self._write_line(command)
            try:
                lines = self._read_until_prompt(deadline, on_line)
            except KeyboardInterrupt:
                self.abort()
                # The firmware still owes us "STOPPED:" and a prompt.
                lines = self._read_until_prompt(
                    time.monotonic() + MOTION_MARGIN_SECONDS, on_line
                )

        # With "echo on" the firmware mirrors the command back first.
        if lines and lines[0] == command:
            lines = lines[1:]
        telemetry = tuple(
            line for line in lines if TELEMETRY_PATTERN.match(line)
        )
        body = tuple(
            line for line in lines if not TELEMETRY_PATTERN.match(line)
        )
        return ConsoleResponse(
            command=command,
            lines=body,
            telemetry=telemetry,
            status=classify(body),
        )
