"""Command-line interface for URT-2 and the twelve-servo Spot Micro."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import os
import statistics
import sys
import time
from pathlib import Path

from .bus import ServoBus
from .console import CONSOLE_BAUDRATE, ConsoleError, Stm32Console
from .contact import LEGS, LoadContactEstimator
from .load_profile import DynamicLoadBaseline
from .spot import GaitParameters, SpotConfig, SpotRobot
from .sts3215 import STS3215

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "joints.json"
DEFAULT_PROFILE_DIR = Path(__file__).resolve().parents[1] / "logs"
PRESETS = {
    "test": GaitParameters(
        period=4.0,
        hip_amplitude=8.8,
        lift_amplitude=12.3,
        crouch_amplitude=0,
        speed=60,
        acceleration=30,
        duty_factor=0.65,
    ),
    "natural": GaitParameters(duty_factor=0.65),
    "power": GaitParameters(
        period=2.4,
        hip_amplitude=10.0,
        lift_amplitude=16.0,
        crouch_amplitude=0.0,
        speed=800,
        acceleration=80,
        duty_factor=0.58,
        control_rate=50.0,
        stance_j1_angle=4.0,
        stance_j2_angle=30.0,
        stance_j3_angle=60.0,
        weight_shift_amplitude=1.5,
        preload_amplitude=1.5,
    ),
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="spotctl",
        description="Inspect and safely exercise the Spot OMG servos.",
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("SPOT_SERVO_PORT"),
        help="URT-2 port; defaults to SPOT_SERVO_PORT or auto-detection",
    )
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--via",
        choices=("auto", "urt2", "stm32"),
        default=os.environ.get("SPOT_TRANSPORT", "auto"),
        help=(
            "which link to use; auto picks it from the attached USB device "
            "(default: auto, or SPOT_TRANSPORT)"
        ),
    )
    parser.add_argument(
        "--stm32-port",
        default=os.environ.get("SPOT_STM32_PORT"),
        help=(
            "ST-LINK virtual COM port; defaults to SPOT_STM32_PORT or "
            "auto-detection by USB vendor ID"
        ),
    )
    parser.add_argument(
        "--console-baudrate", type=int, default=CONSOLE_BAUDRATE
    )
    parser.add_argument(
        "--console-timeout",
        type=float,
        help=(
            "seconds to wait for the STM32 prompt; defaults to a per-command "
            "estimate from cycles and period"
        ),
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="append the STM32 console transcript to this file",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("ports", help="list serial ports")
    scan = commands.add_parser("scan", help="scan for responding servo IDs")
    scan.add_argument("--min-id", type=int, default=1)
    scan.add_argument("--max-id", type=int, default=253)
    diagnose = commands.add_parser(
        "diagnose", help="read configuration and health registers from one servo"
    )
    diagnose.add_argument("--id", type=int, default=1)
    change_id = commands.add_parser(
        "change-id", help="change one servo ID and verify EEPROM persistence"
    )
    change_id.add_argument("old_id", type=int)
    change_id.add_argument("new_id", type=int)
    change_id.add_argument(
        "--scan-max",
        type=int,
        default=253,
        help="highest ID checked before writing (default: 253)",
    )
    swap_ids = commands.add_parser(
        "swap-ids", help="swap two EEPROM servo IDs while all servos stay connected"
    )
    swap_ids.add_argument("first_id", type=int)
    swap_ids.add_argument("second_id", type=int)
    swap_ids.add_argument("--temp-id", type=int, default=253)
    commands.add_parser(
        "configure-mapping", help="interactively assign three servo IDs per leg"
    )
    commands.add_parser(
        "configure-directions", help="interactively set canonical joint directions"
    )
    calibrate = commands.add_parser(
        "calibrate", help="interactively adjust and save neutral offsets"
    )
    calibrate.add_argument("--speed", type=int, default=200)
    calibrate.add_argument("--accel", type=int, default=20)
    calibrate.add_argument("--max-offset", type=int, default=500)
    capture_stand = commands.add_parser(
        "capture-stand",
        aliases=["save-stand"],
        help="save all current positions as calibrated stand centers",
    )
    capture_stand.add_argument(
        "--leg",
        choices=("FL", "FR", "RL", "RR"),
        help="update only the selected leg",
    )
    commands.add_parser("status", help="read all configured servo health values")
    contacts = commands.add_parser(
        "contacts", help="monitor estimated foot contact from J2/J3 motor load"
    )
    contacts.add_argument("--baseline", type=float, default=2.0)
    contacts.add_argument("--duration", type=float, default=10.0)
    contacts.add_argument("--rate", type=float, default=10.0)
    contacts.add_argument("--threshold", type=int, default=24)
    contacts.add_argument("--release", type=int, default=8)
    contacts.add_argument("--on-samples", type=int, default=2)
    contacts.add_argument("--off-samples", type=int, default=3)
    contacts.add_argument(
        "--verbose", action="store_true", help="print every load sample"
    )
    analyze_loads = commands.add_parser(
        "analyze-loads",
        help="build a phase-aligned no-contact baseline from a gait CSV",
    )
    analyze_loads.add_argument("profile", type=Path)
    analyze_loads.add_argument("--output", type=Path)

    relax = commands.add_parser("relax", help="disable torque on all or one leg")
    relax.add_argument(
        "--leg",
        choices=("FL", "FR", "RL", "RR"),
        help="disable torque only for the selected leg",
    )
    hold = commands.add_parser(
        "hold", help="safely enable torque at the current physical position"
    )
    hold.add_argument(
        "--leg",
        choices=("FL", "FR", "RL", "RR"),
        help="enable torque only for the selected leg",
    )
    hold.add_argument("--speed", type=int, default=60)
    hold.add_argument("--accel", type=int, default=30)

    pose = commands.add_parser(
        "pose", aliases=["apply-pose"], help="move to a saved pose"
    )
    pose.add_argument("name", help="pose name stored in the configuration")
    pose.add_argument("--speed", type=int, default=1000)
    pose.add_argument("--accel", type=int, default=80)
    pose.add_argument("--timeout", "--wait", dest="timeout", type=float, default=10.0)
    pose.add_argument("--tolerance", type=int, default=30)

    for command, help_text in (
        ("stand", "move to the calibrated neutral stance"),
        ("stand45", "generate a 45-degree stance from current calibration"),
        ("landing", "move to the calibrated landing stance"),
    ):
        stance = commands.add_parser(command, help=help_text)
        stance.add_argument("--speed", type=int, default=1000)
        stance.add_argument("--accel", type=int, default=80)
        stance.add_argument("--timeout", type=float, default=10.0)
        stance.add_argument("--tolerance", type=int, default=30)
        if command == "stand":
            stance.add_argument(
                "--leg",
                choices=("FL", "FR", "RL", "RR"),
                help="move only the selected leg to calibrated neutral",
            )

    save_pose = commands.add_parser(
        "save-pose", help="save all current servo positions as a named pose"
    )
    save_pose.add_argument("name")

    raw_center = commands.add_parser(
        "raw-center", help="move every servo to the uncalibrated raw value 2048"
    )
    raw_center.add_argument("--speed", type=int, default=400)
    raw_center.add_argument("--accel", type=int, default=50)
    raw_center.add_argument("--timeout", type=float, default=15.0)
    raw_center.add_argument("--tolerance", type=int, default=30)

    walk = commands.add_parser("walk", help="run a diagonal trot or crawl gait")
    walk.add_argument(
        "--gait",
        choices=("trot", "crawl"),
        default="trot",
        help="leg phase pattern (default: trot)",
    )
    walk.add_argument("--preset", choices=PRESETS, default="test")
    walk.add_argument("--cycles", type=int, default=1)
    walk.add_argument("--period", type=float)
    walk.add_argument("--hip", type=float, help="J2 amplitude in degrees")
    walk.add_argument("--lift", type=float, help="J3 lift amplitude in degrees")
    walk.add_argument("--crouch", type=float, help="J3 crouch angle in degrees")
    walk.add_argument(
        "--stance-j1", type=float, help="walking stance J1 abduction angle"
    )
    walk.add_argument(
        "--stance-j2", type=float, help="walking stance J2 angle in degrees"
    )
    walk.add_argument(
        "--stance-j3", type=float, help="walking stance J3 angle in degrees"
    )
    walk.add_argument(
        "--duty", type=float, help="fraction of each cycle spent on the ground"
    )
    walk.add_argument(
        "--weight-shift", type=float, help="diagonal J1 load-transfer amplitude"
    )
    walk.add_argument(
        "--preload", type=float, help="stance-pair preload in degree-equivalent units"
    )
    walk.add_argument(
        "--speed", type=int, help="maximum STS speed-register value"
    )
    walk.add_argument(
        "--accel", type=int, help="STS acceleration-register value"
    )
    walk.add_argument(
        "--rate", type=float, help="position update rate in Hz (default: 30)"
    )
    walk.add_argument(
        "--profile-loads",
        action="store_true",
        help="record phase-aligned J2/J3 states without changing the gait",
    )
    walk.add_argument(
        "--load-rate",
        type=float,
        default=5.0,
        help="samples per second for each J2/J3 servo (default: 5)",
    )
    walk.add_argument(
        "--profile-output",
        type=Path,
        help="CSV path; implies --profile-loads",
    )
    walk.add_argument(
        "--load-baseline",
        type=Path,
        help="compare live loads with a phase baseline; observation only",
    )

    for name, help_text in (
        ("trot", "shared-C sim-trot on the STM32"),
        ("trotplace", "in-place diagonal trot on the STM32"),
        ("trot2", "circular-foot diagonal trot on the STM32"),
        ("jump", "repeating in-place jump on the STM32; 0 cycles repeats"),
    ):
        motion = commands.add_parser(name, help=help_text)
        motion.add_argument("cycles", type=int, nargs="?")
        motion.add_argument("period_ms", type=int, nargs="?")

    commands.add_parser(
        "targets", help="print the STM32 calibrated stand raw targets"
    )
    profile = commands.add_parser(
        "profile", help="show or set the STM32 servo speed and acceleration"
    )
    profile.add_argument("speed", type=int, nargs="?")
    profile.add_argument("accel", type=int, nargs="?")
    imu = commands.add_parser("imu", help="control STM32 10 Hz IMU logging")
    imu.add_argument("mode", nargs="?", choices=("on", "off", "status"))
    balance = commands.add_parser(
        "balance", help="control the STM32 IMU balance policy"
    )
    balance.add_argument(
        "mode", nargs="?", choices=("full", "normal", "on", "off", "status")
    )

    console = commands.add_parser(
        "console",
        help="send raw command lines to the STM32 text console",
        description=(
            "Escape hatch for firmware console commands that have no spotctl "
            "subcommand, for an interactive prompt, or for running a "
            "procedure file.  Ordinary commands such as 'spotctl stand' and "
            "'spotctl trot2 1 1600' already route to the STM32 on their own "
            "when the ST-LINK is the attached device."
        ),
    )
    console_actions = console.add_subparsers(
        dest="console_command", required=True
    )
    console_send = console_actions.add_parser(
        "send", help="run one console command and print its response"
    )
    console_send.add_argument(
        "words",
        nargs="+",
        help="console command and arguments, for example: trot2 1 1600",
    )
    console_actions.add_parser(
        "shell",
        help="interactive prompt; Ctrl+C aborts a motion, Ctrl+D exits",
    )
    console_script = console_actions.add_parser(
        "script", help="run console commands from a file, stopping on error"
    )
    console_script.add_argument("path", type=Path)
    return parser.parse_args(argv)


#: STMicroelectronics; the ST-LINK virtual COM port on every Nucleo board.
ST_LINK_VENDOR_ID = 0x0483

#: Used only when the backend hides the USB numbers.
ST_LINK_HINTS = ("stlink", "st-link", "st link", "stm32")

_PORT_NAME_MARKERS = ("usbmodem", "usbserial", "ttyusb", "ttyacm")


def _usb_number(value) -> int | None:
    """Normalize a pyserial vid/pid, which is an int on some platforms and a
    hex string such as ``'0x0483'`` on others (macOS)."""
    if value is None:
        return None
    if isinstance(value, int):
        return value
    try:
        return int(str(value), 16)
    except ValueError:
        return None


@dataclass(frozen=True)
class PortInfo:
    """One serial port and the USB descriptors behind it."""

    device: str
    description: str
    vid: int | None = None
    pid: int | None = None
    manufacturer: str | None = None
    product: str | None = None
    serial_number: str | None = None

    @property
    def usb_id(self) -> str:
        if self.vid is None or self.pid is None:
            return "-"
        return f"{self.vid:04x}:{self.pid:04x}"

    @property
    def is_st_link(self) -> bool:
        """True for the STM32 debug console, false for the URT-2.

        The vendor ID is the reliable test.  Device names are not: on macOS
        the URT-2 also enumerates as ``/dev/cu.usbmodem...``.
        """
        if self.vid is not None:
            return self.vid == ST_LINK_VENDOR_ID
        text = " ".join(
            part.lower()
            for part in (self.description, self.product, self.manufacturer)
            if part
        )
        return any(hint in text for hint in ST_LINK_HINTS)

    @property
    def looks_like_usb_serial(self) -> bool:
        if self.vid is not None:
            return True
        name = self.device.lower()
        return (
            any(marker in name for marker in _PORT_NAME_MARKERS)
            or self.device.upper().startswith("COM")
        )

    def summary(self) -> str:
        role = " <- STM32 console" if self.is_st_link else ""
        return f"{self.device:32} {self.usb_id:10} {self.description}{role}"


def serial_ports() -> list[PortInfo]:
    """Enumerate serial ports together with their USB vendor/product IDs."""
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise ImportError(
            "pyserial is required; run: pip install -r requirements.txt"
        ) from exc
    return sorted(
        (
            PortInfo(
                device=item.device,
                description=item.description or "unknown device",
                vid=_usb_number(item.vid),
                pid=_usb_number(item.pid),
                manufacturer=item.manufacturer,
                product=item.product,
                serial_number=item.serial_number,
            )
            for item in list_ports.comports()
        ),
        key=lambda port: port.device,
    )


def resolve_port(explicit_port: str | None) -> str:
    """Find the URT-2, which is any USB serial device that is not the ST-LINK."""
    if explicit_port:
        return explicit_port
    likely = [
        port
        for port in serial_ports()
        if port.looks_like_usb_serial and not port.is_st_link
    ]
    if len(likely) == 1:
        return likely[0].device
    if not likely:
        raise RuntimeError(
            "no likely URT-2 port found; run 'spotctl ports' (an ST-LINK "
            "console port is not a servo bus)"
        )
    raise RuntimeError(
        "multiple serial ports found; select one with --port: "
        + ", ".join(f"{port.device} ({port.usb_id})" for port in likely)
    )


def resolve_console_port(explicit_port: str | None) -> str:
    """Find the ST-LINK virtual COM port by its USB vendor ID.

    Rather than guess when the lookup is inconclusive, ask for an explicit
    port: sending console text to the URT-2, or Feetech packets to the
    console, fails silently instead of reporting anything useful.
    """
    if explicit_port:
        return explicit_port
    ports = serial_ports()
    candidates = [port for port in ports if port.is_st_link]
    if len(candidates) == 1:
        return candidates[0].device
    if len(candidates) > 1:
        raise RuntimeError(
            "multiple ST-LINK ports found; select one with --stm32-port: "
            + ", ".join(
                f"{port.device} ({port.usb_id}"
                + (f", SER={port.serial_number}" if port.serial_number else "")
                + ")"
                for port in candidates
            )
        )
    seen = ", ".join(f"{port.device} ({port.usb_id})" for port in ports)
    raise RuntimeError(
        "no ST-LINK virtual COM port found (USB vendor "
        f"{ST_LINK_VENDOR_ID:04x}); pass --stm32-port or set SPOT_STM32_PORT. "
        f"Ports seen: {seen or 'none'}"
    )


def console_exit_code(status: str) -> int:
    if status == "error":
        return 1
    if status == "stopped":
        return 130
    return 0


def run_console_command(
    console: Stm32Console,
    command: str,
    *,
    timeout: float | None,
    log=None,
) -> int:
    """Send one command, streaming the firmware output as it arrives."""

    def emit(line: str) -> None:
        print(line)
        if log is not None:
            log.write(line + "\n")

    if log is not None:
        log.write(f"# {command}\n")
    response = console.send(
        command,
        timeout=timeout if timeout is not None else -1.0,
        on_line=emit,
    )
    return console_exit_code(response.status)


def run_console_shell(
    console: Stm32Console,
    *,
    timeout: float | None,
    log=None,
) -> int:
    print(
        "STM32 console. Ctrl+C aborts a running motion, "
        "Ctrl+D or 'exit' quits."
    )
    last_code = 0
    while True:
        try:
            line = input("stm32# ").strip()
        except EOFError:
            print()
            return last_code
        except KeyboardInterrupt:
            # Nothing is running between commands; just start a fresh line.
            print()
            continue
        if not line:
            continue
        if line in {"exit", "quit"}:
            return last_code
        try:
            last_code = run_console_command(
                console, line, timeout=timeout, log=log
            )
        except ConsoleError as exc:
            print(f"error: {exc}", file=sys.stderr)
            last_code = 1


def run_console_script(
    console: Stm32Console,
    path: Path,
    *,
    timeout: float | None,
    log=None,
) -> int:
    """Run one command per line, stopping at the first failure.

    Blank lines and ``#`` comments are skipped so a script can double as a
    readable test procedure.
    """
    commands = [
        stripped
        for stripped in (line.strip() for line in path.read_text().splitlines())
        if stripped and not stripped.startswith("#")
    ]
    if not commands:
        raise ValueError(f"{path} contains no console commands")
    for command in commands:
        print(f"# {command}")
        code = run_console_command(
            console, command, timeout=timeout, log=log
        )
        if code != 0:
            print(
                f"stopping: '{command}' did not succeed",
                file=sys.stderr,
            )
            return code
    return 0


#: Firmware console commands promoted to top-level spotctl subcommands.
CONSOLE_ONLY_COMMANDS = frozenset(
    {"trot", "trotplace", "trot2", "jump", "targets", "profile", "imu", "balance"}
)

#: Commands both transports implement, routed by the attached device.
DUAL_COMMANDS = frozenset({"scan", "stand", "relax", "hold"})


def resolve_transport(args: argparse.Namespace) -> tuple[str, str]:
    """Decide which link to use and return ``(kind, port)``.

    ``kind`` is ``"stm32"`` for the ST-LINK text console or ``"urt2"`` for the
    direct Feetech bus.  An explicit ``--via`` or an explicit port wins;
    otherwise the attached USB device decides.  With one of each plugged in
    there is no right answer, so ask instead of guessing.
    """
    if args.via == "stm32":
        return "stm32", resolve_console_port(args.stm32_port)
    if args.via == "urt2":
        return "urt2", resolve_port(args.port)
    if args.stm32_port and args.port:
        raise RuntimeError(
            "both --port and --stm32-port are set; choose one with --via"
        )
    if args.stm32_port:
        return "stm32", args.stm32_port
    if args.port:
        return "urt2", args.port

    ports = serial_ports()
    st_link = [port for port in ports if port.is_st_link]
    urt2 = [
        port
        for port in ports
        if port.looks_like_usb_serial and not port.is_st_link
    ]
    if st_link and urt2:
        raise RuntimeError(
            "both an STM32 console and a URT-2 are attached; select one with "
            "--via stm32 or --via urt2 ("
            + ", ".join(
                f"{port.device} ({port.usb_id})" for port in st_link + urt2
            )
            + ")"
        )
    if st_link:
        return "stm32", resolve_console_port(None)
    if urt2:
        return "urt2", resolve_port(None)
    raise RuntimeError(
        "no STM32 console or URT-2 found; run 'spotctl ports'"
    )


def console_line_for(args: argparse.Namespace) -> str:
    """Translate a routed spotctl command into one firmware console line.

    Options that only make sense on the direct Feetech bus are rejected
    rather than silently dropped, because over this link the firmware owns
    the trajectory.
    """
    command = args.command
    if command == "scan":
        if args.min_id != 1 or args.max_id != 253:
            raise ValueError(
                "the STM32 console always scans IDs 1..12; drop --min-id and "
                "--max-id, or use --via urt2"
            )
        return "scan"
    if command in {"stand", "relax", "hold"}:
        if getattr(args, "leg", None):
            raise ValueError(
                f"'{command} --leg' needs a URT-2 direct connection; the "
                "STM32 console always moves all twelve joints"
            )
        return command
    if command == "targets":
        return "targets"
    if command == "profile":
        if args.speed is None and args.accel is None:
            return "profile"
        if args.speed is None or args.accel is None:
            raise ValueError("profile takes both SPEED and ACCEL, or neither")
        return f"profile {args.speed} {args.accel}"
    if command in {"imu", "balance"}:
        return command if args.mode is None else f"{command} {args.mode}"
    if command in {"trot", "trotplace", "trot2", "jump"}:
        if args.cycles is None and args.period_ms is not None:
            raise ValueError("PERIOD_MS requires CYCLES")
        parts = [command]
        if args.cycles is not None:
            parts.append(str(args.cycles))
        if args.period_ms is not None:
            parts.append(str(args.period_ms))
        return " ".join(parts)
    raise ValueError(f"'{command}' has no STM32 console equivalent")


def run_routed_console_command(args: argparse.Namespace, port: str) -> int:
    """Run one routed command over the STM32 console."""
    line = console_line_for(args)
    log_file = None
    try:
        if args.log is not None:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log_file = args.log.open("a", encoding="utf-8")
            started = datetime.now().isoformat(timespec="seconds")
            log_file.write(f"\n=== {started} {port} ===\n")
        with Stm32Console(port, args.console_baudrate) as console:
            console.sync()
            return run_console_command(
                console, line, timeout=args.console_timeout, log=log_file
            )
    finally:
        if log_file is not None:
            log_file.close()


def run_console(args: argparse.Namespace) -> int:
    """Open the STM32 console and dispatch the requested console action."""
    port = resolve_console_port(args.stm32_port)
    log_file = None
    try:
        if args.log is not None:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log_file = args.log.open("a", encoding="utf-8")
            started = datetime.now().isoformat(timespec="seconds")
            log_file.write(f"\n=== {started} {port} ===\n")

        with Stm32Console(port, args.console_baudrate) as console:
            console.sync()
            if args.console_command == "send":
                return run_console_command(
                    console,
                    " ".join(args.words),
                    timeout=args.console_timeout,
                    log=log_file,
                )
            if args.console_command == "script":
                return run_console_script(
                    console, args.path, timeout=args.console_timeout,
                    log=log_file,
                )
            return run_console_shell(
                console, timeout=args.console_timeout, log=log_file
            )
    finally:
        if log_file is not None:
            log_file.close()


def resolve_gait(args: argparse.Namespace) -> GaitParameters:
    preset = PRESETS[args.preset]
    gait = GaitParameters(
        pattern=args.gait,
        period=args.period if args.period is not None else preset.period,
        hip_amplitude=(
            args.hip if args.hip is not None else preset.hip_amplitude
        ),
        lift_amplitude=(
            args.lift if args.lift is not None else preset.lift_amplitude
        ),
        crouch_amplitude=(
            args.crouch
            if args.crouch is not None
            else preset.crouch_amplitude
        ),
        speed=args.speed if args.speed is not None else preset.speed,
        acceleration=(
            args.accel if args.accel is not None else preset.acceleration
        ),
        duty_factor=(
            args.duty
            if args.duty is not None
            else preset.duty_factor if args.gait == "trot" else 0.75
        ),
        control_rate=(
            args.rate if args.rate is not None else preset.control_rate
        ),
        stance_j1_angle=(
            args.stance_j1
            if args.stance_j1 is not None
            else preset.stance_j1_angle
        ),
        stance_j2_angle=(
            args.stance_j2
            if args.stance_j2 is not None
            else preset.stance_j2_angle
        ),
        stance_j3_angle=(
            args.stance_j3
            if args.stance_j3 is not None
            else preset.stance_j3_angle
        ),
        weight_shift_amplitude=(
            args.weight_shift
            if args.weight_shift is not None
            else preset.weight_shift_amplitude
        ),
        preload_amplitude=(
            args.preload
            if args.preload is not None
            else preset.preload_amplitude
        ),
    )
    gait.validate()
    return gait


def show_ports() -> None:
    ports = serial_ports()
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(port.summary())


def show_status(robot: SpotRobot) -> None:
    robot.require_all()
    states = robot.read_states()
    print("Leg Joint ID Position Speed Load Voltage Temp HW Current Moving")
    print("--- ----- -- -------- ----- ---- ------- ---- -- ------- ------")
    for joint in robot.config.joints:
        state = states[joint.servo_id]
        print(
            f"{joint.leg:>3} {joint.joint:>5} {joint.servo_id:>2} "
            f"{state.position:>8} {state.speed:>5} {state.load:>4} "
            f"{state.voltage:>6.1f}V {state.temperature:>3}C "
            f"{state.hardware_error:>2} {state.current:>7} "
            f"{'yes' if state.moving else 'no':>6}"
        )


def monitor_contacts(
    robot: SpotRobot,
    *,
    baseline_seconds: float,
    duration: float,
    rate: float,
    threshold: int,
    release: int,
    on_samples: int,
    off_samples: int,
    verbose: bool,
) -> None:
    """Calibrate unloaded load values, then report inferred foot contact."""
    if not 0.2 <= baseline_seconds <= 10.0:
        raise ValueError("baseline must be between 0.2 and 10 seconds")
    if not 0.5 <= duration <= 300.0:
        raise ValueError("duration must be between 0.5 and 300 seconds")
    if not 1.0 <= rate <= 50.0:
        raise ValueError("contact rate must be between 1 and 50 Hz")

    # Validate threshold and debounce options before starting a timed read.
    empty_baseline = {
        (leg, joint_number): 0
        for leg in LEGS
        for joint_number in (2, 3)
    }
    LoadContactEstimator(
        empty_baseline,
        engage_threshold=threshold,
        release_threshold=release,
        engage_samples=on_samples,
        release_samples=off_samples,
    )

    robot.require_all()
    interval = 1.0 / rate
    sample_count = max(2, round(baseline_seconds * rate))
    samples = []
    print(
        f"Collecting unloaded baseline for {sample_count / rate:.1f}s; "
        "keep all four feet clear of the ground."
    )
    started_at = time.monotonic()
    for index in range(sample_count):
        samples.append(robot.read_leg_loads())
        deadline = started_at + (index + 1) * interval
        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    baseline = {
        key: round(statistics.median(sample[key] for sample in samples))
        for key in empty_baseline
    }
    estimator = LoadContactEstimator(
        baseline,
        engage_threshold=threshold,
        release_threshold=release,
        engage_samples=on_samples,
        release_samples=off_samples,
    )
    print("Unloaded baseline (signed STS load units):")
    for leg in LEGS:
        print(f"  {leg}: J2={baseline[(leg, 2)]:+d}, J3={baseline[(leg, 3)]:+d}")
    print(
        f"Monitoring for {duration:g}s at {rate:g}Hz; "
        f"contact >= {threshold}, release <= {release}."
    )
    print("Time    FL          FR          RL          RR")

    started_at = time.monotonic()
    frame = 0
    next_periodic_report = 0.0
    peak_scores = {leg: 0 for leg in LEGS}
    contact_events = {leg: 0 for leg in LEGS}
    while True:
        elapsed = time.monotonic() - started_at
        if elapsed >= duration:
            break
        estimates = estimator.update(robot.read_leg_loads())
        for leg, estimate in estimates.items():
            peak_scores[leg] = max(peak_scores[leg], estimate.score)
            if estimate.changed and estimate.contact:
                contact_events[leg] += 1
        changed = any(estimate.changed for estimate in estimates.values())
        if verbose or changed or elapsed >= next_periodic_report:
            cells = []
            for leg in LEGS:
                estimate = estimates[leg]
                marker = "ON " if estimate.contact else "off"
                if verbose:
                    cells.append(
                        f"{marker}:{estimate.score:>3}"
                        f"({estimate.j2_delta:+d},{estimate.j3_delta:+d})"
                    )
                else:
                    cells.append(f"{marker}:{estimate.score:>3}")
            print(f"{elapsed:5.1f}s  " + "  ".join(cells))
            next_periodic_report = elapsed + 1.0
        frame += 1
        deadline = started_at + frame * interval
        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    print("Summary (peak score / contact events):")
    print(
        "  "
        + ", ".join(
            f"{leg}={peak_scores[leg]}/{contact_events[leg]}"
            for leg in LEGS
        )
    )


def analyze_load_profile(profile: Path, output: Path | None = None) -> Path:
    baseline = DynamicLoadBaseline.from_csv(profile)
    destination = output or profile.with_suffix(".baseline.json")
    baseline.save(destination)
    print("Leg Joint ID NoiseP90 NoiseP95 NoiseMax Engage Release")
    print("--- ----- -- -------- -------- -------- ------ -------")
    for servo_id, model in sorted(baseline.models.items()):
        print(
            f"{model['leg']:>3} {model['joint']:>5} {servo_id:>2} "
            f"{model['noise_p90']:>8g} {model['noise_p95']:>8g} "
            f"{model['noise_max']:>8g} {model['engage_threshold']:>6} "
            f"{model['release_threshold']:>7}"
        )
    print(f"Dynamic load baseline: {destination}")
    return destination


def diagnose_servo(bus: ServoBus, servo_id: int) -> None:
    if not 1 <= servo_id <= 253:
        raise ValueError("servo ID must be between 1 and 253")
    servo = STS3215(bus, servo_id)
    if not servo.ping():
        raise RuntimeError(f"servo {servo_id} did not respond")
    values = servo.read_diagnostics()
    state = values.state
    print(f"ID:             {servo_id}")
    print(f"Operating mode: {values.operating_mode}")
    print(f"Torque enabled: {'yes' if values.torque_enabled else 'no'}")
    print(f"Acceleration:   {values.acceleration}")
    print(f"Goal position:  {values.goal_position}")
    print(f"Goal speed:     {values.goal_speed}")
    print(f"Torque limit:   {values.torque_limit}")
    print(f"Position:       {state.position}")
    print(f"Speed:          {state.speed}")
    print(f"Load:           {state.load}")
    print(f"Voltage:        {state.voltage:.1f} V")
    print(f"Temperature:    {state.temperature} C")
    print(f"HW error:       {state.hardware_error}")
    print(f"Current:        {state.current}")
    print(f"Moving:         {'yes' if state.moving else 'no'}")


def read_direction(prompt: str) -> int:
    while True:
        value = input(prompt).strip()
        if value in {"1", "+1"}:
            return 1
        if value == "-1":
            return -1
        print("Enter +1 or -1.")


def configure_directions(config: SpotConfig) -> None:
    print("Set motor signs for positive canonical joint angles.")
    print("Use the same geometric positive direction for every leg.")
    directions = {}
    for leg in ("FL", "FR", "RL", "RR"):
        side_id = config.joint(leg, 1).servo_id
        hip_id = config.joint(leg, 2).servo_id
        knee_id = config.joint(leg, 3).servo_id
        print(f"{leg}: J1=ID {side_id}, J2=ID {hip_id}, J3=ID {knee_id}")
        directions[leg] = (
            read_direction("  J1 positive-angle motor sign (+1/-1): "),
            read_direction("  J2 positive-angle motor sign (+1/-1): "),
            read_direction("  J3 positive-angle motor sign (+1/-1): "),
        )
    config.set_joint_directions(directions)
    config.save()
    print(f"Saved canonical joint directions: {config.path}")


def configure_mapping(config: SpotConfig) -> None:
    print("Enter three servo IDs for each leg in J1 J2 J3 order.")
    mapping = {}
    used = set()
    for leg in ("FL", "FR", "RL", "RR"):
        while True:
            raw = input(f"{leg} IDs: ").split()
            try:
                ids = [int(value) for value in raw]
            except ValueError:
                print("Enter exactly three integer IDs.")
                continue
            if len(ids) != 3 or any(not 1 <= value <= 253 for value in ids):
                print("Enter exactly three IDs in the 1..253 range.")
                continue
            if used.intersection(ids) or len(set(ids)) != 3:
                print("Servo IDs must be unique.")
                continue
            for joint_number, servo_id in enumerate(ids, start=1):
                mapping[(leg, joint_number)] = servo_id
            used.update(ids)
            break
    config.remap_ids(mapping)
    config.save()
    print(f"Saved servo mapping: {config.path}")


def show_calibration(robot: SpotRobot) -> None:
    print("Leg Joint ID Offset Center Present")
    print("--- ----- -- ------ ------ -------")
    positions = robot.read_positions()
    for joint in robot.config.joints:
        print(
            f"{joint.leg:>3} {joint.joint:>5} {joint.servo_id:>2} "
            f"{joint.offset:>+6} {joint.center:>6} "
            f"{positions[joint.servo_id]:>7}"
        )


def run_calibration(
    robot: SpotRobot,
    *,
    speed: int,
    acceleration: int,
    max_offset: int,
) -> None:
    if not 1 <= speed <= 3400 or not 0 <= acceleration <= 254:
        raise ValueError("invalid calibration speed or acceleration")
    if not 1 <= max_offset <= 1000:
        raise ValueError("max-offset must be between 1 and 1000")
    robot.prepare_for_motion(speed=speed, acceleration=acceleration)
    selected_leg = "FL"
    selected_joint = 1
    adjustments = {"+1": 1, "-1": -1, "+5": 5, "-5": -5,
                   "+10": 10, "-10": -10}
    print("Commands: leg FL | joint 1 | +/-1 | +/-5 | +/-10 | zero | show | quit")
    show_calibration(robot)
    while True:
        joint = robot.config.joint(selected_leg, selected_joint)
        command = input(
            f"[{selected_leg} J{selected_joint} ID={joint.servo_id}]> "
        ).strip()
        parts = command.upper().split()
        if len(parts) == 2 and parts[0] == "LEG" and parts[1] in {"FL", "FR", "RL", "RR"}:
            selected_leg = parts[1]
        elif len(parts) == 2 and parts[0] == "JOINT" and parts[1] in {"1", "2", "3"}:
            selected_joint = int(parts[1])
        elif command in adjustments or command.lower() == "zero":
            new_offset = (
                0 if command.lower() == "zero"
                else joint.offset + adjustments[command]
            )
            if abs(new_offset) > max_offset:
                print(f"Offset is limited to +/-{max_offset}.")
                continue
            target = robot.config.reference_center + new_offset
            if not joint.minimum <= target <= joint.maximum:
                print("Target is outside this joint's limits.")
                continue
            STS3215(robot.bus, joint.servo_id).move(
                target, speed=speed, acceleration=acceleration
            )
            robot.config.set_joint_center(joint.servo_id, target)
            robot.config.save()
            print(f"Saved ID {joint.servo_id}: offset={new_offset:+d}, center={target}")
        elif parts == ["SHOW"]:
            show_calibration(robot)
        elif parts in (["QUIT"], ["EXIT"], ["Q"]):
            return
        elif command:
            print("Unknown command. Use leg, joint, +/-1/5/10, zero, show, or quit.")


def capture_stand(robot: SpotRobot, leg: str | None = None) -> dict[int, int]:
    """Capture the current physical pose as calibrated neutral centers."""
    robot.require_all()
    positions = robot.read_positions()
    selected = (
        {
            robot.config.joint(leg, joint_number).servo_id
            for joint_number in (1, 2, 3)
        }
        if leg else set(robot.config.servo_ids)
    )
    for servo_id, position in positions.items():
        if servo_id not in selected:
            continue
        robot.config.set_joint_center(servo_id, position)
    robot.config.save()
    return {servo_id: positions[servo_id] for servo_id in sorted(selected)}


def change_servo_id(
    port: str,
    old_id: int,
    new_id: int,
    *,
    baudrate: int = 1_000_000,
    scan_max: int = 253,
    reconnect_wait: float = 0.5,
) -> None:
    if not 1 <= old_id <= 253 or not 1 <= new_id <= 253:
        raise ValueError("old and new IDs must be between 1 and 253")
    if old_id == new_id:
        raise ValueError("old and new IDs are the same")
    if not max(old_id, new_id) <= scan_max <= 253:
        raise ValueError("scan-max must include both IDs and be at most 253")

    print(f"Checking IDs 1..{scan_max}; connect only the target servo.")
    with ServoBus(port, baudrate) as bus:
        found = bus.scan(range(1, scan_max + 1))
        if found != [old_id]:
            detail = ", ".join(map(str, found)) if found else "none"
            raise RuntimeError(
                f"expected only servo {old_id}, but found: {detail}"
            )
        servo = STS3215(bus, old_id)
        servo.change_id(new_id)

    time.sleep(reconnect_wait)
    print("Reopening the port and verifying both IDs...")
    with ServoBus(port, baudrate) as bus:
        new_responds = bus.ping(new_id)
        old_responds = bus.ping(old_id)
    if not new_responds or old_responds:
        raise RuntimeError(
            "ID verification after reconnect failed: "
            f"new={new_responds}, old={old_responds}"
        )
    print(f"Changed servo ID: {old_id} -> {new_id}")


def swap_servo_ids_on_bus(
    bus: ServoBus,
    first_id: int,
    second_id: int,
    temp_id: int,
    *,
    eeprom_wait: float = 1.0,
) -> None:
    ids = (first_id, second_id, temp_id)
    if any(not 1 <= servo_id <= 253 for servo_id in ids):
        raise ValueError("servo IDs must be between 1 and 253")
    if len(set(ids)) != 3:
        raise ValueError("first, second, and temporary IDs must be different")
    missing = [
        servo_id
        for servo_id in (first_id, second_id)
        if not bus.ping(servo_id)
    ]
    if missing:
        raise RuntimeError(
            "servo IDs did not respond: " + ", ".join(map(str, missing))
        )
    if bus.ping(temp_id):
        raise RuntimeError(f"temporary servo ID {temp_id} is already in use")

    STS3215(bus, first_id).change_id(temp_id, eeprom_wait=eeprom_wait)
    STS3215(bus, second_id).change_id(first_id, eeprom_wait=eeprom_wait)
    STS3215(bus, temp_id).change_id(second_id, eeprom_wait=eeprom_wait)

    if not bus.ping(first_id) or not bus.ping(second_id) or bus.ping(temp_id):
        raise RuntimeError("servo ID swap verification failed")


def swap_servo_ids(
    port: str,
    first_id: int,
    second_id: int,
    temp_id: int,
    *,
    baudrate: int = 1_000_000,
    reconnect_wait: float = 0.5,
) -> None:
    print(
        f"Swapping EEPROM IDs: {first_id} -> {temp_id}, "
        f"{second_id} -> {first_id}, {temp_id} -> {second_id}"
    )
    with ServoBus(port, baudrate) as bus:
        swap_servo_ids_on_bus(bus, first_id, second_id, temp_id)

    time.sleep(reconnect_wait)
    with ServoBus(port, baudrate) as bus:
        first_responds = bus.ping(first_id)
        second_responds = bus.ping(second_id)
        temp_responds = bus.ping(temp_id)
    if not first_responds or not second_responds or temp_responds:
        raise RuntimeError(
            "ID verification after reconnect failed: "
            f"first={first_responds}, second={second_responds}, temp={temp_responds}"
        )
    print(f"Swapped servo EEPROM IDs: {first_id} <-> {second_id}")


def apply_pose(
    robot: SpotRobot,
    name: str,
    *,
    speed: int,
    acceleration: int,
    timeout: float,
    tolerance: int,
) -> None:
    if name in {"neutral", "stand"}:
        targets = robot.config.pose("neutral")
    elif name == "stand45":
        targets = robot.config.stand45_targets()
    elif name == "landing":
        targets = robot.config.landing_targets()
    else:
        targets = robot.config.pose(name)
    apply_targets(
        robot,
        targets,
        speed=speed,
        acceleration=acceleration,
        timeout=timeout,
        tolerance=tolerance,
    )


def apply_targets(
    robot: SpotRobot,
    targets: dict[int, int],
    *,
    speed: int,
    acceleration: int,
    timeout: float,
    tolerance: int,
) -> None:
    if not 1 <= speed <= 3400:
        raise ValueError("speed must be between 1 and 3400")
    if not 0 <= acceleration <= 254:
        raise ValueError("acceleration must be between 0 and 254")
    if not 0.1 <= timeout <= 60.0:
        raise ValueError("timeout must be between 0.1 and 60 seconds")
    if not 0 <= tolerance <= 500:
        raise ValueError("tolerance must be between 0 and 500 ticks")
    current = robot.prepare_for_motion(speed=speed, acceleration=acceleration)
    synchronized_speeds = robot.synchronized_arrival_speeds(
        current,
        targets,
        speed_cap=speed,
        acceleration=acceleration,
    )
    robot.move_and_wait(
        targets,
        speed=synchronized_speeds,
        acceleration=acceleration,
        timeout=timeout,
        tolerance=tolerance,
    )


def default_load_profile_path() -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_PROFILE_DIR / f"air_gait_loads_{timestamp}.csv"


def run_walk(
    robot: SpotRobot,
    gait: GaitParameters,
    cycles: int,
    *,
    profile_path: Path | None = None,
    load_rate: float = 5.0,
    load_baseline: DynamicLoadBaseline | None = None,
) -> None:
    if not 1 <= cycles <= 20:
        raise ValueError("cycles must be between 1 and 20")
    if profile_path is not None and not 0.5 <= load_rate <= 10.0:
        raise ValueError("load-rate must be between 0.5 and 10 Hz per servo")

    base = robot.config.angles_to_targets(
        {
            (leg, joint_number): angle
            for leg in ("FL", "FR", "RL", "RR")
            for joint_number, angle in (
                (1, gait.stance_j1_angle),
                (2, gait.stance_j2_angle),
                (3, gait.stance_j3_angle),
            )
        }
    )
    # Set one ordinary STS position profile, enable torque, and then only
    # stream absolute goal positions. No status reads or arrival polling.
    robot.sync_move(
        base, speed=gait.speed, acceleration=gait.acceleration
    )
    robot.set_torque(True)
    time.sleep(1.0)

    interval = 1.0 / gait.control_rate
    total_duration = cycles * gait.period
    total_frames = round(total_duration * gait.control_rate)
    ramp_duration = min(0.5, total_duration / 2.0)
    started_at = time.monotonic()

    profile_file = None
    profile_writer = None
    sample_schedule: dict[tuple[str, int], float] = {}
    profile_samples = 0
    maximum_read_time = 0.0
    late_frames = 0
    residual_peaks = {leg: 0.0 for leg in LEGS}
    residual_exceedances = {leg: 0 for leg in LEGS}
    effective_load_rate = min(load_rate, gait.control_rate / 8.0)
    if profile_path is not None:
        profile_path.parent.mkdir(parents=True, exist_ok=True)
        profile_file = profile_path.open("w", encoding="utf-8", newline="")
        profile_writer = csv.writer(profile_file)
        profile_writer.writerow(
            (
                "elapsed_s",
                "phase",
                "amplitude",
                "leg",
                "leg_phase",
                "planned_contact",
                "joint",
                "servo_id",
                "target_position",
                "present_position",
                "position_error",
                "speed",
                "load",
                "current",
                "moving",
                "read_time_ms",
                "frame_late_ms",
                "expected_load",
                "load_residual",
                "residual_threshold",
                "residual_exceeded",
            )
        )
        sample_keys = [
            (leg, joint_number)
            for leg in ("FL", "FR", "RL", "RR")
            for joint_number in (2, 3)
        ]
        # Stagger the eight servos so at most one request is sent per 50 Hz
        # position-control frame at the default 5 Hz per-servo rate.
        sample_schedule = {
            key: index / (len(sample_keys) * effective_load_rate)
            for index, key in enumerate(sample_keys)
        }

    try:
        for frame in range(total_frames):
            elapsed = frame * interval
            remaining = total_duration - elapsed
            ramp_progress = min(1.0, elapsed / ramp_duration, remaining / ramp_duration)
            amplitude = min(
                1.0,
                max(0.0, SpotRobot._smoothstep(max(0.0, ramp_progress))),
            )
            phase = (elapsed / gait.period) % 1.0
            targets = robot.gait_targets(
                phase, base, gait, amplitude_scale=amplitude
            )
            robot.sync_positions(targets)

            if profile_writer is not None:
                due = [
                    (due_at, key)
                    for key, due_at in sample_schedule.items()
                    if due_at <= elapsed + 1e-9
                ]
                if due:
                    _, (leg, joint_number) = min(due)
                    joint = robot.config.joint(leg, joint_number)
                    read_started = time.monotonic()
                    state = STS3215(robot.bus, joint.servo_id).read_state()
                    read_finished = time.monotonic()
                    read_time = read_finished - read_started
                    deadline = started_at + (frame + 1) * interval
                    frame_late = max(0.0, read_finished - deadline)
                    leg_phase = (
                        phase + SpotRobot.PHASE_OFFSETS[gait.pattern][leg]
                    ) % 1.0
                    expected_load = ""
                    load_residual = ""
                    residual_threshold = ""
                    residual_exceeded = ""
                    residual_active = (
                        load_baseline is not None
                        and amplitude >= 0.999999
                        and elapsed >= gait.period
                    )
                    if residual_active:
                        expected = load_baseline.expected_load(
                            joint.servo_id, leg_phase
                        )
                        residual = abs(state.load) - expected
                        threshold = load_baseline.models[joint.servo_id][
                            "engage_threshold"
                        ]
                        exceeded = abs(residual) >= threshold
                        expected_load = f"{expected:.3f}"
                        load_residual = f"{residual:.3f}"
                        residual_threshold = threshold
                        residual_exceeded = int(exceeded)
                        residual_peaks[leg] = max(
                            residual_peaks[leg], abs(residual)
                        )
                        residual_exceedances[leg] += int(exceeded)
                    profile_writer.writerow(
                        (
                            f"{elapsed:.6f}",
                            f"{phase:.6f}",
                            f"{amplitude:.6f}",
                            leg,
                            f"{leg_phase:.6f}",
                            int(leg_phase < gait.duty_factor),
                            joint_number,
                            joint.servo_id,
                            targets[joint.servo_id],
                            state.position,
                            state.position - targets[joint.servo_id],
                            state.speed,
                            state.load,
                            state.current,
                            int(state.moving),
                            f"{read_time * 1000.0:.3f}",
                            f"{frame_late * 1000.0:.3f}",
                            expected_load,
                            load_residual,
                            residual_threshold,
                            residual_exceeded,
                        )
                    )
                    profile_samples += 1
                    maximum_read_time = max(maximum_read_time, read_time)
                    late_frames += int(frame_late > 0.0)
                    sample_schedule[(leg, joint_number)] += (
                        1.0 / effective_load_rate
                    )

            deadline = started_at + (frame + 1) * interval
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        if profile_file is not None:
            profile_file.close()
        try:
            robot.sync_move(
                base, speed=gait.speed, acceleration=gait.acceleration
            )
        except Exception:
            pass
    if profile_path is not None:
        print(
            f"Load profile: {profile_path} ({profile_samples} samples, "
            f"{effective_load_rate:.2f}Hz/servo, "
            f"max read={maximum_read_time * 1000.0:.2f}ms, "
            f"late frames={late_frames})"
        )
    if load_baseline is not None:
        print(
            "Residual check (peak / threshold exceedances): "
            + ", ".join(
                f"{leg}={residual_peaks[leg]:.0f}/{residual_exceedances[leg]}"
                for leg in LEGS
            )
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "ports":
            show_ports()
            return 0

        if args.command == "analyze-loads":
            analyze_load_profile(args.profile, args.output)
            return 0

        if args.command in {"configure-mapping", "configure-directions"}:
            config = SpotConfig.load(args.config)
            if args.command == "configure-mapping":
                configure_mapping(config)
            else:
                configure_directions(config)
            return 0

        if args.command == "console":
            return run_console(args)

        transport, port = resolve_transport(args)
        if transport == "stm32":
            if args.command in CONSOLE_ONLY_COMMANDS | DUAL_COMMANDS:
                return run_routed_console_command(args, port)
            raise RuntimeError(
                f"'{args.command}' needs a URT-2 connected directly to this "
                "computer; the STM32 console does not implement it"
            )
        if args.command in CONSOLE_ONLY_COMMANDS:
            raise RuntimeError(
                f"'{args.command}' runs on the STM32; connect the ST-LINK "
                "port, or use 'spotctl walk' over the URT-2"
            )

        if args.command == "scan":
            if not 0 <= args.min_id <= args.max_id <= 253:
                raise ValueError("ID range must satisfy 0 <= min-id <= max-id <= 253")
            with ServoBus(port, args.baudrate) as bus:
                found = bus.scan(range(args.min_id, args.max_id + 1))
            print("Found:", ", ".join(map(str, found)) if found else "none")
            return 0
        if args.command == "diagnose":
            with ServoBus(port, args.baudrate) as bus:
                diagnose_servo(bus, args.id)
            return 0
        if args.command == "change-id":
            change_servo_id(
                port,
                args.old_id,
                args.new_id,
                baudrate=args.baudrate,
                scan_max=args.scan_max,
            )
            return 0
        if args.command == "swap-ids":
            swap_servo_ids(
                port,
                args.first_id,
                args.second_id,
                args.temp_id,
                baudrate=args.baudrate,
            )
            return 0

        config = SpotConfig.load(args.config)
        with ServoBus(port, args.baudrate) as bus:
            robot = SpotRobot(bus, config)
            if args.command == "status":
                show_status(robot)
            elif args.command == "contacts":
                monitor_contacts(
                    robot,
                    baseline_seconds=args.baseline,
                    duration=args.duration,
                    rate=args.rate,
                    threshold=args.threshold,
                    release=args.release,
                    on_samples=args.on_samples,
                    off_samples=args.off_samples,
                    verbose=args.verbose,
                )
            elif args.command == "calibrate":
                run_calibration(
                    robot,
                    speed=args.speed,
                    acceleration=args.accel,
                    max_offset=args.max_offset,
                )
            elif args.command in {"capture-stand", "save-stand"}:
                positions = capture_stand(robot, args.leg)
                for servo_id in sorted(positions):
                    print(f"ID {servo_id:2}: center={positions[servo_id]}")
                scope = args.leg if args.leg else "all legs"
                print(f"Captured current pose as stand ({scope}): {config.path}")
            elif args.command == "save-pose":
                robot.require_all()
                config.set_pose(args.name, robot.read_positions())
                config.save()
                print(f"Saved pose {args.name!r}: {config.path}")
            elif args.command == "raw-center":
                targets = {
                    servo_id: config.reference_center
                    for servo_id in config.servo_ids
                }
                robot.prepare_for_motion(
                    speed=args.speed, acceleration=args.accel
                )
                robot.move_and_wait(
                    targets,
                    speed=args.speed,
                    acceleration=args.accel,
                    timeout=args.timeout,
                    tolerance=args.tolerance,
                )
                print(f"All servos reached raw center {config.reference_center}.")
            elif args.command == "relax":
                robot.require_all()
                if args.leg:
                    servo_ids = [
                        config.joint(args.leg, joint_number).servo_id
                        for joint_number in (1, 2, 3)
                    ]
                    for servo_id in servo_ids:
                        STS3215(bus, servo_id).enable_torque(False)
                    print(
                        f"Torque disabled for {args.leg}: "
                        + ", ".join(f"ID {servo_id}" for servo_id in servo_ids)
                    )
                else:
                    robot.set_torque(False)
                    print("Torque disabled on all 12 servos.")
            elif args.command == "hold":
                servo_ids = (
                    {
                        config.joint(args.leg, joint_number).servo_id
                        for joint_number in (1, 2, 3)
                    }
                    if args.leg else None
                )
                held = robot.hold_current(
                    servo_ids, speed=args.speed, acceleration=args.accel
                )
                scope = args.leg if args.leg else "all 12 servos"
                print(
                    f"Torque enabled at current positions for {scope}: "
                    + ", ".join(
                        f"ID {servo_id}={position}"
                        for servo_id, position in held.items()
                    )
                )
            elif args.command in {"pose", "apply-pose"}:
                apply_pose(
                    robot,
                    args.name,
                    speed=args.speed,
                    acceleration=args.accel,
                    timeout=args.timeout,
                    tolerance=args.tolerance,
                )
                print(f"Applied pose: {args.name}")
            elif args.command in {"stand", "stand45", "landing"}:
                if args.command == "stand" and args.leg:
                    robot.require_all()
                    neutral = config.pose("neutral")
                    targets = {
                        config.joint(args.leg, joint_number).servo_id:
                        neutral[config.joint(args.leg, joint_number).servo_id]
                        for joint_number in (1, 2, 3)
                    }
                    robot.move_subset_and_wait(
                        targets,
                        speed=args.speed,
                        acceleration=args.accel,
                        timeout=args.timeout,
                        tolerance=args.tolerance,
                    )
                elif args.command == "stand":
                    targets = config.pose("neutral")
                elif args.command == "stand45":
                    targets = config.stand45_targets()
                else:
                    targets = config.landing_targets()
                if not (args.command == "stand" and args.leg):
                    apply_targets(
                        robot,
                        targets,
                        speed=args.speed,
                        acceleration=args.accel,
                        timeout=args.timeout,
                        tolerance=args.tolerance,
                    )
                scope = f" {args.leg}" if args.command == "stand" and args.leg else ""
                print(f"Applied stance: {args.command}{scope}")
            elif args.command == "walk":
                gait = resolve_gait(args)
                profile_path = args.profile_output
                if (
                    args.profile_loads or args.load_baseline is not None
                ) and profile_path is None:
                    profile_path = default_load_profile_path()
                if args.profile_output is not None:
                    profile_path = args.profile_output
                load_baseline = (
                    DynamicLoadBaseline.load(args.load_baseline)
                    if args.load_baseline is not None
                    else None
                )
                print(
                    f"{gait.pattern.title()} gait: {args.preset}, "
                    f"{args.cycles} cycle(s), "
                    f"period={gait.period:.1f}s, rate={gait.control_rate:.0f}Hz, "
                    f"J2={gait.hip_amplitude:g}deg, "
                    f"J3={gait.lift_amplitude:g}deg, "
                    f"speed cap={gait.speed}, accel={gait.acceleration}, "
                    f"stance=J1 {gait.stance_j1_angle:g}deg/"
                    f"J2 {gait.stance_j2_angle:g}deg/"
                    f"J3 {gait.stance_j3_angle:g}deg, "
                    f"duty={gait.duty_factor:.2f}, "
                    f"shift={gait.weight_shift_amplitude:g}deg, "
                    f"preload={gait.preload_amplitude:g}deg"
                )
                run_walk(
                    robot,
                    gait,
                    args.cycles,
                    profile_path=profile_path,
                    load_rate=args.load_rate,
                    load_baseline=load_baseline,
                )
                print("Gait complete; returned to the selected walking stance.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.", file=sys.stderr)
        return 130
    except (EOFError, ImportError, KeyError, OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
