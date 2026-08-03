"""Command-line interface for URT-2 and the twelve-servo Spot Micro."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from .bus import ServoBus
from .spot import GaitParameters, SpotConfig, SpotRobot
from .sts3215 import STS3215

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "joints.json"
PRESETS = {
    "test": GaitParameters(
        period=4.0,
        hip_amplitude=8.8,
        lift_amplitude=12.3,
        crouch_amplitude=0,
        speed=60,
        acceleration=30,
    ),
    "natural": GaitParameters(),
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
        "--speed", type=int, help="maximum STS speed-register value"
    )
    walk.add_argument(
        "--accel", type=int, help="STS acceleration-register value"
    )
    walk.add_argument(
        "--rate", type=float, help="position update rate in Hz (default: 30)"
    )
    return parser.parse_args(argv)


def serial_ports() -> list[tuple[str, str]]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise ImportError(
            "pyserial is required; run: pip install -r requirements.txt"
        ) from exc
    return sorted(
        (
            (item.device, item.description or "unknown device")
            for item in list_ports.comports()
        ),
        key=lambda item: item[0],
    )


def resolve_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port
    ports = serial_ports()
    likely = [
        device
        for device, _ in ports
        if any(
            marker in device.lower()
            for marker in ("usbmodem", "usbserial", "ttyusb", "ttyacm")
        )
        or device.upper().startswith("COM")
    ]
    if len(likely) == 1:
        return likely[0]
    if not likely:
        raise RuntimeError("no likely URT-2 port found; run 'spotctl ports'")
    raise RuntimeError(
        "multiple serial ports found; select one with --port: "
        + ", ".join(likely)
    )


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
        duty_factor=0.65 if args.gait == "trot" else 0.75,
        control_rate=(
            args.rate if args.rate is not None else preset.control_rate
        ),
    )
    gait.validate()
    return gait


def show_ports() -> None:
    ports = serial_ports()
    if not ports:
        print("No serial ports found.")
        return
    for device, description in ports:
        print(f"{device:24} {description}")


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
    robot.prepare_for_motion(speed=speed, acceleration=acceleration)
    robot.move_and_wait(
        targets,
        speed=speed,
        acceleration=acceleration,
        timeout=timeout,
        tolerance=tolerance,
    )


def run_walk(robot: SpotRobot, gait: GaitParameters, cycles: int) -> None:
    if not 1 <= cycles <= 20:
        raise ValueError("cycles must be between 1 and 20")

    base = robot.config.stand45_targets()
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

            deadline = started_at + (frame + 1) * interval
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        try:
            robot.sync_move(
                base, speed=gait.speed, acceleration=gait.acceleration
            )
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "ports":
            show_ports()
            return 0

        if args.command in {"configure-mapping", "configure-directions"}:
            config = SpotConfig.load(args.config)
            if args.command == "configure-mapping":
                configure_mapping(config)
            else:
                configure_directions(config)
            return 0

        port = resolve_port(args.port)
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
                print(
                    f"{gait.pattern.title()} gait: {args.preset}, "
                    f"{args.cycles} cycle(s), "
                    f"period={gait.period:.1f}s, rate={gait.control_rate:.0f}Hz, "
                    f"J2={gait.hip_amplitude:g}deg, "
                    f"J3={gait.lift_amplitude:g}deg, "
                    f"speed cap={gait.speed}, accel={gait.acceleration}"
                )
                run_walk(robot, gait, args.cycles)
                print("Gait complete; returned to stand45.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.", file=sys.stderr)
        return 130
    except (EOFError, ImportError, KeyError, OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
