#!/usr/bin/env python3
"""Replay the real spotctl gait trajectory in the MuJoCo URDF model."""

from __future__ import annotations

import argparse
import math
import os
import platform
import time
from dataclasses import replace
from pathlib import Path

import mujoco

from servo import AttitudeController, ImuSample, SpotConfig, SpotRobot
from servo.cli import PRESETS


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "hardware" / "urdf" / "spot_omg.urdf"
SCENE = Path(__file__).resolve().parent / "spot_omg_scene.xml"
CONFIG = ROOT / "tools" / "servo_tool" / "config" / "joints.json"
LEGS = ("FL", "FR", "RL", "RR")
LEG_SIDE = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}
SIM_PRESETS = {
    **PRESETS,
    # A slow 4-second test cycle has diagonal phase offsets but tips onto a
    # third foot before the paired swing completes. This simulator-only preset
    # is fast enough to produce a genuine alternating diagonal contact gait.
    "sim-trot": replace(
        PRESETS["test"],
        period=0.8,
        lift_amplitude=30.0,
        duty_factor=0.50,
        control_rate=50.0,
        stance_j1_angle=4.0,
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gait", choices=("trot", "crawl"), default="trot")
    parser.add_argument("--preset", choices=tuple(SIM_PRESETS), default="test")
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--period", type=float)
    parser.add_argument("--hip", type=float)
    parser.add_argument("--lift", type=float)
    parser.add_argument("--crouch", type=float)
    parser.add_argument("--stance-j1", type=float)
    parser.add_argument("--stance-j2", type=float)
    parser.add_argument("--stance-j3", type=float)
    parser.add_argument("--duty", type=float)
    parser.add_argument("--weight-shift", type=float)
    parser.add_argument("--preload", type=float)
    parser.add_argument("--rate", type=float)
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="enable gravity, ground contact, floating base and position servos",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="seconds to settle on the ground before walking (default: 1.0)",
    )
    parser.add_argument(
        "--balance",
        action="store_true",
        help="use simulated body roll/pitch feedback to keep the body level",
    )
    parser.add_argument(
        "--balance-kp",
        type=float,
        help="attitude proportional gain; preset-specific default",
    )
    parser.add_argument(
        "--balance-kd",
        type=float,
        help="attitude derivative gain; preset-specific default",
    )
    parser.add_argument(
        "--balance-limit",
        type=float,
        help="maximum normalized leg-length correction; preset-specific default",
    )
    parser.add_argument(
        "--balance-mode",
        choices=("all-legs", "contact-aware"),
        default="contact-aware",
        help="select legacy all-leg or support/swing-aware stabilization",
    )
    parser.add_argument(
        "--j1-balance-gain",
        type=float,
        default=5.0,
        help="J1 roll correction in degrees per normalized control effort",
    )
    parser.add_argument(
        "--j1-balance-limit",
        type=float,
        default=5.0,
        help="maximum absolute J1 feedback correction in degrees",
    )
    parser.add_argument(
        "--foot-placement-gain",
        type=float,
        default=0.0,
        help="experimental swing-foot forward correction (default: disabled)",
    )
    parser.add_argument(
        "--foot-placement-limit",
        type=float,
        default=0.08,
        help="maximum normalized swing-foot forward correction",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate one cycle without opening the viewer",
    )
    return parser.parse_args()


def resolve_gait(args: argparse.Namespace):
    """Apply the same preset-overrides convention as spotctl walk."""
    gait = SIM_PRESETS[args.preset]
    overrides = {"pattern": args.gait}
    for argument, field in (
        ("period", "period"),
        ("hip", "hip_amplitude"),
        ("lift", "lift_amplitude"),
        ("crouch", "crouch_amplitude"),
        ("stance_j1", "stance_j1_angle"),
        ("stance_j2", "stance_j2_angle"),
        ("stance_j3", "stance_j3_angle"),
        ("duty", "duty_factor"),
        ("weight_shift", "weight_shift_amplitude"),
        ("preload", "preload_amplitude"),
        ("rate", "control_rate"),
    ):
        value = getattr(args, argument)
        if value is not None:
            overrides[field] = value
    gait = replace(gait, **overrides)
    gait.validate()
    balance_defaults = (
        (1.0, 0.04, 0.15)
        if args.preset == "sim-trot"
        else (0.6, 0.04, 0.10)
    )
    if args.balance_kp is None:
        args.balance_kp = balance_defaults[0]
    if args.balance_kd is None:
        args.balance_kd = balance_defaults[1]
    if args.balance_limit is None:
        args.balance_limit = balance_defaults[2]
    if not 1 <= args.cycles <= 1000:
        raise ValueError("cycles must be between 1 and 1000")
    if not 0.0 <= args.settle <= 10.0:
        raise ValueError("settle must be between 0 and 10 seconds")
    if args.balance and not args.dynamic:
        raise ValueError("--balance requires --dynamic")
    if not 0.0 <= args.balance_kp <= 5.0:
        raise ValueError("balance-kp must be between 0 and 5")
    if not 0.0 <= args.balance_kd <= 1.0:
        raise ValueError("balance-kd must be between 0 and 1")
    if not 0.0 < args.balance_limit <= 0.30:
        raise ValueError("balance-limit must be above 0 and at most 0.30")
    if not 0.0 <= args.j1_balance_gain <= 100.0:
        raise ValueError("j1-balance-gain must be between 0 and 100")
    if not 0.0 <= args.j1_balance_limit <= 15.0:
        raise ValueError("j1-balance-limit must be between 0 and 15 degrees")
    if not 0.0 <= args.foot_placement_gain <= 2.0:
        raise ValueError("foot-placement-gain must be between 0 and 2")
    if not 0.0 <= args.foot_placement_limit <= 0.30:
        raise ValueError("foot-placement-limit must be between 0 and 0.30")
    return gait


def stance_targets(config: SpotConfig, gait) -> dict[int, int]:
    return config.angles_to_targets(
        {
            (leg, joint_number): angle
            for leg in LEGS
            for joint_number, angle in (
                (1, gait.stance_j1_angle),
                (2, gait.stance_j2_angle),
                (3, gait.stance_j3_angle),
            )
        }
    )


def apply_targets(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: SpotConfig,
    targets: dict[int, int],
) -> None:
    """Convert hardware targets back to canonical angles and set MuJoCo qpos."""
    for leg in LEGS:
        for joint_number in (1, 2, 3):
            joint = config.joint(leg, joint_number)
            angle_deg = config.position_to_angle(
                leg, joint_number, targets[joint.servo_id]
            )
            name = f"{leg.lower()}_j{joint_number}"
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"URDF joint not found: {name}")
            data.qpos[model.jnt_qposadr[joint_id]] = math.radians(angle_deg)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def set_controls(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: SpotConfig,
    targets: dict[int, int],
) -> None:
    """Send canonical joint angles to the dynamic position actuators."""
    for leg in LEGS:
        for joint_number in (1, 2, 3):
            joint = config.joint(leg, joint_number)
            angle_deg = config.position_to_angle(
                leg, joint_number, targets[joint.servo_id]
            )
            actuator_name = f"{leg.lower()}_j{joint_number}_position"
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            if actuator_id < 0:
                raise ValueError(f"MuJoCo actuator not found: {actuator_name}")
            data.ctrl[actuator_id] = math.radians(angle_deg)


def initialize_on_ground(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: SpotConfig,
    base: dict[int, int],
) -> None:
    """Place the commanded stance just above the plane before gravity starts."""
    mujoco.mj_resetData(model, data)
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    if root_id < 0:
        raise ValueError("dynamic scene has no root free joint")
    root_qpos = model.jnt_qposadr[root_id]
    data.qpos[root_qpos : root_qpos + 7] = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    apply_targets(model, data, config, base)

    foot_bottoms = []
    for leg in ("fl", "fr", "rl", "rr"):
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot_geom"
        )
        if geom_id < 0:
            raise ValueError(f"dynamic scene foot geom not found: {leg}")
        foot_bottoms.append(
            data.geom_xpos[geom_id][2] - model.geom_size[geom_id][0]
        )
    data.qpos[root_qpos + 2] = -min(foot_bottoms) + 0.003
    data.qvel[:] = 0.0
    set_controls(model, data, config, base)
    mujoco.mj_forward(model, data)


def root_position(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, ...]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return tuple(float(value) for value in data.xpos[body_id])


def body_up_z(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Return the world-Z component of the body-local up axis."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return float(data.xmat[body_id][8])


def read_imu(model: mujoco.MjModel, data: mujoco.MjData) -> ImuSample:
    """Read the one body-mounted virtual IMU in BNO086-compatible form."""
    orientation_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "body_imu_orientation"
    )
    gyro_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "body_imu_gyro"
    )
    if orientation_id < 0 or gyro_id < 0:
        raise ValueError("dynamic scene is missing the body IMU sensors")

    orientation_address = model.sensor_adr[orientation_id]
    w, x, y, z = data.sensordata[
        orientation_address : orientation_address + 4
    ]
    gyro_address = model.sensor_adr[gyro_id]
    roll_rate, pitch_rate, _ = data.sensordata[
        gyro_address : gyro_address + 3
    ]

    # Intrinsic ZYX Euler angles from MuJoCo's world-frame (w, x, y, z)
    # quaternion. Only roll/pitch are used; yaw does not affect body leveling.
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(
        max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    )
    return ImuSample(
        roll=float(roll),
        pitch=float(pitch),
        roll_rate=float(roll_rate),
        pitch_rate=float(pitch_rate),
    )


def balance_targets(
    config: SpotConfig,
    robot: SpotRobot,
    targets: dict[int, int],
    *,
    sample: ImuSample,
    controller: AttitudeController,
    support_legs: set[str],
    mode: str,
    j1_gain: float,
    j1_limit: float,
    foot_placement_gain: float,
    foot_placement_limit: float,
) -> dict[int, int]:
    """Stabilize stance legs and place swing feet using one body IMU."""
    corrected = dict(targets)
    corrections = controller.leg_length_corrections(sample)
    roll_control, pitch_control = controller.axis_controls(sample)
    for leg in LEGS:
        is_support = mode == "all-legs" or leg in support_legs
        j1 = config.joint(leg, 1)
        j2 = config.joint(leg, 2)
        j3 = config.joint(leg, 3)
        lateral = config.position_to_angle(leg, 1, targets[j1.servo_id])
        upper = config.position_to_angle(leg, 2, targets[j2.servo_id])
        knee = config.position_to_angle(leg, 3, targets[j3.servo_id])
        forward, down = robot.leg_forward_kinematics(upper, knee)

        if mode == "contact-aware":
            # While planted, move the hip opposite the lean through the fixed
            # foot. While swinging, place the foot toward the falling side.
            j1_effort = LEG_SIDE[leg] * roll_control
            if not is_support:
                j1_effort = -j1_effort
            j1_correction = max(
                -j1_limit,
                min(j1_limit, j1_gain * j1_effort),
            )
            lateral += j1_correction

            # Pitch recovery is primarily a sagittal foot-placement problem.
            # Positive pitch leans forward, so put the next swing foot farther
            # forward in the robot coordinate system.
            if not is_support:
                placement = max(
                    -foot_placement_limit,
                    min(
                        foot_placement_limit,
                        foot_placement_gain * pitch_control,
                    ),
                )
                forward += config.gait_forward_signs[leg] * placement

        # Keep vertical IK continuous through liftoff and touchdown. Limiting
        # this term to measured contacts creates a target-position step on
        # these position-controlled servos and increases body oscillation.
        down_correction = corrections[leg]
        upper, knee = robot.leg_inverse_kinematics(
            forward, down + down_correction
        )
        corrected[j1.servo_id] = config.angle_to_position(leg, 1, lateral)
        corrected[j2.servo_id] = config.angle_to_position(leg, 2, upper)
        corrected[j3.servo_id] = config.angle_to_position(leg, 3, knee)
    config.validate_targets(corrected)
    return corrected


def scheduled_support_legs(phase: float, gait) -> set[str]:
    """Return legs commanded to be in stance at this gait phase."""
    offsets = SpotRobot.PHASE_OFFSETS[gait.pattern]
    return {
        leg
        for leg in LEGS
        if (phase + offsets[leg]) % 1.0 < gait.duty_factor
    }


def ground_contact_legs(
    model: mujoco.MjModel, data: mujoco.MjData
) -> set[str]:
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    feet = {
        leg.upper(): mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot_geom"
        )
        for leg in ("fl", "fr", "rl", "rr")
    }
    contacts = set()
    for contact in data.contact[: data.ncon]:
        for leg, foot in feet.items():
            if (contact.geom1 == ground and contact.geom2 == foot) or (
                contact.geom2 == ground and contact.geom1 == foot
            ):
                contacts.add(leg)
    return contacts


def amplitude_at(elapsed: float, total_duration: float) -> float:
    ramp_duration = min(0.5, total_duration / 2.0)
    remaining = total_duration - elapsed
    progress = min(1.0, elapsed / ramp_duration, remaining / ramp_duration)
    return SpotRobot._smoothstep(max(0.0, progress))


def run_dynamic(
    args: argparse.Namespace,
    gait,
    config: SpotConfig,
    robot: SpotRobot,
    base: dict[int, int],
) -> int:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    initialize_on_ground(model, data, config, base)

    viewer = None
    if not args.check:
        if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
            raise RuntimeError(
                "macOS live viewer requires mjpython; run: "
                ".venv/bin/mjpython simulation/mujoco/walk.py --dynamic"
            )
        import mujoco.viewer as mj_viewer

        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 140.0
        viewer.cam.elevation = -20.0

    wall_origin = time.monotonic() - data.time
    next_view_time = data.time
    attitude_controller = AttitudeController(
        kp=args.balance_kp,
        kd=args.balance_kd,
        correction_limit=args.balance_limit,
    )
    max_roll = 0.0
    max_pitch = 0.0
    min_body_z = math.inf
    max_body_z = -math.inf
    attitude_squared_sum = 0.0
    attitude_samples = 0
    gait_contact_samples = 0
    diagonal_contact_samples = 0

    def step_until(
        end_time: float,
        controls,
        *,
        measure: bool = False,
        measure_contacts: bool = False,
    ) -> int:
        nonlocal next_view_time
        nonlocal max_roll, max_pitch, min_body_z, max_body_z
        nonlocal attitude_squared_sum, attitude_samples
        nonlocal gait_contact_samples, diagonal_contact_samples
        contact_frames = 0
        next_control_time = data.time
        while data.time < end_time and (viewer is None or viewer.is_running()):
            if data.time + 1e-12 >= next_control_time:
                targets, support_legs = controls(data.time)
                if args.balance_mode == "contact-aware":
                    # Prefer measured contact over the planned gait phase.
                    # The phase schedule remains the fallback during brief
                    # contact transitions or before the first physics step.
                    measured_support = ground_contact_legs(model, data)
                    if measured_support:
                        support_legs = measured_support
                sample = read_imu(model, data)
                if args.balance:
                    targets = balance_targets(
                        config,
                        robot,
                        targets,
                        sample=sample,
                        controller=attitude_controller,
                        support_legs=support_legs,
                        mode=args.balance_mode,
                        j1_gain=args.j1_balance_gain,
                        j1_limit=args.j1_balance_limit,
                        foot_placement_gain=args.foot_placement_gain,
                        foot_placement_limit=args.foot_placement_limit,
                    )
                set_controls(model, data, config, targets)
                next_control_time += 1.0 / gait.control_rate
            mujoco.mj_step(model, data)
            contact_legs = ground_contact_legs(model, data)
            contact_frames += int(bool(contact_legs))
            if measure_contacts:
                gait_contact_samples += 1
                diagonal_contact_samples += contact_legs in (
                    {"FL", "RR"},
                    {"FR", "RL"},
                )
            if measure:
                sample = read_imu(model, data)
                max_roll = max(max_roll, abs(sample.roll))
                max_pitch = max(max_pitch, abs(sample.pitch))
                body_z = root_position(model, data)[2]
                min_body_z = min(min_body_z, body_z)
                max_body_z = max(max_body_z, body_z)
                attitude_squared_sum += sample.roll**2 + sample.pitch**2
                attitude_samples += 1
            if viewer is not None and data.time >= next_view_time:
                position = root_position(model, data)
                viewer.cam.lookat[:] = position
                viewer.sync()
                next_view_time += 1.0 / 60.0
                delay = wall_origin + data.time - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
        return contact_frames

    settle_end = data.time + args.settle
    contact_frames = step_until(settle_end, lambda _: (base, set(LEGS)))
    start = root_position(model, data)

    gait_started = data.time
    total_duration = args.cycles * gait.period

    def gait_controls(sim_time: float) -> tuple[dict[int, int], set[str]]:
        elapsed = min(total_duration, sim_time - gait_started)
        phase = (elapsed / gait.period) % 1.0
        targets = robot.gait_targets(
            phase,
            base,
            gait,
            amplitude_scale=amplitude_at(elapsed, total_duration),
        )
        return targets, scheduled_support_legs(phase, gait)

    contact_frames += step_until(
        gait_started + total_duration,
        gait_controls,
        measure=True,
        measure_contacts=True,
    )
    contact_frames += step_until(
        data.time + 0.5,
        lambda _: (base, set(LEGS)),
        measure=True,
    )
    finish = root_position(model, data)
    upright = body_up_z(model, data)
    fell = finish[2] < max(0.08, start[2] * 0.55) or upright < 0.5
    diagonal_ratio = (
        diagonal_contact_samples / gait_contact_samples
        if gait_contact_samples
        else 0.0
    )
    contact_gait = (
        "TROT" if gait.pattern == "trot" and diagonal_ratio >= 0.5
        else "NON_TROT_CONTACT"
    )
    attitude_rms = math.sqrt(
        attitude_squared_sum / attitude_samples
    ) if attitude_samples else 0.0

    if not all(math.isfinite(value) for value in (*data.qpos, *data.qvel)):
        raise RuntimeError("dynamic simulation produced a non-finite state")
    print(
        "Dynamic result: "
        f"delta=({finish[0] - start[0]:+.3f}, "
        f"{finish[1] - start[1]:+.3f})m, "
        f"body_z={finish[2]:.3f}m, up_z={upright:.3f}, "
        f"max_roll={math.degrees(max_roll):.2f}deg, "
        f"max_pitch={math.degrees(max_pitch):.2f}deg, "
        f"attitude_rms={math.degrees(attitude_rms):.2f}deg, "
        f"body_z_range={max_body_z - min_body_z:.3f}m, "
        f"diagonal_contact={diagonal_ratio:.1%}, "
        f"contact_frames={contact_frames}, "
        f"state={'FALLEN' if fell else 'UPRIGHT'}, gait={contact_gait}"
    )

    if viewer is not None:
        print("Gait complete; close the MuJoCo window to exit.")
        while viewer.is_running():
            viewer.cam.lookat[:] = root_position(model, data)
            viewer.sync()
            time.sleep(0.05)
        viewer.close()
    return 2 if args.check and fell else 0


def main() -> int:
    args = parse_args()
    gait = resolve_gait(args)
    config = SpotConfig.load(CONFIG)
    if args.preset == "sim-trot":
        # The physical gait signs predate the unified rearward-knee URDF. Keep
        # hardware calibration untouched and use the URDF-consistent mapping
        # only inside this simulator-specific preset.
        config.gait_forward_signs = {leg: -1 for leg in LEGS}
    robot = SpotRobot(None, config)
    base = stance_targets(config, gait)

    print(
        f"MuJoCo {'dynamic' if args.dynamic else 'kinematic'} gait: "
        f"{gait.pattern}/{args.preset}, {args.cycles} cycles, "
        f"period={gait.period:g}s, rate={gait.control_rate:g}Hz, "
        f"stance=J1 {gait.stance_j1_angle:g}deg/"
        f"J2 {gait.stance_j2_angle:g}deg/J3 {gait.stance_j3_angle:g}deg, "
        f"balance={args.balance_mode if args.balance else 'off'}"
    )

    if args.dynamic:
        return run_dynamic(args, gait, config, robot, base)

    model = mujoco.MjModel.from_xml_path(str(URDF))
    data = mujoco.MjData(model)

    # Kinematic replay: hold the body in the air and show the exact commanded
    # joint trajectory without gravity or an actuator tracking model.
    model.opt.gravity[:] = 0.0
    apply_targets(model, data, config, base)

    if args.check:
        for frame in range(round(gait.control_rate * gait.period)):
            elapsed = frame / gait.control_rate
            targets = robot.gait_targets(
                elapsed / gait.period, base, gait, amplitude_scale=1.0
            )
            apply_targets(model, data, config, targets)
        print("valid: one complete spotctl gait cycle applied to 12 MuJoCo joints")
        return 0

    if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
        raise RuntimeError(
            "macOS live viewer requires mjpython; run: "
            ".venv/bin/mjpython simulation/mujoco/walk.py"
        )

    import mujoco.viewer as mj_viewer

    interval = 1.0 / gait.control_rate
    total_duration = args.cycles * gait.period
    started_at = time.monotonic()
    with mj_viewer.launch_passive(model, data) as viewer:
        frame = 0
        while viewer.is_running() and frame * interval < total_duration:
            elapsed = frame * interval
            targets = robot.gait_targets(
                (elapsed / gait.period) % 1.0,
                base,
                gait,
                amplitude_scale=amplitude_at(elapsed, total_duration),
            )
            apply_targets(model, data, config, targets)
            viewer.sync()
            frame += 1
            delay = started_at + frame * interval - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)

        apply_targets(model, data, config, base)
        viewer.sync()
        print("Gait complete; returned to the selected walking stance.")
        while viewer.is_running():
            time.sleep(0.05)
            viewer.sync()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
