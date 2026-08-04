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

from servo import SpotConfig, SpotRobot
from servo.cli import PRESETS


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "hardware" / "urdf" / "spot_omg.urdf"
SCENE = Path(__file__).resolve().parent / "spot_omg_scene.xml"
CONFIG = ROOT / "tools" / "servo_tool" / "config" / "joints.json"
LEGS = ("FL", "FR", "RL", "RR")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gait", choices=("trot", "crawl"), default="trot")
    parser.add_argument("--preset", choices=tuple(PRESETS), default="test")
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
        "--check",
        action="store_true",
        help="validate one cycle without opening the viewer",
    )
    return parser.parse_args()


def resolve_gait(args: argparse.Namespace):
    """Apply the same preset-overrides convention as spotctl walk."""
    gait = PRESETS[args.preset]
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
    if not 1 <= args.cycles <= 1000:
        raise ValueError("cycles must be between 1 and 1000")
    if not 0.0 <= args.settle <= 10.0:
        raise ValueError("settle must be between 0 and 10 seconds")
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


def ground_contact_count(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    feet = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot_geom")
        for leg in ("fl", "fr", "rl", "rr")
    }
    return sum(
        (contact.geom1 == ground and contact.geom2 in feet)
        or (contact.geom2 == ground and contact.geom1 in feet)
        for contact in data.contact[: data.ncon]
    )


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

    def step_until(end_time: float, controls) -> int:
        nonlocal next_view_time
        contact_frames = 0
        next_control_time = data.time
        while data.time < end_time and (viewer is None or viewer.is_running()):
            if data.time + 1e-12 >= next_control_time:
                set_controls(model, data, config, controls(data.time))
                next_control_time += 1.0 / gait.control_rate
            mujoco.mj_step(model, data)
            contact_frames += int(ground_contact_count(model, data) > 0)
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
    contact_frames = step_until(settle_end, lambda _: base)
    start = root_position(model, data)

    gait_started = data.time
    total_duration = args.cycles * gait.period

    def gait_controls(sim_time: float) -> dict[int, int]:
        elapsed = min(total_duration, sim_time - gait_started)
        return robot.gait_targets(
            (elapsed / gait.period) % 1.0,
            base,
            gait,
            amplitude_scale=amplitude_at(elapsed, total_duration),
        )

    contact_frames += step_until(gait_started + total_duration, gait_controls)
    contact_frames += step_until(data.time + 0.5, lambda _: base)
    finish = root_position(model, data)
    upright = body_up_z(model, data)
    fell = finish[2] < max(0.08, start[2] * 0.55) or upright < 0.5

    if not all(math.isfinite(value) for value in (*data.qpos, *data.qvel)):
        raise RuntimeError("dynamic simulation produced a non-finite state")
    print(
        "Dynamic result: "
        f"delta=({finish[0] - start[0]:+.3f}, "
        f"{finish[1] - start[1]:+.3f})m, "
        f"body_z={finish[2]:.3f}m, up_z={upright:.3f}, "
        f"contact_frames={contact_frames}, "
        f"state={'FALLEN' if fell else 'UPRIGHT'}"
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
    robot = SpotRobot(None, config)
    base = stance_targets(config, gait)

    print(
        f"MuJoCo {'dynamic' if args.dynamic else 'kinematic'} gait: "
        f"{gait.pattern}/{args.preset}, {args.cycles} cycles, "
        f"period={gait.period:g}s, rate={gait.control_rate:g}Hz, "
        f"stance=J1 {gait.stance_j1_angle:g}deg/"
        f"J2 {gait.stance_j2_angle:g}deg/J3 {gait.stance_j3_angle:g}deg"
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
