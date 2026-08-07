#!/usr/bin/env python3
"""Run the STM32-shared repeating jump policy in MuJoCo."""

from __future__ import annotations

import argparse
import math
import os
import platform
import time

import mujoco

from servo import SharedGaitPolicy, SpotConfig
from walk import (
    CONFIG,
    LEGS,
    SCENE,
    body_up_z,
    canonical_targets_to_positions,
    ground_contact_legs,
    initialize_on_ground,
    read_imu,
    root_position,
    set_controls,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate the STM32-shared in-place/forward jump policy."
    )
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--period", type=float, default=1.2)
    parser.add_argument(
        "--forward-travel",
        type=float,
        default=0.0,
        help="normalized sagittal travel; STM32 in-place default is 0",
    )
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument(
        "--check", action="store_true", help="run headless and validate upright state"
    )
    args = parser.parse_args()
    if args.cycles < 1 or args.cycles > 20:
        parser.error("--cycles must be in 1..20")
    if args.period < 0.8 or args.period > 5.0:
        parser.error("--period must be in 0.8..5.0 seconds")
    if not -0.30 <= args.forward_travel <= 0.30:
        parser.error("--forward-travel must be in -0.30..0.30")
    if args.settle < 0.0:
        parser.error("--settle must be non-negative")
    return args


def main() -> int:
    args = parse_args()
    config = SpotConfig.load(CONFIG)
    policy = SharedGaitPolicy()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    stand = config.angles_to_targets(
        {
            (leg, joint): angle
            for leg in LEGS
            for joint, angle in ((1, 0.0), (2, 45.0), (3, 90.0))
        }
    )
    initialize_on_ground(model, data, config, stand)

    viewer = None
    if not args.check:
        if platform.system() == "Darwin" and "MJPYTHON_BIN" not in os.environ:
            raise RuntimeError("macOS live viewer requires mjpython")
        import mujoco.viewer as mj_viewer

        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 140.0
        viewer.cam.elevation = -20.0

    wall_origin = time.monotonic() - data.time
    next_view_time = data.time

    def physics_step() -> None:
        nonlocal next_view_time
        mujoco.mj_step(model, data)
        if viewer is not None and data.time >= next_view_time:
            viewer.cam.lookat[:] = root_position(model, data)
            viewer.sync()
            next_view_time += 1.0 / 60.0
            delay = wall_origin + data.time - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)

    settle_end = data.time + args.settle
    while data.time < settle_end and (viewer is None or viewer.is_running()):
        set_controls(model, data, config, stand)
        physics_step()

    start = root_position(model, data)
    started_at = data.time
    finish_at = started_at + args.cycles * args.period
    next_control = data.time
    min_z = math.inf
    max_z = -math.inf
    max_roll = 0.0
    max_pitch = 0.0
    airborne_steps = 0
    measured_steps = 0

    while data.time < finish_at and (viewer is None or viewer.is_running()):
        if data.time + 1e-12 >= next_control:
            phase = ((data.time - started_at) / args.period) % 1.0
            canonical, _ = policy.jump_targets(
                phase, args.forward_travel, config.gait_forward_signs
            )
            targets = canonical_targets_to_positions(config, canonical)
            set_controls(model, data, config, targets)
            next_control += 1.0 / 50.0
        physics_step()
        sample = read_imu(model, data)
        body_z = root_position(model, data)[2]
        min_z = min(min_z, body_z)
        max_z = max(max_z, body_z)
        max_roll = max(max_roll, abs(sample.roll))
        max_pitch = max(max_pitch, abs(sample.pitch))
        airborne_steps += not ground_contact_legs(model, data)
        measured_steps += 1

    recovery_end = data.time + 0.5
    while data.time < recovery_end and (viewer is None or viewer.is_running()):
        set_controls(model, data, config, stand)
        physics_step()

    finish = root_position(model, data)
    upright = body_up_z(model, data)
    fell = finish[2] < max(0.08, start[2] * 0.55) or upright < 0.5
    airborne_ratio = airborne_steps / measured_steps if measured_steps else 0.0
    print(
        "Jump result: "
        f"delta=({finish[0] - start[0]:+.3f}, {finish[1] - start[1]:+.3f})m, "
        f"body_z_range=({min_z:.3f}, {max_z:.3f})m, "
        f"airborne={airborne_ratio:.1%}, "
        f"max_roll={math.degrees(max_roll):.2f}deg, "
        f"max_pitch={math.degrees(max_pitch):.2f}deg, "
        f"up_z={upright:.3f}, state={'FALLEN' if fell else 'UPRIGHT'}"
    )

    if viewer is not None:
        print("Jump complete; close the MuJoCo window to exit.")
        while viewer.is_running():
            viewer.cam.lookat[:] = root_position(model, data)
            viewer.sync()
            time.sleep(0.05)
        viewer.close()
    return 2 if args.check and fell else 0


if __name__ == "__main__":
    raise SystemExit(main())
