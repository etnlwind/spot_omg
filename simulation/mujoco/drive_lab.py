#!/usr/bin/env python3
"""Dynamic drive-policy experiment; no robot connection or hardware commands."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
from servo import SharedGaitPolicy, SpotConfig
from walk import (CONFIG, SCENE, LEGS, initialize_on_ground, root_position,
                  body_up_z, read_imu, ground_contact_legs)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='run without a window')
    parser.add_argument('--duration', type=float, default=12)
    parser.add_argument('--linear', type=float, default=0.6, help='-1..1')
    parser.add_argument('--yaw', type=float, default=0, help='-1..1, positive right')
    parser.add_argument('--balance', action='store_true', help='shared C IMU correction')
    parser.add_argument('--torque-scale', type=float, default=1,
                        help='scale the existing estimated actuator force limits')
    parser.add_argument('--output', type=Path, default=Path('/private/tmp/spot-drive-lab'))
    args = parser.parse_args()
    for name in ('duration', 'linear', 'yaw', 'torque_scale'):
        if not math.isfinite(getattr(args, name)):
            parser.error(f'{name} must be finite')
    if args.duration <= 0 or not 0 < args.torque_scale <= 3:
        parser.error('duration must be positive; torque-scale must be in (0, 3]')
    if abs(args.linear) > 1 or abs(args.yaw) > 1:
        parser.error('linear/yaw must be within -1..1')
    return args


def main():
    args = arguments()
    if not args.check and platform.system() == 'Darwin' and 'MJPYTHON_BIN' not in os.environ:
        raise RuntimeError('Use mjpython simulation/mujoco/drive_lab.py on macOS')
    policy = SharedGaitPolicy()
    config = SpotConfig.load(CONFIG)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.actuator_forcerange[:] *= args.torque_scale
    model.jnt_actfrcrange[:] *= args.torque_scale
    data = mujoco.MjData(model)
    keys = [(leg, joint) for leg in LEGS for joint in (1, 2, 3)]
    joint_ids = [model.joint(f'{leg.lower()}_j{joint}').id for leg, joint in keys]
    qpos = model.jnt_qposadr[joint_ids]
    actuators = [model.actuator(f'{leg.lower()}_j{joint}_position').id for leg, joint in keys]
    base, _ = policy.drive_targets(0, 0, 0, 0)
    initialize_on_ground(model, data, config, config.angles_to_targets(base))
    # Settle before measurement; the initial body drop is not forward travel.
    for _ in range(round(1 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = np.array(root_position(model, data))
    start_time = data.time
    requested = [args.linear, args.yaw]
    lock = threading.Lock()

    def keypress(key):
        with lock:
            mapping = {87: (1, 0), 83: (-1, 0), 65: (0, -1),
                       68: (0, 1), 32: (0, 0)}
            if key in mapping:
                requested[:] = mapping[key]

    viewer = None
    if not args.check:
        import mujoco.viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data, key_callback=keypress)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.3, 135, -22
        print('W forward | S backward | A/D turn | Space neutral | close window to exit', flush=True)

    frames = []
    linear = yaw = phase = 0.0
    wall_start = time.monotonic()
    fallen = False
    try:
        while data.time - start_time < args.duration:
            if viewer and not viewer.is_running():
                break
            elapsed = data.time - start_time
            with lock:
                target_linear, target_yaw = requested
            # Mirrors the firmware's 40/1000 input slew per 20 ms and
            # 2400..1800 ms period, not its entire scheduler/safety firmware.
            linear += max(-0.04, min(0.04, target_linear - linear))
            yaw += max(-0.04, min(0.04, target_yaw - yaw))
            period = 2.4 - 0.6 * min(1, abs(linear) + abs(yaw))
            targets, support = policy.drive_targets(
                phase, policy.smootherstep(min(1, elapsed / 0.7)), linear, yaw)
            if args.balance:
                targets = policy.balance_targets(
                    targets, sample=read_imu(model, data), support_legs=support,
                    kp=1, kd=0.04, leg_length_limit=0.08, mode='contact-aware',
                    j1_gain=15, j1_limit=5, foot_placement_gain=0,
                    foot_placement_limit=0)
            desired = np.radians([targets[key] for key in keys])
            data.ctrl[actuators] = desired
            for _ in range(round(0.02 / model.opt.timestep)):
                mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('Non-finite physics state')
            phase = (phase + 0.02 / period) % 1
            position = root_position(model, data)
            imu = read_imu(model, data)
            error = np.degrees(desired - data.qpos[qpos])
            force = data.actuator_force[actuators]
            limits = model.actuator_forcerange[actuators, 1]
            contact = ground_contact_legs(model, data)
            row = dict(time_s=float(data.time - start_time), x_m=position[0],
                       y_m=position[1], z_m=position[2], linear=linear, yaw=yaw,
                       period_s=period, roll_deg=math.degrees(imu.roll),
                       pitch_deg=math.degrees(imu.pitch),
                       max_error_deg=float(abs(error).max()),
                       torque_limit_fraction=float(np.mean(abs(force) >= limits * 0.95)),
                       contact_legs=' '.join(sorted(contact)))
            for i, (leg, joint) in enumerate(keys):
                row[f'{leg}_J{joint}_target_deg'] = targets[(leg, joint)]
                row[f'{leg}_J{joint}_actual_deg'] = float(np.degrees(data.qpos[qpos[i]]))
                row[f'{leg}_J{joint}_torque_nm'] = float(force[i])
            frames.append(row)
            fallen = body_up_z(model, data) < 0.5 or position[2] < max(0.08, start[2] * 0.55)
            if viewer:
                viewer.cam.lookat[:] = position
                viewer.sync()
                time.sleep(max(0, data.time - start_time - (time.monotonic() - wall_start)))
            if fallen:
                break
        finish = np.array(root_position(model, data))
        args.output.mkdir(parents=True, exist_ok=True)
        if frames:
            with (args.output / 'frames.csv').open('w') as file:
                writer = csv.DictWriter(file, fieldnames=list(frames[0]))
                writer.writeheader()
                writer.writerows(frames)
        report = dict(
            state='FALLEN' if fallen else 'UPRIGHT', samples=len(frames),
            elapsed_s=float(data.time - start_time),
            displacement_m=(finish - start).tolist(),
            mean_forward_m_s=float((finish[0] - start[0]) / max(data.time - start_time, 1e-9)),
            peak_tracking_error_deg=max((r['max_error_deg'] for r in frames), default=None),
            mean_torque_limit_fraction=float(np.mean([r['torque_limit_fraction'] for r in frames])) if frames else None,
            settings=dict(linear=args.linear, yaw=args.yaw, balance=args.balance, torque_scale=args.torque_scale),
            limitations=['Shared C drive trajectory, not full STM32 execution.',
                         'No firmware actuator limiter, voltage sag, UART delay or watchdog model.',
                         'Legacy scene: uniform STS3215 torque; real J2 is STS3250.',
                         'Battery mass/COM and friction not calibrated; directions require physical validation.'])
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        if viewer:
            print('Simulation paused. Close the window to exit.', flush=True)
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)
        return 0
    finally:
        if viewer:
            viewer.close()


if __name__ == '__main__':
    raise SystemExit(main())
