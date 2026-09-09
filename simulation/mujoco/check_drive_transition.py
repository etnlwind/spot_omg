"""Replay left-turn to forward on the STEP rig; never connects to hardware.

This tests the shared trajectory with a 20 ms input slew, estimated torque
motors and contacts. It does not emulate the full STM32 scheduler, filtering,
watchdog, tilt-stop recovery or servo calibration. A pass is not a hardware pass.
"""
import argparse
import copy
import json
import time
from pathlib import Path

import mujoco
import numpy as np
from cad_physics import Simulation, build, KEYS


def legacy_targets(policy, phase, startup, linear, yaw):
    """Snapshot of V13 joint-angle addition, for before/after comparison."""
    scale = max(1, abs(linear) + abs(yaw))
    base, support = policy.trot4_targets(phase, 0)
    straight, _ = policy.trot4_direction_targets(phase, startup*abs(linear)/scale, -1 if linear < 0 else 1)
    turn, _ = policy.turn_targets(phase, startup*abs(yaw)/scale, 1 if yaw < 0 else -1)
    return {k: straight[k]+turn[k]-base[k] for k in KEYS}, support


def evaluate(p, model, switch_at, legacy=False, balance=False, replay=False, drive_stride=None):
    sim = Simulation(p, model)
    if legacy:
        policy = sim.policy
        policy.drive_targets = lambda *args: legacy_targets(policy, *args)
    viewer = None
    if replay:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        viewer.opt.geomgroup[3] = 0
        viewer.cam.distance = 1.1
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
    started = time.monotonic()
    rows = []
    for frame in range(round((switch_at+4)/.02)):
        t = frame*.02
        linear, yaw = (0, 0) if t < 2 else ((0, -1) if t < switch_at else (1, 0))
        sim.step(linear, yaw, balance=balance, limit_yaw=not legacy, stride_scale=None if legacy else drive_stride, startup=max(0, min(1, (t-2)/.7)))
        row = sim.row()
        rows.append(row)
        if viewer:
            if not viewer.is_running():
                viewer.close()
                raise RuntimeError("Replay closed before completion")
            viewer.cam.lookat[:] = sim.data.subtree_com[model.body('robot').id]
            viewer.sync()
            time.sleep(max(0, sim.data.time-(time.monotonic()-started)))
        if max(abs(row['roll_deg']), abs(row['pitch_deg'])) > 40 or any(
                name != 'floor' and not name.endswith('_foot') for name in row['contacts']):
            break
    if viewer:
        viewer.close()
    after = [r for r in rows if r['time_s'] >= switch_at]
    return dict(legacy=legacy, variant="V13" if legacy else "turn_limit_500", balance=balance, switch_at_s=switch_at,
                reached_transition=bool(after), fallen=len(rows)<round((switch_at+4)/.02),
                elapsed_s=rows[-1]['time_s'],
                peak_transition_tilt_deg=max((max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in after),default=None),
                final_roll_deg=rows[-1]['roll_deg'],
                peak_tracking_error_deg=max(r['max_tracking_error_deg'] for r in rows)), rows


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--output', type=Path, default=Path('/private/tmp/spot-drive-transition'))
    parser.add_argument('--balance', action='store_true', help='experimental simulator feedback; not the full firmware filter')
    parser.add_argument('--replay', action='store_true', help='show nominal left-turn to forward; use mjpython on macOS')
    parser.add_argument('--legacy', action='store_true', help='replay V13 without the turn governor')
    args = parser.parse_args()
    if args.legacy and not args.replay:
        parser.error('--legacy is only used with --replay')
    _, base = build(write_scene=False)
    args.output.mkdir(parents=True, exist_ok=True)
    if args.replay:
        xml, _ = build(base, write_scene=False)
        report, rows = evaluate(base, mujoco.MjModel.from_xml_string(xml), 4.9,
                                legacy=args.legacy, balance=args.balance, replay=True)
        (args.output/'replay.json').write_text(json.dumps(dict(report=report, frames=rows)))
        print(json.dumps(report, indent=2))
        return
    reports = []
    for scenario in ('nominal', 'slippery', 'offset_delay'):
        p = copy.deepcopy(base)
        if scenario == 'slippery': p['friction'][0] = .4
        if scenario == 'offset_delay':
            p['battery_center_cad_m'][0] += .025
            p['battery_center_cad_m'][1] -= .05
            p['command_delay_s'] = .04
        xml, _ = build(p, write_scene=False)
        model = mujoco.MjModel.from_xml_string(xml)
        for switch_at in (4., 4.45, 4.9, 5.35):
            for legacy in (True, False):
                report, rows = evaluate(p, model, switch_at, legacy, args.balance)
                report['scenario'] = scenario
                reports.append(report)
                print(json.dumps(report), flush=True)
                (args.output/f'{scenario}-{switch_at}-{legacy}.json').write_text(json.dumps(rows))
    (args.output/'summary.json').write_text(json.dumps(reports, indent=2)+'\n')


if __name__ == '__main__': main()
