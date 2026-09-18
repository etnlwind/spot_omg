"""V4 joystick endpoints, live mode transitions, and previous-profile preservation.

Estimated physics only; never opens a physical robot transport.
"""
import argparse
import ctypes as C
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

import numpy as np
from servo import SharedGaitPolicy
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.analysis.analyze_attitudepd_direction_speed import run_case
from simulation.mujoco.scripts.visualization.run_calculated_placement import parameters


def previous_profiles(reference):
    saved = np.load(reference)
    policy = SharedGaitPolicy()
    target = policy._library.spot_locomotion_targets
    target.argtypes = (C.c_int, *([C.c_float]*4), C.POINTER(C.c_float))
    target.restype = C.c_int
    period = policy._library.spot_locomotion_period
    period.argtypes = (C.c_int, C.c_float, C.c_float)
    period.restype = C.c_float
    values, periods = [], []
    for profile, phase, scale, linear, yaw in saved['cases']:
        output = (C.c_float*12)()
        ok = target(int(profile), phase, scale, linear, yaw, output)
        values.append([ok, *output])
        periods.append(period(int(profile), linear, yaw))
    np.testing.assert_array_equal(values, saved['values'])
    np.testing.assert_array_equal(periods, saved['periods'])
    return dict(cases=len(values), target_and_period_exact_match=True,
                profiles=['centerpivot', 'attitudepd', 'attitudepd_v2', 'attitudepd_v3'])


def transitions(output):
    robot = RobotController(Simulation(parameters()))
    assert robot.profile == 'attitudepd_v4'
    robot.body_stabilizer.enabled = False
    robot.heading.enabled = False
    schedule = [(7., 588), (10., 1000), (13., 588), (16., -588), (19., -1000), (22., 0)]
    rows, command_events = [], []
    index, requested = 0, 0
    fault = None
    robot.command('stand', 0.)
    try:
        for i in range(1301):
            t = i*.02
            if index < len(schedule) and t >= schedule[index][0]:
                requested = schedule[index][1]
                cmd = f'drive {requested} 0 1' if index == 0 else f'@D {i+2} {requested} 0'
                if requested == 0:
                    cmd = f'@S {i+2}'
                robot.command(cmd, t)
                command_events.append(dict(t=t, command=cmd))
                index += 1
            elif requested and i % 10 == 0:
                robot.command(f'@D {i+2} {requested} 0', t)
            robot.tick(t)
            state = robot.plant.row()
            rows.append(dict(t=t, requested=requested, linear=robot.linear, phase=robot.phase,
                params=robot.active_profile_params().tolist(), position_m=state['position_m'],
                roll_deg=state['roll_deg'], pitch_deg=state['pitch_deg'],
                command=robot.command_target.tolist(), actual=state['actual_deg'],
                safety=robot.safety, pose=robot.pose, moving=robot.motion is not None,
                transition=robot.transition is not None, messages=robot.drain().decode(errors='replace')))
            if robot.safety != 'ok':
                fault = dict(t=t, reason=robot.safety)
                break
        summary = dict(case='half_full_half_reverse_stop', fault=fault,
            command_events=command_events, final_pose=robot.pose,
            final_target_b_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
            max_abs_roll_deg=max(abs(r['roll_deg']) for r in rows),
            max_abs_pitch_deg=max(abs(r['pitch_deg']) for r in rows),
            max_command_step_deg=float(np.max(abs(np.diff([r['command'] for r in rows if 8 <= r['t'] < 22], axis=0)))),
            complete=fault is None and index == len(schedule) and robot.pose == 'stand' and robot.motion is None)
        (output/'transitions.json').write_text(json.dumps(dict(summary=summary, rows=rows), indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        return summary
    finally:
        robot.body_stabilizer.close()


def main(output, reference):
    output.mkdir(parents=True, exist_ok=True)
    preserved = previous_profiles(reference)
    print(json.dumps(preserved), flush=True)
    cases = []
    for linear in (588, -588, 1000, -1000):
        summary, physics = run_case(linear, output, 12., profile='attitudepd_v4')
        cases.append(summary)
    transition = transitions(output)
    report = dict(profile='attitudepd_v4', physical_robot_test=False,
        PD='off', heading_control='off', physics_parameters=physics,
        previous_profiles=preserved, cases=cases, transition=transition,
        command_flow_pass=all(c['complete'] for c in cases) and transition['complete'])
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    if not report['command_flow_pass']:
        raise SystemExit('V4 estimated-physics command flow failed; inspect summary.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/attitudepd-v4/simulation')
    parser.add_argument('--reference', type=Path, default=ROOT/'artifacts/attitudepd-v4/previous-profile-reference.npz')
    args = parser.parse_args()
    main(args.output, args.reference)
