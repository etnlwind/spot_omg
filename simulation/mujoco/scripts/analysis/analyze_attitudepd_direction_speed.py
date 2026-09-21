"""Compare unchanged V3 forward/reverse commands in estimated MuJoCo physics.

No hardware transport or profile tuning. Save positions as well as phase so
command cadence is not confused with measured body translation.
"""
import argparse
import csv
import ctypes
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.drive_controller import NAMES
from simulation.mujoco.runtime.gait_evidence import foot_loads
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.visualization.run_calculated_placement import parameters


def commanded_kinematics(robot, linear):
    """Report CAD stance travel separately from the planar IK input parameter."""
    fn = robot.plant.policy._library.spot_locomotion_targets
    fn.argtypes = (ctypes.c_int, *([ctypes.c_float]*4), ctypes.POINTER(ctypes.c_float))
    fn.restype = ctypes.c_int
    endpoints = []
    duty=float(robot.active_profile_params()[1])
    for phase in (0., duty-1.e-6):
        out = (ctypes.c_float*12)()
        if not fn(NAMES.index(robot.profile), phase, 1., linear/1000, 0., out):
            raise ValueError('Invalid nominal target')
        endpoints.append(robot.body_stabilizer.feet(out))
    # FL and RR have phase offset zero. Opposite pair shares the same straight trajectory.
    return dict(fl_stance_dx_mm=float((endpoints[1][0, 0]-endpoints[0][0, 0])*1000),
                rr_stance_dx_mm=float((endpoints[1][3, 0]-endpoints[0][3, 0])*1000),
                fl_stance_dz_mm=float((endpoints[1][0, 2]-endpoints[0][0, 2])*1000))


def run_case(linear, output, duration, profile='attitudepd_v3', foot_lift_mm=None, yaw=0, balance=False, heading=False):
    robot = RobotController(Simulation(parameters()))
    robot.select_profile(profile)
    robot.foot_lift_mm=list(foot_lift_mm or [0,0,0,0])
    robot.body_stabilizer.enabled = balance
    robot.heading.enabled = heading
    period = robot.plant.policy._library.spot_locomotion_period
    period.argtypes = (ctypes.c_int, ctypes.c_float, ctypes.c_float)
    period.restype = ctypes.c_float
    rows, events = [], []
    nominal = commanded_kinematics(robot, linear)
    model, data = robot.plant.model, robot.plant.data
    feet = [model.geom(leg+'_foot').id for leg in ('fl', 'fr', 'rl', 'rr')]
    drive_at = stop_at = fault = None
    robot.command('stand', 0.)
    try:
        for i in range(round((7 + duration + 6) / .02)):
            t = i * .02
            if drive_at is None and t >= 7 and robot.transition is None:
                robot.command(f'drive {linear} {yaw} 1', t)
                drive_at = t
            elif drive_at is not None and stop_at is None:
                if t - drive_at >= duration:
                    robot.command('@S 100000', t)
                    stop_at = t
                elif i % 10 == 0:
                    robot.command(f'@D {i + 2} {linear} {yaw}', t)
            robot.tick(t)
            state = robot.plant.row()
            w, x, y, z = robot.plant.data.qpos[3:7]
            heading = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
            row = dict(t=t, walk_s=None if drive_at is None else t-drive_at,
                       x_m=state['position_m'][0], y_m=state['position_m'][1],
                       z_m=state['position_m'][2], heading_deg=heading,
                       roll_deg=state['roll_deg'], pitch_deg=state['pitch_deg'],
                       phase=robot.phase, target_phase=getattr(robot, 'nominal_phase', robot.phase), linear=robot.linear,
                       period_s=float(period(NAMES.index(robot.profile), robot.linear, robot.yaw)),
                       tracking_enabled=robot.tracking_enabled,
                       tracking_error_deg=state['max_tracking_error_deg'],
                       voltage_v=state['voltage_v'], safety=robot.safety,
                       pose=robot.pose, moving=robot.motion is not None)
            for leg, foot, load, offset in zip(('fl', 'fr', 'rl', 'rr'), feet, foot_loads(model, data), (0., .5, .5, 0.)):
                row[f'{leg}_load_n'] = load
                row[f'{leg}_clearance_mm'] = foot_clearance(model, data, foot)*1000
                row[f'{leg}_phase'] = (row['target_phase']+offset) % 1
                row[f'{leg}_world_x_m'] = float(data.geom_xpos[foot, 0])
            slip_sum = load_sum = 0.
            floor = model.geom('floor').id
            for ci, contact in enumerate(data.contact):
                if floor not in (contact.geom1, contact.geom2):
                    continue
                foot = contact.geom2 if contact.geom1 == floor else contact.geom1
                if foot not in feet:
                    continue
                velocity, force = np.zeros(6), np.zeros(6)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_GEOM, foot, velocity, 0)
                mujoco.mj_contactForce(model, data, ci, force)
                contact_v = velocity[3:] + np.cross(velocity[:3], contact.pos-data.geom_xpos[foot])
                load = max(0., float(force[0]))
                slip_sum += load*float(np.linalg.norm(contact_v[:2]))
                load_sum += load
            row['contact_load_sum_n'] = load_sum
            row['contact_slip_load_sum_n_m_s'] = slip_sum
            for j, (command, actual) in enumerate(zip(robot.command_target, state['actual_deg'])):
                row[f'command_{j}_deg'] = float(command)
                row[f'actual_{j}_deg'] = actual
            rows.append(row)
            messages = robot.drain().decode(errors='replace')
            if messages:
                events.append(dict(t=t, messages=messages))
            if robot.safety != 'ok':
                fault = dict(t=t, reason=robot.safety)
                break
            if stop_at is not None and t-stop_at >= 3 and robot.motion is None and robot.transition is None:
                break
        name = ('forward' if linear > 0 else 'reverse') + f'_{abs(linear)}' + (f'_yaw_{yaw}' if yaw else '')
        with (output / f'{name}.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader()
            writer.writerows(rows)
        (output / f'{name}_events.json').write_text(json.dumps(events, indent=2)+'\n', encoding='utf-8')
        steady = [r for r in rows if r['walk_s'] is not None and 2 <= r['walk_s'] < duration]
        summary = dict(case=name, profile=profile, foot_lift_mm=robot.foot_lift_mm, linear_per_mille=linear, yaw_per_mille=yaw, balance_enabled=balance, heading_enabled=heading, drive_at=drive_at, stop_at=stop_at,
                       fault=fault, final_pose=robot.pose,
                       final_target_b_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
                       commanded_cad_kinematics=nominal,
                       steady_sample_count=len(steady),
                       complete=fault is None and stop_at is not None and robot.pose == 'stand')
        if len(steady) >= 2:
            times = np.array([r['t'] for r in steady])
            phase = np.unwrap([r['phase']*2*np.pi for r in steady])/(2*np.pi)
            phase_hz = float(np.polyfit(times, phase, 1)[0])
            speed = float(np.polyfit(times, [r['x_m'] for r in steady], 1)[0])
            headings = np.unwrap(np.radians([r['heading_deg'] for r in steady]))
            summary.update(
                steady_window_s=[steady[0]['walk_s'], steady[-1]['walk_s']],
                signed_world_x_speed_m_s=speed,
                speed_in_command_direction_m_s=speed*np.sign(linear),
                endpoint_world_x_speed_m_s=(steady[-1]['x_m']-steady[0]['x_m'])/(times[-1]-times[0]),
                lateral_speed_m_s=float(np.polyfit(times, [r['y_m'] for r in steady], 1)[0]),
                phase_hz=phase_hz, measured_phase_period_s=1/phase_hz,
                shared_c_period_s=steady[-1]['period_s'],
                max_abs_heading_deg=float(np.degrees(abs(headings)).max()),
                heading_change_deg=float(np.degrees(headings[-1]-headings[0])),
                max_abs_roll_deg=max(abs(r['roll_deg']) for r in steady),
                max_abs_pitch_deg=max(abs(r['pitch_deg']) for r in steady),
                mean_root_z_m=float(np.mean([r['z_m'] for r in steady])),
                max_tracking_error_deg=max(r['tracking_error_deg'] for r in steady),
                min_voltage_v=min(r['voltage_v'] for r in steady),
                load_weighted_contact_slip_m_s=sum(r['contact_slip_load_sum_n_m_s'] for r in steady)/max(1.e-9, sum(r['contact_load_sum_n'] for r in steady)),
                tracking_enabled=any(r['tracking_enabled'] for r in steady))
            summary['middle_swing'] = {}
            for leg in ('fl', 'fr', 'rl', 'rr'):
                # Middle 60% of the selected profile's swing.
                duty=float(robot.active_profile_params()[1])
                swing = [r for r in steady if duty+(1-duty)*.2 <= r[f'{leg}_phase'] <= duty+(1-duty)*.8]
                summary['middle_swing'][leg] = dict(samples=len(swing),
                    loaded_fraction=sum(r[f'{leg}_load_n'] > .5 for r in swing)/len(swing) if swing else None,
                    min_clearance_mm=min((r[f'{leg}_clearance_mm'] for r in swing), default=None))
        print(json.dumps(summary), flush=True)
        return summary, robot.plant.p
    finally:
        robot.body_stabilizer.close()


def run(output, duration):
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = ROOT / 'config/locomotion_profiles.json'
    profile = json.loads(manifest_path.read_text())['profiles']['attitudepd_v3']
    summaries = []
    for amplitude in (1000, 500):
        for sign in (1, -1):
            result, physics = run_case(sign*amplitude, output, duration)
            summaries.append(result)
    comparisons = []
    for forward, reverse in zip(summaries[::2], summaries[1::2]):
        comparable = (forward['complete'] and reverse['complete']
                      and forward.get('speed_in_command_direction_m_s', 0) > 0
                      and reverse.get('speed_in_command_direction_m_s', 0) > 0)
        comparisons.append(dict(input_magnitude=forward['linear_per_mille'],
            both_complete=forward['complete'] and reverse['complete'],
            reverse_forward_body_speed_ratio=reverse['speed_in_command_direction_m_s']/forward['speed_in_command_direction_m_s'] if comparable else None,
            reverse_forward_phase_rate_ratio=reverse['phase_hz']/forward['phase_hz'] if comparable else None))
    report = dict(profile='attitudepd_v3', source_commit=subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        physical_robot_test=False, backend='MuJoCo estimated physics',
        PD='off', heading_control='off', drive_duration_s=duration,
        measurement='50 Hz world X root position linear fit, walk +2s to before stop; world X is initial forward',
        deployed_forward_params=profile['params'], deployed_reverse_params=profile['turn_reverse_params'],
        physics_parameters=physics, cases=summaries, comparisons=comparisons)
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(comparisons), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=12.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 4:
        parser.error('--duration must be finite and at least 4 seconds')
    run(args.output, args.duration)
