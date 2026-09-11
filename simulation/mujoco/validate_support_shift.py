"""Fixed large-step acceptance benchmark. Simulator truth is evaluation only.

Run from any directory with the MuJoCo Python environment. No robot transport.
Both policies use identical start, drive packets, physics, and measurement times.
"""
import copy
import csv
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from diagnose_turn_clearance import run

ROOT = Path(__file__).resolve().parent
OUT = ROOT.parents[1] / 'artifacts/upright/2026-09-11/cushion'
SOURCE_FILES = ['support_shift.py', 'virtual_robot.py', 'cad_physics.py',
                'bno055_emulator.py', 'position_wbc.py', 'cad_300mm/physics_parameters.json']


def fingerprints():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES}


class Recorder:
    def __init__(self):
        self.frames = []
        self.previous = None

    def __call__(self, now, robot):
        plant = robot.plant
        state = plant.row()
        target = np.asarray(robot.command_target)
        target_speed = np.zeros(12) if self.previous is None else (target-self.previous)/.02
        self.previous = target.copy()
        friction = []
        for i, contact in enumerate(plant.data.contact):
            if plant.model.geom('floor').id not in (contact.geom1, contact.geom2):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(plant.model, plant.data, i, force)
            if force[0] > .2:
                friction.append(float(np.linalg.norm(force[1:3])/(contact.friction[0]*force[0])))
        shift = copy.deepcopy(robot.support_shift.diagnostic) if robot.support_shift else {}
        self.frames.append(dict(time_s=now, safety=robot.safety, moving=robot.motion is not None,
            transition=robot.transition is not None, roll_deg=state['roll_deg'], pitch_deg=state['pitch_deg'],
            command_deg=target.tolist(), actual_deg=state['actual_deg'],
            command_speed_deg_s=target_speed.tolist(), actual_speed_deg_s=np.degrees(plant.data.qvel[plant.v]).tolist(),
            motor_speed_deg_s=np.degrees(plant.speed).tolist(), torque_nm=state['torque_nm'],
            torque_limits_nm=plant.limits.tolist(), above_rated_fraction=state['above_rated_fraction'],
            saturated_fraction=state['torque_limit_fraction'], friction_utilization=max(friction, default=0.),
            position_m=state['position_m'], support_shift=shift))


def metrics(frames, rows):
    # 60 full seconds after the common startup, including stopped/fault frames.
    frames = [f for f in frames if f['time_s'] >= 5]
    angles = np.array([[f['roll_deg'], f['pitch_deg']] for f in frames])
    commands = np.array([f['command_deg'] for f in frames])
    actual = np.array([f['actual_deg'] for f in frames])
    speeds = np.abs([f['command_speed_deg_s'] for f in frames])
    motor_speed = np.array([f['motor_speed_deg_s'] for f in frames])
    shift = [f['support_shift'] for f in frames if f['support_shift']]
    stance = [r['contact_center_speed_m_s'] for r in rows if r['moving'] and not r['swing'] and r['force_n'] > .2]
    return dict(max_roll_deg=float(abs(angles[:, 0]).max()), max_pitch_deg=float(abs(angles[:, 1]).max()),
        rms_roll_deg=float(np.sqrt(np.mean(angles[:, 0]**2))), rms_pitch_deg=float(np.sqrt(np.mean(angles[:, 1]**2))),
        tracking_rms_deg=float(np.sqrt(np.mean((commands-actual)**2))),
        tracking_peak_deg=float(abs(commands-actual).max()),
        stance_center_speed_proxy_m_s=float(np.mean(stance)) if stance else None,
        # Includes rolling of the deformable cap; this is NOT a pure slip sensor.
        max_command_speed_deg_s=float(speeds.max()), max_command_motor_speed_ratio=float((speeds/motor_speed).max()),
        joint_ranges_ok=bool(np.all(commands >= np.tile([-30, -45, 0], 4)) and np.all(commands <= np.tile([30, 100, 150], 4))),
        max_torque_nm=float(np.max(np.abs([f['torque_nm'] for f in frames]))),
        max_torque_envelope_ratio=float(max(np.max(np.abs(f['torque_nm'])/np.maximum(.001, f['torque_limits_nm'])) for f in frames)),
        mean_motor_saturated_fraction=float(np.mean([f['saturated_fraction'] for f in frames])),
        mean_above_rated_fraction=float(np.mean([f['above_rated_fraction'] for f in frames])),
        max_friction_utilization=max(f['friction_utilization'] for f in frames),
        max_planned_ik_residual_m=max((f.get('planned_residual_m', 0.) for f in shift), default=0.),
        max_stance_ik_residual_m=max((f.get('stance_residual_m', 0.) for f in shift), default=0.),
        max_reference_body_shift_m=max((max(abs(np.array(f.get('body_shift_m', [0, 0, 0])))) for f in shift), default=0.),
        safety_faults=sorted(set(f['safety'] for f in frames if f['safety'] != 'ok')),
        moving_fraction=float(np.mean([f['moving'] for f in frames])))


def gates(result, baseline, expect_stop=False):
    m, b = result['metrics'], baseline['metrics']
    legs = result['legs']
    return dict(
        uninterrupted=m['safety_faults'] == [] and (expect_stop or m['moving_fraction'] == 1.),
        tilt_max=m['max_roll_deg'] <= 3 and m['max_pitch_deg'] <= 3,
        tilt_rms=m['rms_roll_deg'] <= 1.5 and m['rms_pitch_deg'] <= 1.5,
        large_step=all(legs[l]['touchdown_forward_median_mm'] is not None and legs[l]['touchdown_forward_median_mm'] >= 30 for l in ('FL', 'FR')),
        clearance=all(v['peak_clearance_mm'] is not None and v['peak_clearance_mm'] >= 15 for v in legs.values()),
        middle_swing_contact=all(v['middle_swing_contact_fraction'] is not None and v['middle_swing_contact_fraction'] < .1 for v in legs.values()),
        stance_motion_not_worse=m['stance_center_speed_proxy_m_s'] is not None and b['stance_center_speed_proxy_m_s'] is not None and m['stance_center_speed_proxy_m_s'] <= b['stance_center_speed_proxy_m_s'],
        tracking_not_worse=m['tracking_rms_deg'] <= b['tracking_rms_deg'] and m['tracking_peak_deg'] <= b['tracking_peak_deg'],
        geometry=m['joint_ranges_ok'] and m['max_planned_ik_residual_m'] <= .001 and m['max_reference_body_shift_m'] <= .01000001,
        speed_feasible=m['max_command_motor_speed_ratio'] <= 1.,
        torque_feasible=m['mean_motor_saturated_fraction'] < .01 and m['mean_above_rated_fraction'] < .01,
        friction_feasible=m['max_friction_utilization'] <= 1.01,
        stop_completed=(result.get('stop_completed', True) if expect_stop else True))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    profiles = json.loads((ROOT/'upright_profiles.json').read_text())['profiles']
    pad = json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    soft = dict(pad, contact_time_constant_s=.04, damping_ratio=.8, friction=[.6, .005, .0001])
    cases = [('forward', 'nominal', 65.02, pad, {}, None),
             ('com_offset', 'com_offset', 30, pad, {}, None),
             ('servo_delay', 'nominal', 30, pad, {'command_delay_s': .08}, None),
             ('motor_loss', 'motor_loss', 30, pad, {}, None),
             ('cushion_soft_slippery', 'nominal', 30, soft, {}, None),
             ('imu_delay', 'nominal', 30, pad, {'bno055': {'fusion_delay_s': .06}}, None),
             ('forward_stop', 'nominal', 24, pad, {}, 18.)]
    results, baselines = [], []
    for case, scenario, seconds, cushion, overrides, stop in cases:
        pair = []
        for name in ('cushion_forward', 'cushion_support_shift'):
            recorder = Recorder()
            result, rows = run(1000, 0, seconds, profile=name, override=profiles[name], scenario=scenario,
                cushion=cushion, parameter_overrides=overrides, observer=recorder, stop_at=stop)
            result.update(case=case, duration_s=seconds, measurement_start_s=5.,
                metrics=metrics(recorder.frames, rows), parameter_overrides=overrides,
                controller_sha256=fingerprints(), stop_completed=not recorder.frames[-1]['moving'] and not recorder.frames[-1]['transition'])
            stem = f'support-shift-{case}-{name}'
            with (OUT/(stem+'.csv')).open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
            if case == 'forward':
                window = [f for f in recorder.frames if 6.6-1e-6 <= f['time_s'] <= 7.4+1e-6]
                (OUT/(stem+'-window.json')).write_text(json.dumps(window, indent=2, default=lambda x: x.item()))
                result['reference_7s'] = dict(frame=min(window, key=lambda f: abs(f['time_s']-7)),
                    feet=[r for r in rows if abs(r['time_s']-7) < .001])
            pair.append(result)
        pair[1]['gates'] = gates(pair[1], pair[0], stop is not None)
        pair[1]['quality_passed'] = all(pair[1]['gates'].values())
        baselines.append(pair[0]); results.append(pair[1])
        (OUT/'support-shift-baseline.json').write_text(json.dumps(baselines, indent=2, default=lambda x: x.item()))
        (OUT/'support-shift-validation.json').write_text(json.dumps(results, indent=2, default=lambda x: x.item()))
        print(case, 'PASS' if pair[1]['quality_passed'] else 'FAIL',
              {k: round(pair[1]['metrics'][k], 3) for k in ('max_roll_deg', 'max_pitch_deg', 'tracking_rms_deg')},
              'failed:', [k for k,v in pair[1]['gates'].items() if not v], flush=True)
    # UI promotion requires every declared condition, not just the nominal run.
    for r in results:
        r['suite_passed'] = all(x['quality_passed'] for x in results)
    (OUT/'support-shift-validation.json').write_text(json.dumps(results, indent=2, default=lambda x: x.item()))


if __name__ == '__main__':
    main()
