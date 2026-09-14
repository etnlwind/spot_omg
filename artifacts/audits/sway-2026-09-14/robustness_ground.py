"""Independent stop/uncertainty evaluation; never imports evaluator results.

Example: python robustness_ground.py --config feedforward-iteration3.json
  --scenario nominal --output robustness-iteration3-nominal.json
No hardware/app interfaces are opened. Plant truth is recorded for evaluation.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'simulation/mujoco'))
from cad_physics import Simulation, foot_clearance
from virtual_robot import RobotController
from ground_frame_preview import configure
from support_shift import SupportShift


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(rows, duty):
    if not rows:
        return None
    values = lambda key: np.array([r[key] for r in rows])
    rp = values('roll_pitch_deg')
    phase = (values('phase')[:, None] + [0., .5, .5, 0.]) % 1.
    u = (phase - duty) / (1 - duty)
    mid = (u > 1/6) & (u < 5/6)
    force = values('foot_normal_force_n')
    clearance = values('foot_clearance_mm')
    positions = values('com_m')
    result = dict(samples=len(rows), time_range_s=[rows[0]['time_s'], rows[-1]['time_s']],
        roll_pitch_rms_deg=np.sqrt(np.mean(rp * rp, axis=0)).tolist(),
        roll_pitch_abs_max_deg=abs(rp).max(axis=0).tolist(),
        forward_speed_m_s=float((positions[-1, 0] - positions[0, 0]) / max(.02, rows[-1]['time_s'] - rows[0]['time_s'])),
        com_lateral_range_mm=float(np.ptp(positions[:, 1]) * 1000),
        com_height_range_mm=float(np.ptp(positions[:, 2]) * 1000),
        joint_tracking_rms_deg=np.sqrt(np.mean((values('command_deg') - values('actual_deg'))**2, axis=0)).reshape(4, 3).tolist(),
        mid_swing_contact_fraction=[float(np.mean(force[mid[:, i], i] > 1)) if mid[:, i].any() else None for i in range(4)],
        mid_swing_peak_clearance_mm=[float(clearance[mid[:, i], i].max()) if mid[:, i].any() else None for i in range(4)],
        maximum_saturation_fraction=float(values('saturation_fraction').max()),
        minimum_voltage_v=float(values('voltage_v').min()))
    for stage in ('nominal', 'command', 'actual'):
        feet = values(stage + '_body_feet_m')
        result[stage + '_body_foot_range_mm'] = (np.ptp(feet, axis=0) * 1000).tolist()
        relative = feet - values('foot_goals_m')
        pair_delta = np.stack([relative[:, 2] - relative[:, 1], relative[:, 3] - relative[:, 0]], axis=1)
        result[stage + '_same_phase_pair_difference_rms_mm'] = (np.sqrt(np.mean(pair_delta**2, axis=0)) * 1000).tolist()
    return result


def run(config_path, scenario, idle_s=10., forward_s=20., stop_s=3.):
    config_path = Path(config_path).resolve()
    loaded = json.loads(config_path.read_text())
    config = loaded.get('config', loaded)
    parameter_path = ROOT / 'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json'
    p = json.loads(parameter_path.read_text())
    p.update(timestep_s=.0005, experimental_stow=True)
    p['foot_cushion'] = json.loads((ROOT / 'simulation/mujoco/foot_cushion_10mm.json').read_text())
    changes = {}
    if scenario in ('delay60', 'combined'):
        p['command_delay_s'] = .06
        changes['command_delay_s'] = .06
    if scenario in ('motor80', 'combined'):
        for motor in ('motor_3215', 'motor_3250'):
            for key in ('stall_nm', 'rated_nm', 'speed_rad_s'):
                p[motor][key] *= .8
        changes['motor_torque_and_no_load_speed_factor'] = .8
    plant = Simulation(p); robot = RobotController(plant); robot.select_profile('centerpivot')
    info = configure(robot, config)
    m, d = plant.model, plant.data
    kin = SupportShift(m)
    feet = [m.geom(x + '_foot').id for x in ('fl', 'fr', 'rl', 'rr')]
    floor = m.geom('floor').id
    rows = []; outputs = []; fault = None; exception = None
    stop_at = idle_s + forward_s
    stopped_at = None
    for tick in range(round((stop_at + stop_s) / .02)):
        now = tick * .02
        if tick == round(idle_s / .02):
            robot.command('drive 1000 0 1', now)
        if idle_s < now < stop_at and tick % 10 == 0:
            robot.command(f'@D {tick} 1000 0', now)
        if tick == round(stop_at / .02):
            robot.command(f'@S {tick + 10000}', now)
        try:
            robot.tick(now)
        except Exception as error:
            exception = dict(time_s=now, type=type(error).__name__, message=str(error))
            break
        responses = robot.drain()
        if responses:
            outputs.append(dict(time_s=now, messages=responses.decode('utf-8', errors='replace')))
        if now >= stop_at and not robot.motion and not robot.transition and stopped_at is None:
            stopped_at = now
        force = np.zeros(4)
        for index, contact in enumerate(d.contact):
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            if other in feet:
                wrench = np.zeros(6); mujoco.mj_contactForce(m, d, index, wrench)
                force[feet.index(other)] += max(0., wrench[0])
        measured = np.degrees(d.qpos[plant.q])
        state = plant.row()
        record = dict(time_s=now, stage='idle' if now < idle_s else 'forward' if now < stop_at else 'stop',
            phase=float(getattr(robot, 'ground_evaluated_phase', robot.phase)),
            roll_pitch_deg=[state['roll_deg'], state['pitch_deg']], com_m=state['com_m'],
            foot_normal_force_n=force.tolist(), foot_clearance_mm=[foot_clearance(m, d, f) * 1000 for f in feet],
            foot_world_center_m=[d.geom_xpos[f].tolist() for f in feet],
            nominal_deg=robot.target.tolist(), command_deg=robot.command_target.tolist(), actual_deg=measured.tolist(),
            quantized_command_deg=np.degrees(plant.desired).tolist(), voltage_v=state['voltage_v'],
            saturation_fraction=plant.saturated, safety=robot.safety, torque=robot.torque,
            motion=bool(robot.motion), transition=bool(robot.transition), pose=robot.pose,
            sensor_roll_pitch_tenths=list(robot.attitude_filter.filtered),
            sensor_rate_tenths_s=list(robot.attitude_filter.rate),
            foot_goals_m=info['foot_goals_m'], roll_feedforward_rad=float(info.get('roll_feedforward_rad', 0.)))
        for label, angles in (('nominal', robot.target), ('command', robot.command_target), ('actual', measured)):
            kin.set_angles(angles)
            record[label + '_body_feet_m'] = [kin.foot(i).tolist() for i in range(4)]
        rows.append(record)
        if robot.safety != 'ok':
            fault = dict(time_s=now, reason=robot.safety)
            break
    duty = float(config.get('duty', .52))
    forward = [r for r in rows if r['stage'] == 'forward']
    steady_after = idle_s + float(config.get('startup_s', 1.))
    stopping = [r for r in rows if r['stage'] == 'stop']
    summary = dict(scenario=scenario, synthetic_parameter_changes=changes,
        mass_kg=float(m.body_mass.sum()), fault=fault, exception=exception,
        stop_requested_s=stop_at, stopped_at_s=stopped_at,
        stop_completed=stopped_at is not None, final_pose=robot.pose, final_torque=robot.torque,
        full_forward=summarize(forward, duty),
        steady_forward=summarize([r for r in forward if r['time_s'] >= steady_after], duty),
        stop_interval=summarize(stopping, duty))
    if stopping:
        commands = np.array([r['command_deg'] for r in rows])
        times = np.array([r['time_s'] for r in rows])
        selected = (times[1:] >= stop_at - .04)
        summary['maximum_stop_command_step_deg'] = float(abs(np.diff(commands, axis=0))[selected].max())
        summary['final_command_vs_aligned_neutral_deg'] = (np.array(rows[-1]['command_deg']) - np.array(info['neutral_deg']).reshape(12)).tolist()
    sources = ['ground_frame_nominal.py', 'ground_frame_preview.py', 'ground_foothold_transfer.py',
               'virtual_robot.py', 'cad_physics.py']
    return dict(summary=summary, config=config, config_source=str(config_path), config_sha256=fingerprint(config_path),
        source_sha256={name:fingerprint(ROOT / 'simulation/mujoco' / name) for name in sources},
        geometry=info, output=outputs, records=rows,
        notes=['Body-foot XY is cushion geometry center; Z is lowest cushion mesh vertex, not contact-pressure center.',
               'COM and actual contacts are simulator truth for evaluation only.',
               'Motor80 is a synthetic sensitivity condition, not a measured degradation calibration.',
               'Stop interval phase metrics are recorded but not interpreted as a continuing gait.'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--scenario', choices=('nominal', 'delay60', 'motor80', 'combined'), default='nominal')
    parser.add_argument('--output', required=True)
    parser.add_argument('--idle-seconds', type=float, default=10.)
    parser.add_argument('--forward-seconds', type=float, default=20.)
    parser.add_argument('--stop-seconds', type=float, default=3.)
    args = parser.parse_args()
    result = run(args.config, args.scenario, args.idle_seconds, args.forward_seconds, args.stop_seconds)
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps(result['summary'], indent=2), flush=True)


if __name__ == '__main__':
    main()
