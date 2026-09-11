"""Measure swing-foot clearance/contact in the deployed shared-C drive kernel.

No hardware commands are sent. Contact and mass parameters are estimates.
"""
import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from cad_physics import Simulation, foot_clearance, geom_extent_along, build
from search_gait_profiles import physics
from virtual_robot import RobotController


def run(linear, yaw, seconds, capture=None, profile='jointsport', override=None, scenario='nominal', cushion=None,
        observer=None, stop_at=None, parameter_overrides=None):
    parameters, model = physics(scenario)
    if cushion or parameter_overrides:
        parameters.update(parameter_overrides or {})
    if cushion:
        parameters['foot_cushion']=cushion
    if cushion or parameter_overrides:
        xml,parameters=build(parameters,write_scene=False)
        model=mujoco.MjModel.from_xml_string(xml)
    robot = RobotController(Simulation(parameters, model))
    if override is not None:
        robot.profiles[profile] = override
    robot.select_profile(profile)
    feet = [model.geom(f'{leg}_foot').id for leg in ('fl', 'fr', 'rl', 'rr')]
    floor = model.geom('floor').id
    rows = []
    previous_force = dict.fromkeys(feet, 0.)
    previous_foot_position = {}
    travel_start = None
    travel_time = None
    capture_state = None
    capture_distance = float('inf')
    for frame in range(round(seconds / .02)):
        now = frame * .02
        if frame == 100:
            robot.command('drive 0 0 1', now)
        if frame >= 100 and frame % 10 == 0 and (stop_at is None or now < stop_at):
            robot.command(f'@D {frame} {linear} {yaw}', now)
        if stop_at is not None and frame == round(stop_at/.02):
            robot.command(f'@S {frame+1}', now)
        phase = robot.phase
        robot.tick(now)
        robot.drain()
        if observer is not None:
            observer(now, robot)
        if frame < 250:
            continue
        if travel_start is None:
            travel_start = robot.plant.data.xpos[model.body('robot').id].copy()
            travel_time = now
        p = robot.active_profile_params()
        offsets = np.array([0.,.5,.5,0.])
        state = robot.plant.row()
        fr_u = (((phase + .5) % 1) - p[1]) / (1-p[1])
        if now >= seconds-2 and abs(fr_u-.5) < capture_distance:
            capture_distance = abs(fr_u-.5)
            capture_state = robot.plant.data.qpos.copy()
            capture_info = dict(time_s=now, fr_swing_fraction=fr_u,
                fr_clearance_mm=1000*(foot_clearance(model,robot.plant.data,feet[1])))
        forces = dict.fromkeys(feet, 0.)
        for index, contact in enumerate(robot.plant.data.contact):
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            if other in forces:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, robot.plant.data, index, force)
                forces[other] += max(0., force[0])
        forward_axis=robot.plant.data.xmat[model.body('robot').id].reshape(3,3)[:,0].copy()
        forward_axis[2]=0;forward_axis/=np.linalg.norm(forward_axis)
        for leg, foot in enumerate(feet):
            local_forward=robot.plant.data.geom_xmat[foot].reshape(3,3).T@forward_axis
            front_extent=geom_extent_along(model,robot.plant.data,foot,forward_axis)
            forward_center=1000*float(np.dot(robot.plant.data.geom_xpos[foot]-robot.plant.data.xanchor[model.joint(('fl','fr','rl','rr')[leg]+'_j1').id],forward_axis))
            q = (phase + offsets[leg]) % 1
            u = (q - p[1]) / (1 - p[1])
            position=robot.plant.data.geom_xpos[foot].copy()
            speed=float(np.linalg.norm(position[:2]-previous_foot_position.get(foot,position)[:2])/.02)
            previous_foot_position[foot]=position
            rows.append(dict(roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
                contact_center_speed_m_s=speed, j1_correction_deg=robot.command_target[3*leg]-robot.target[3*leg],
                adaptive_scale=(robot.position_wbc.scale if robot.profiles[robot.profile].get("position_wbc") else robot.support_j1.scale), time_s=now, leg=('FL', 'FR', 'RL', 'RR')[leg],
                phase=q, swing=bool(q >= p[1]), middle_swing=bool(.2 <= u <= .8),
                clearance_mm=1000*(foot_clearance(model,robot.plant.data,foot)),
                forward_from_j1_mm=forward_center,
                front_edge_from_j1_mm=forward_center+1000*front_extent,
                touchdown=forces[foot]>.2 and previous_force[foot]<=.2,
                force_n=forces[foot], j1_target=state['target_deg'][3*leg],
                j1_actual=state['actual_deg'][3*leg], j2_target=state['target_deg'][3*leg+1],
                j3_target=state['target_deg'][3*leg+2],
                tilt_deg=max(abs(state['roll_deg']), abs(state['pitch_deg'])),
                safety=robot.safety, moving=robot.motion is not None))
        previous_force=forces.copy()
    summary = dict(linear=linear, yaw=yaw, safety=robot.safety,
                   estimated_physics=True, profile=robot.profiles[profile], scenario=scenario, cushion=cushion,
                   speed_m_s=float(np.linalg.norm(robot.plant.data.xpos[model.body('robot').id][:2]-travel_start[:2])/(seconds-.02-travel_time)), legs={})
    for leg in ('FL', 'FR', 'RL', 'RR'):
        samples = [r for r in rows if r['leg'] == leg and r['moving']]
        swing = [r for r in samples if r['swing']]
        middle = [r for r in samples if r['middle_swing']]
        front_edges=[r['front_edge_from_j1_mm'] for r in samples if r['touchdown']]
        touchdowns=[r['forward_from_j1_mm'] for r in samples if r['touchdown']]
        summary['legs'][leg] = dict(
            rms_roll_deg=float(np.sqrt(np.mean([r['roll_deg']**2 for r in samples]))) if samples else None,
            rms_pitch_deg=float(np.sqrt(np.mean([r['pitch_deg']**2 for r in samples]))) if samples else None,
            stance_center_speed_m_s=float(np.mean([r['contact_center_speed_m_s'] for r in samples if r['force_n']>.2 and not r['swing']])) if any(r['force_n']>.2 and not r['swing'] for r in samples) else None,
            max_j1_correction_deg=max((abs(r['j1_correction_deg']) for r in samples),default=None),
            minimum_adaptive_scale=min((r['adaptive_scale'] for r in samples),default=None),
            touchdown_front_edge_median_mm=float(np.median(front_edges)) if front_edges else None,
            touchdown_forward_median_mm=float(np.median(touchdowns)) if touchdowns else None,
            touchdown_forward_min_mm=min(touchdowns,default=None),
            samples=len(samples), swing_samples=len(swing),
            swing_contact_fraction=float(np.mean([r['force_n'] > .2 for r in swing])) if swing else None,
            middle_swing_contact_fraction=float(np.mean([r['force_n'] > .2 for r in middle])) if middle else None,
            peak_clearance_mm=max((r['clearance_mm'] for r in swing), default=None),
            peak_contact_compression_proxy_mm=max((max(0.,-r['clearance_mm']) for r in samples), default=None),
            peak_j1_error_deg=max((abs(r['j1_target']-r['j1_actual']) for r in samples), default=None),
            peak_tilt_deg=max((r['tilt_deg'] for r in samples), default=None))
    if capture is not None and capture_state is not None:
        from PIL import Image
        robot.plant.data.qpos[:] = capture_state
        mujoco.mj_forward(model, robot.plant.data)
        model.vis.global_.offwidth = 960
        model.vis.global_.offheight = 720
        camera = mujoco.MjvCamera()
        camera.lookat[:] = robot.plant.data.xipos[model.body('cad_base').id]
        camera.lookat[2] -= .06
        rotation = robot.plant.data.xmat[model.body('robot').id].reshape(3, 3)
        camera.azimuth = -45 + np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
        camera.elevation = -10
        camera.distance = 1.05
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(robot.plant.data, camera=camera, scene_option=options)
            Image.fromarray(renderer.render()).save(capture)
        summary['capture'] = capture_info
    return summary, rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear', type=int, default=177)
    parser.add_argument('--yaw', type=int, default=221)
    parser.add_argument('--seconds', type=float, default=14)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture', type=Path)
    args = parser.parse_args()
    if not (-1000 <= args.linear <= 1000 and -1000 <= args.yaw <= 1000 and args.seconds > 5):
        parser.error('Commands must be within +/-1000; duration must exceed 5 s')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.capture:
        args.capture.parent.mkdir(parents=True, exist_ok=True)
    summary, rows = run(args.linear, args.yaw, args.seconds, args.capture)
    args.output.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
    with args.output.with_suffix('.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(summary, indent=2))
