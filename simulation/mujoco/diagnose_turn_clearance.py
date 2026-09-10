"""Measure swing-foot clearance/contact in the deployed shared-C drive kernel.

No hardware commands are sent. Contact and mass parameters are estimates.
"""
import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from cad_physics import Simulation
from search_gait_profiles import physics
from virtual_robot import RobotController


def run(linear, yaw, seconds, capture=None):
    parameters, model = physics()
    robot = RobotController(Simulation(parameters, model))
    robot.select_profile('jointsport')
    feet = [model.geom(f'{leg}_foot').id for leg in ('fl', 'fr', 'rl', 'rr')]
    floor = model.geom('floor').id
    rows = []
    capture_state = None
    capture_distance = float('inf')
    for frame in range(round(seconds / .02)):
        now = frame * .02
        if frame == 100:
            robot.command('drive 0 0 1', now)
        if frame >= 100 and frame % 10 == 0:
            robot.command(f'@D {frame} {linear} {yaw}', now)
        phase = robot.phase
        robot.tick(now)
        robot.drain()
        if frame < 250:
            continue
        p = robot.active_profile_params()
        offsets = np.array([0.,.5,.5,0.])
        state = robot.plant.row()
        fr_u = (((phase + .5) % 1) - p[1]) / (1-p[1])
        if now >= seconds-2 and abs(fr_u-.5) < capture_distance:
            capture_distance = abs(fr_u-.5)
            capture_state = robot.plant.data.qpos.copy()
            capture_info = dict(time_s=now, fr_swing_fraction=fr_u,
                fr_clearance_mm=1000*(robot.plant.data.geom_xpos[feet[1], 2]-model.geom_size[feet[1], 0]))
        forces = dict.fromkeys(feet, 0.)
        for index, contact in enumerate(robot.plant.data.contact):
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            if other in forces:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, robot.plant.data, index, force)
                forces[other] += max(0., force[0])
        for leg, foot in enumerate(feet):
            q = (phase + offsets[leg]) % 1
            u = (q - p[1]) / (1 - p[1])
            rows.append(dict(time_s=now, leg=('FL', 'FR', 'RL', 'RR')[leg],
                phase=q, swing=bool(q >= p[1]), middle_swing=bool(.2 <= u <= .8),
                clearance_mm=1000*(robot.plant.data.geom_xpos[foot, 2]-model.geom_size[foot, 0]),
                force_n=forces[foot], j1_target=state['target_deg'][3*leg],
                j1_actual=state['actual_deg'][3*leg], j2_target=state['target_deg'][3*leg+1],
                j3_target=state['target_deg'][3*leg+2],
                tilt_deg=max(abs(state['roll_deg']), abs(state['pitch_deg'])),
                safety=robot.safety, moving=robot.motion is not None))
    summary = dict(linear=linear, yaw=yaw, safety=robot.safety,
                   estimated_physics=True, profile=robot.profiles['jointsport'], legs={})
    for leg in ('FL', 'FR', 'RL', 'RR'):
        samples = [r for r in rows if r['leg'] == leg and r['moving']]
        swing = [r for r in samples if r['swing']]
        middle = [r for r in samples if r['middle_swing']]
        summary['legs'][leg] = dict(
            samples=len(samples), swing_samples=len(swing),
            swing_contact_fraction=float(np.mean([r['force_n'] > .2 for r in swing])) if swing else None,
            middle_swing_contact_fraction=float(np.mean([r['force_n'] > .2 for r in middle])) if middle else None,
            peak_clearance_mm=max((r['clearance_mm'] for r in swing), default=None),
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
