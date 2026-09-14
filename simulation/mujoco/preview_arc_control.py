"""Render reproducible physical A/B arc trials with contact/height overlays.

The JSON files are search_arc_control results. Rendering reruns the controller
and physics, never interpolates a scripted joint animation. No hardware I/O.
"""
import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from cad_physics import Simulation, build, foot_clearance
from search_gait_profiles import physics
from virtual_robot import RobotController

ROOT = Path(__file__).resolve().parent


def render(cases, output, stop_at=18., seconds=22.):
    robots = []
    renderers = []
    for case in cases:
        parameters, _ = physics(case.get('scenario', 'nominal'))
        parameters.update(case['parameters'], tracking_feedback_enabled=False)
        parameters['foot_cushion'] = json.loads((ROOT/'foot_cushion_10mm.json').read_text())
        xml, parameters = build(parameters, write_scene=False)
        model = mujoco.MjModel.from_xml_string(xml)
        model.vis.global_.offwidth = 640
        model.vis.global_.offheight = 360
        robot = RobotController(Simulation(parameters, model))
        robot.select_profile('arcturn')
        robots.append(robot)
        renderers.append(mujoco.Renderer(model, height=360, width=640))
    output.parent.mkdir(parents=True, exist_ok=True)
    width = 640*len(cases)
    font = ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc', 22)
    small = ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc', 18)
    camera = mujoco.MjvCamera()
    camera.distance = 1.15
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    encoder = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
        '-pixel_format', 'rgb24', '-video_size', f'{width}x930', '-framerate', '25',
        '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE)
    try:
        for frame in range(round(seconds/.02)+1):
            now = frame*.02
            for case, robot in zip(cases, robots):
                if frame == 100:
                    robot.command('drive 0 0 1', now)
                if frame >= 100 and now < stop_at and frame % 10 == 0:
                    robot.command(f"@D {frame} {case.get('linear',0)} {case.get('yaw',-1000)}", now)
                if frame == round(stop_at/.02):
                    robot.command(f'@S {frame+1}', now)
                robot.tick(now)
                robot.drain()
            if frame % 2:
                continue
            canvas = Image.new('RGB', (width, 930), '#172332')
            draw = ImageDraw.Draw(canvas)
            for index, (case, robot, renderer) in enumerate(zip(cases, robots, renderers)):
                x = 640*index
                model, data = robot.plant.model, robot.plant.data
                row = robot.plant.row()
                draw.text((x+16, 8), case.get('label', case['name']), font=font, fill='white')
                draw.text((x+16, 40), f'{now:.2f}초 · '+('Stop' if now >= stop_at else '좌회전' if case.get('yaw',-1000)<0 else '우회전'), font=small, fill='#b8d5e9')
                camera.lookat[:] = data.xipos[model.body('cad_base').id]
                camera.lookat[2] -= .06
                rotation = data.xmat[model.body('robot').id].reshape(3, 3)
                heading = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
                for view, azimuth in enumerate((90, 0)):
                    camera.azimuth = heading+azimuth
                    camera.elevation = -15
                    renderer.update_scene(data, camera=camera, scene_option=options)
                    canvas.paste(Image.fromarray(renderer.render()), (x, 75+view*360))
                feet = [model.geom(leg+'_foot').id for leg in ('fl','fr','rl','rr')]
                clearance = [1000*foot_clearance(model, data, foot) for foot in feet]
                forces = np.zeros(4)
                for contact_index, contact in enumerate(data.contact):
                    pair = (contact.geom1, contact.geom2)
                    if model.geom('floor').id not in pair:
                        continue
                    for leg, foot in enumerate(feet):
                        if foot in pair:
                            force = np.zeros(6)
                            mujoco.mj_contactForce(model, data, contact_index, force)
                            forces[leg] += max(0., force[0])
                support = ', '.join(leg for leg, force in zip(('FL','FR','RL','RR'),forces) if force>.2) or '없음'
                draw.text((x+16, 800), f'Roll {row["roll_deg"]:+.2f}° / Pitch {row["pitch_deg"]:+.2f}° · 접지 {support}', font=small, fill='white')
                draw.text((x+16, 831), '발 여유 FL/FR/RL/RR: '+' / '.join(f'{v:.1f}' for v in clearance)+' mm', font=small, fill='#a8e8ff')
                draw.text((x+16, 862), '추정 물성 · MuJoCo 물리 계산 · 지면 접촉은 평가용', font=small, fill='#ffcd86')
                draw.text((x+16, 893), case.get('verdict', '실험 후보 · 전체 기준 통과 전'), font=small, fill='#ffcd86')
            encoder.stdin.write(np.asarray(canvas).tobytes())
            if frame == 350:
                canvas.save(output.with_suffix('.png'))
    finally:
        encoder.stdin.close()
        status = encoder.wait()
        for renderer in renderers:
            renderer.close()
        if status:
            raise RuntimeError('Video encoder failed')
    output.with_suffix('.cases.json').write_text(json.dumps(cases, indent=2, ensure_ascii=False))
    print(output, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stop-at', type=float, default=18.)
    parser.add_argument('--seconds', type=float, default=22.)
    args = parser.parse_args()
    if not 1 <= len(args.cases) <= 2 or not 5 < args.stop_at < args.seconds:
        parser.error('One or two cases, and 5 < stop-at < seconds, are required')
    cases = []
    for path in args.cases:
        contents = json.loads(path.read_text())
        cases.append(contents.get('case', contents))
    render(cases, args.output, args.stop_at, args.seconds)
