"""Capture measured tilt growth, foot loading and rearward reach in S gaits."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import argparse
import json
import subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.s_native_gait import PROFILES
from simulation.mujoco.runtime.standing_pose import SoleKinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=list(PROFILES), required=True)
    parser.add_argument('--command', type=int, default=600)
    parser.add_argument('--walk-seconds', type=int, default=10)
    parser.add_argument('--settle-seconds', type=int, default=4,
                        help='Record return to S after STOP (0 keeps a walking-only capture).')
    parser.add_argument('--ffmpeg', default='ffmpeg')
    parser.add_argument('--allow-fall', action='store_true')
    parser.add_argument('--fourth-view', choices=('side','bottom'), default='side')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.command <= 1000:
        parser.error('command must be 1..1000')
    if not 3 <= args.walk_seconds <= 120:
        parser.error('walk-seconds must be 3..120')
    if not 0 <= args.settle_seconds <= 30:
        parser.error('settle-seconds must be 0..30')
    stop_tick = (2 + args.walk_seconds)*50
    total_ticks = stop_tick + args.settle_seconds*50
    plant = Simulation(load_parameters(parse_args(['--allow-fall'] if args.allow_fall else [])))
    robot = RobotController(plant)
    robot.profile = args.profile
    model, data = plant.model, plant.data
    kin = SoleKinematics(model, plant.stand_target)
    origin = np.array([kin.foot(i) for i in range(4)])
    floor = model.geom('floor').id
    legs = ('FL', 'FR', 'RL', 'RR')
    records, poses, velocities = [], [], []
    for i in range(total_ticks):
        t = i * .02
        if i == 100:
            robot.command(f'drive {args.command} 0 1', t)
        elif i == stop_tick:
            robot.command(f'@S {i}', t)
        elif 100 < i < stop_tick and i % 10 == 0:
            robot.command(f'@D {i} {args.command} 0', t)
        robot.tick(t)
        state = plant.row()
        kin.set_angles(np.degrees(data.qpos[plant.q]))
        load = np.zeros(4)
        for j in range(data.ncon):
            contact = data.contact[j]
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            for leg, foot in enumerate(kin.feet):
                if other == foot:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, j, force)
                    load[leg] += max(0., force[0])
        placement=''
        gait=getattr(robot,'s_native_gait',None)
        if gait is not None and hasattr(gait,'stop_first'):
            progress=gait.stop_progress
            pair=gait.stop_first if progress<.5 else [j for j in range(4) if j not in gait.stop_first]
            placement=('S HOLD' if progress>=1 else
                       f'STOP STEP {1 if progress<.5 else 2}: '+ '+'.join(legs[j] for j in pair))
        records.append(dict(time_s=t, since_drive_s=t-2,
            stop_placement=placement,
            phase=float(getattr(robot, 'nominal_phase', 0)),
            roll_deg=state['roll_deg'], pitch_deg=state['pitch_deg'],
            tilt_deg=max(abs(state['roll_deg']), abs(state['pitch_deg'])),
            clearance_mm=[foot_clearance(model, data, foot)*1000 for foot in kin.feet],
            normal_force_n=load.tolist(),
            body_relative_x_from_s_mm=[(kin.foot(j)[0]-origin[j, 0])*1000 for j in range(4)],
            body_relative_y_from_j1_mm=[(kin.foot(j)[1]-kin.data.xanchor[model.joint(leg.lower()+'_j1').id,1])*1000 for j,leg in enumerate(legs)],
            j1_target_deg=robot.command_target[::3].tolist(),
            j1_actual_deg=np.degrees(data.qpos[plant.q])[::3].tolist(),
            j3_target_deg=robot.command_target[2::3].tolist(),
            j3_actual_deg=np.degrees(data.qpos[plant.q])[2::3].tolist(),
            lateral_placement_fraction=(robot.s_native_gait.placement_fraction.tolist() if hasattr(robot,'s_native_gait') else [0.]*4),
            normal_adduction_deg=(robot.s_native_gait.normal_adduction.tolist() if hasattr(robot,'s_native_gait') and hasattr(robot.s_native_gait,'normal_adduction') else None),
            safety=robot.safety, moving=robot.motion is not None,
            transitioning=robot.transition is not None,
            target_s_error_deg=float(np.max(np.abs(robot.command_target-robot.stand_target))),
            actual_s_error_deg=float(np.max(np.abs(np.degrees(data.qpos[plant.q])-robot.stand_target))),
            controller_reply=robot.drain().decode()))
        poses.append(data.qpos.copy())
        velocities.append(data.qvel.copy())
    # A reproducible observation marker, not a claim of a universal fall threshold.
    candidates = [j for j in range(100, total_ticks-3)
                  if all(r['tilt_deg'] >= 5 for r in records[j:j+4])]
    onset = candidates[0] if candidates else max(range(100, total_ticks), key=lambda j: records[j]['tilt_deg'])
    peak = max(range(onset, min(total_ticks, onset+51)), key=lambda j: records[j]['tilt_deg'])
    tipping = next((j for j in range(100, total_ticks) if records[j]['tilt_deg'] >= 45), None)
    toppled = next((j for j in range(100, total_ticks) if records[j]['tilt_deg'] >= 90), None)
    selected = [('Before', max(100, onset-10)), ('Tilt growth', onset), ('Peak within 1s', peak)]
    args.output.mkdir(parents=True, exist_ok=True)
    font_path = 'C:/Windows/Fonts/arial.ttf' if Path('C:/Windows/Fonts/arial.ttf').exists() else '/System/Library/Fonts/Supplemental/Arial.ttf'
    font = ImageFont.truetype(font_path, 16) if Path(font_path).exists() else ImageFont.load_default()
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 400
    renderer = mujoco.Renderer(model, height=400, width=640)
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    camera = mujoco.MjvCamera()
    camera.elevation = 0
    # Leave room above the body for legs during the requested fall observation.
    camera.distance = 1.35
    floor_group = int(model.geom_group[floor])
    model.geom_group[floor] = 5

    def render(index, azimuth, label, elevation=0):
        data.qpos[:] = poses[index]
        data.qvel[:] = velocities[index]
        mujoco.mj_forward(model, data)
        rotation = data.xmat[model.body('robot').id].reshape(3, 3)
        yaw = np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
        camera.azimuth = azimuth + yaw
        camera.elevation = elevation
        # Hide only the rendered floor in the underside view; physics is unchanged.
        options.geomgroup[5] = int(elevation != 90)
        camera.lookat[:] = data.subtree_com[model.body('robot').id]+[0, 0, .03]
        renderer.update_scene(data, camera=camera, scene_option=options)
        frame = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(frame)
        row = records[index]
        draw.rectangle((0, 0, 640, 56), fill='#17232f')
        timing = 'WAIT' if index < 100 else f"walk +{row['since_drive_s']:.2f}s"
        if index >= stop_tick:
            settled = not row['moving'] and not row['transitioning'] and row['target_s_error_deg'] < .01
            timing = ('S HOLD' if settled else 'STOP / RETURN TO S') + f" +{(index-stop_tick)*.02:.2f}s"
        draw.text((10, 5), f"{label} | video {row['time_s']:.2f}s | {timing}", font=font, fill='white')
        draw.text((10, 30), f"Roll {row['roll_deg']:+.1f} deg | Pitch {row['pitch_deg']:+.1f} deg | phase {row['phase']:.3f}", font=font, fill='white')
        draw.rectangle((0, 350, 640, 400), fill='#17232f')
        draw.text((10, 354), 'Ground load N: '+ '  '.join(f'{l} {n:.1f}' for l,n in zip(legs,row['normal_force_n'])), font=font, fill='white')
        draw.text((10, 378), 'Foot height mm: '+ '  '.join(f'{l} {z:.1f}' for l,z in zip(legs,row['clearance_mm'])), font=font, fill='white')
        if candidates and onset <= index < onset+25:
            draw.rectangle((5, 65, 365, 94), fill='#aa392d')
            draw.text((12, 70), f"TILT GROWTH from {records[onset]['time_s']:.2f}s", font=font, fill='white')
        if tipping is not None and index >= tipping:
            message = f"TIPPING from {records[tipping]['time_s']:.2f}s"
            if toppled is not None and index >= toppled:
                message = f"TOPPLED at {records[toppled]['time_s']:.2f}s"
            draw.rectangle((5, 65, 365, 94), fill='#aa392d')
            draw.text((12, 70), message, font=font, fill='white')
        if elevation == 90:
            draw.text((10, 330), 'Floor hidden in this view', font=font, fill='#17232f')
        return frame

    active = np.array([r['body_relative_x_from_s_mm'] for r in records[200:stop_tick]])
    returned = next((r for r in records[stop_tick:] if not r['moving'] and not r['transitioning']
                     and r['target_s_error_deg'] < .01), None)
    report = dict(profile=args.profile, parameters=PROFILES[args.profile], command=args.command,
        allow_fall=args.allow_fall, wait_s=2, walk_s=args.walk_seconds,
        settle_s=args.settle_seconds, video_s=total_ticks*.02, fps=25,
        stop_video_s=stop_tick*.02 if args.settle_seconds else None,
        returned_to_s_video_s=returned['time_s'] if returned else None,
        final_target_s_error_deg=records[-1]['target_s_error_deg'],
        final_actual_s_error_deg=records[-1]['actual_s_error_deg'],
        views=['front','rear','top',args.fourth_view],
        selection='First >=5 degree roll/pitch sustained for four 20ms samples; an observation marker, not a stability pass/fail definition.',
        tilt_growth_detected=bool(candidates),
        tipping_video_s=records[tipping]['time_s'] if tipping is not None else None,
        toppled_video_s=records[toppled]['time_s'] if toppled is not None else None,
        selected=[dict(label=label, **records[index]) for label,index in selected],
        actual_body_relative_x_mm={l:dict(min=float(active[:,j].min()),max=float(active[:,j].max())) for j,l in enumerate(legs)},
        peak_tilt_during_drive_deg=max(r['tilt_deg'] for r in records[100:stop_tick]),
        note='Estimated physical simulation; all four views render the same physical state. Underside rendering hides the floor only. Contact forces and foot heights are measured, distinct from command timing. Static COM projection alone does not establish dynamic stability.')
    (args.output/'summary.json').write_text(json.dumps(report, indent=2))
    (args.output/'trajectory.json').write_text(json.dumps(records))
    print(json.dumps(report, indent=2), flush=True)

    # MuJoCo azimuth describes the viewing direction: 180 faces the +X front.
    views = [('front', 180, 0), ('rear', 0, 0), ('top', 90, -90),
             ('side',90,0) if args.fourth_view=='side' else ('bottom',90,90)]
    encoders = {}
    def encoder(name, width, height):
        return subprocess.Popen([args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{width}x{height}', '-r', '25',
            '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.output/f'{name}.mp4')], stdin=subprocess.PIPE)
    try:
        sheet = Image.new('RGB', (1920, 800), '#17232f')
        for column, (label, index) in enumerate(selected):
            for row, azimuth in enumerate((90, 0)):
                frame = render(index, azimuth, label + (' / SIDE' if row == 0 else ' / END'))
                sheet.paste(frame, (640*column, 400*row))
                frame.save(args.output/f'{column+1}-{row+1}.png')
        sheet.save(args.output/'balance-points.png')
        frames = [render(j, 90, 'SIDE') for j in range(max(100,onset-15), min(total_ticks,onset+46), 2)]
        frames[0].save(args.output/'balance-window.gif', save_all=True, append_images=frames[1:], duration=40, loop=0)
        encoders = {name: encoder(name, 640, 400) for name,_,_ in views}
        encoders['four-views'] = encoder('four-views', 1280, 880)
        for index in range(0, total_ticks, 2):
            mosaic = Image.new('RGB', (1280, 880), '#17232f')
            draw = ImageDraw.Draw(mosaic)
            draw.text((10, 10), f"{args.profile} | input {args.command/10:.0f}% | 2s wait + {args.walk_seconds}s walk + {args.settle_seconds}s stop/hold | same simulation / 4 views", font=font, fill='white')
            draw.text((10, 37), f'Mass {model.body_mass.sum():.3f} kg | cushion D37.3 x 27 mm | estimated physics | fall observation: {args.allow_fall}', font=font, fill='white')
            if index >= stop_tick:
                draw.text((10, 59), f"{records[index]['stop_placement']} | S joint error: target {records[index]['target_s_error_deg']:.2f} deg | actual {records[index]['actual_s_error_deg']:.2f} deg (max / 12 joints)", font=font, fill='white')
            else:
                row=records[index]
                draw.text((10,59), f"Rear J3 actual: RL {row['j3_actual_deg'][2]:.1f} / RR {row['j3_actual_deg'][3]:.1f} deg | foot X from S: RL {row['body_relative_x_from_s_mm'][2]:+.1f} / RR {row['body_relative_x_from_s_mm'][3]:+.1f} mm",font=font,fill='white')
            for view_index, (name, azimuth, elevation) in enumerate(views):
                frame = render(index, azimuth, name.upper(), elevation)
                encoders[name].stdin.write(np.asarray(frame).tobytes())
                mosaic.paste(frame, ((view_index%2)*640, 80+(view_index//2)*400))
            encoders['four-views'].stdin.write(np.asarray(mosaic).tobytes())
            captures = {min(total_ticks-2,onset+onset%2): 'tilt-onset-four-views.png', total_ticks-2: 'final-four-views.png'}
            if toppled is not None:
                captures[min(total_ticks-2, toppled+toppled%2)] = 'toppled-four-views.png'
            if index in captures:
                mosaic.save(args.output/captures[index])
        for process in encoders.values():
            process.stdin.close()
        for name, process in encoders.items():
            if process.wait() != 0:
                raise RuntimeError(f'Video encoder failed: {name}')
    finally:
        for process in encoders.values():
            if process.poll() is None:
                process.kill()
                process.wait()
        model.geom_group[floor] = floor_group
        renderer.close()


if __name__ == '__main__':
    main()
