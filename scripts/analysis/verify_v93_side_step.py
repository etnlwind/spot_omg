"""Offline V7/V8 lateral audit: full time histories, fixed cameras, no robot IO."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools/servo_tool')]
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.support_shift import SupportShift

LEGS = ('FL', 'FR', 'RL', 'RR')


def paired_clearance(rows):
    """Measured clearance/load during central 40% of a scheduled pair swing.

    This is not a success criterion for the complete gait or proof of exact
    simultaneous liftoff. Faulted runs must be rejected separately.
    """
    samples=[]
    for row in rows:
        nav=row.get('navigation',{})
        if row['stage']!='walk' or row['walk_s']<4.4 or not nav.get('paired_side') or row.get('safety','ok')!='ok' or not row.get('moving',True):
            continue
        u=(np.array(row['leg_phase'])-nav['duty'])/(1-nav['duty'])
        mask=(u>.3)&(u<.7)
        if mask.sum()==2:
            clear=(np.array(row['clearance_mm'])>2)&(np.array(row['load_n'])<.5)
            samples.append(bool(clear[mask].all()))
    return dict(core_frames=len(samples), both_clear_frames=sum(samples),
                both_clear_fraction=float(np.mean(samples)) if samples else None)


def summary(rows):
    walk = [r for r in rows if r['stage'] == 'walk']
    steady = [r for r in walk if r['walk_s'] >= 4.4]
    def measures(part):
        xyz = np.array([r['position_mm'] for r in part])
        angles = np.array([[r['roll_deg'], r['pitch_deg'], r['yaw_deg']] for r in part])
        return dict(x_range_mm=float(np.ptp(xyz[:, 0])), y_range_mm=float(np.ptp(xyz[:, 1])),
                    yaw_range_deg=float(np.ptp(angles[:, 2])),
                    roll_range_deg=float(np.ptp(angles[:, 0])), pitch_range_deg=float(np.ptp(angles[:, 1])),
                    tilt_peak_deg=float(np.max(abs(angles[:, :2]))),
                    min_loaded_feet=min(sum(f > .5 for f in r['load_n']) for r in part),
                    tracking_peak_deg=max(r['tracking_error_deg'] for r in part))
    initial = rows[99]['position_mm']
    fault=next((dict(t=r['t'], safety=r['safety']) for r in rows if r['safety'] != 'ok'), None)
    before_fault=[r for r in walk if fault is None or r['t']<fault['t']]
    return dict(paired_clearance=paired_clearance(rows),walk=measures(walk), steady=measures(steady) if steady else None,
                before_fault=measures(before_fault) if before_fault else None,
                displacement_mm=(np.array(walk[-1]['position_mm'])-initial).tolist(),
                final_yaw_deg=walk[-1]['yaw_deg'],
                first_fault=fault,
                normal_stop=fault is None and not rows[-1]['moving'] and not rows[-1]['transitioning'],
                stopped=not rows[-1]['moving'] and not rows[-1]['transitioning'],
                stop=measures([r for r in rows if r['stage'] == 'stop']))


def simulate(args):
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'navigation-source.h.txt').write_bytes((ROOT/'firmware/stm32-learning/Inc/navigation_control.h').read_bytes())
    for source in ('lateral_instep.h','drive_control.h'):
        (args.output/(source+'.txt')).write_bytes((ROOT/'firmware/stm32-learning/Inc'/source).read_bytes())
    parameters = load_parameters(parse_args([]))
    if getattr(args,'v10_config',None) is not None:
        parameters['v10_experiment']=list(args.v10_config)
    if getattr(args,'transfer_mm',None) is not None:
        parameters['v9_transfer_m']=args.transfer_mm*.001
    plant = Simulation(parameters)
    robot = RobotController(plant)
    robot.command('gaitprofile '+args.profile, 0)
    robot.command('footlift save ' + ' '.join(map(str, args.lift + args.width)), 0)
    robot.command('heading ' + ('off' if args.no_heading else 'on'), 0)
    robot.command('stand', 0)
    model, data = plant.model, plant.data
    kin = SupportShift(model)
    feet = [model.geom(leg.lower() + '_foot').id for leg in LEGS]
    floor = model.geom('floor').id
    body = model.body('robot').id
    rows, qpos, qvel = [], [], []
    stop_tick = round((2 + args.seconds) * 50)
    stop_sent = False
    for i in range(stop_tick + 200):
        now = float(data.time)
        if i == 100:
            robot.command(f'drive 0 {args.input} 1', now)
        elif i == stop_tick:
            robot.command('@S 10000', now)
            stop_sent = True
        elif 100 < i < stop_tick and i % 10 == 0 and robot.motion is not None:
            robot.command(f'@D {i+1} 0 {args.input}', now)
        robot.tick(now)
        d = plant.row()
        rotation = data.xmat[body].reshape(3, 3)
        yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        load = np.zeros(4)
        contact_count = np.zeros(4, dtype=int)
        for j in range(data.ncon):
            c = data.contact[j]
            if floor not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == floor else c.geom1
            if other in feet:
                leg = feet.index(other)
                f = np.zeros(6)
                mujoco.mj_contactForce(model, data, j, f)
                load[leg] += max(0, f[0])
                contact_count[leg] += 1
        nav = dict(getattr(robot, 'navigation_frame', {}))
        offsets = [.3, .8, .55, .05] if nav.get('lateral', 0) < 0 else [.8, .3, .05, .55]
        if nav.get('paired_side'):
            offset=nav['pair_offset'];offsets=[offset,offset-.5,offset-.5,offset]
            if nav.get('ipsilateral_side'):offsets=[offset,offset+.5,offset,offset+.5]
        phase = float(getattr(robot, 'nominal_phase', 0))
        leg_phase = ((phase + np.array(offsets)) % 1).tolist()
        kin.set_angles(robot.command_target)
        target_xyz = np.array([kin.foot(j) for j in range(4)])
        kin.set_angles(np.degrees(data.qpos[plant.q]))
        actual_xyz = np.array([kin.foot(j) for j in range(4)])
        row = dict(t=now + .02, walk_s=now + .02-2,
                   stage='stop' if stop_sent else 'walk' if i >= 100 else 'wait',
                   position_mm=(data.qpos[:3] * 1000).tolist(),
                   com_mm=(data.subtree_com[body]*1000).tolist(),
                   roll_deg=d['roll_deg'], pitch_deg=d['pitch_deg'], yaw_deg=yaw,
                   phase=phase, leg_phase=leg_phase, navigation=nav,
                   load_n=load.tolist(), contacts=contact_count.tolist(),
                   clearance_mm=[foot_clearance(model, data, f)*1000 for f in feet],
                   foot_world_mm=(data.geom_xpos[feet]*1000).tolist(),
                   nominal_deg=robot.target.tolist(), target_deg=robot.command_target.tolist(),
                   actual_deg=d['actual_deg'], target_foot_mm=(target_xyz*1000).tolist(),
                   actual_foot_mm=(actual_xyz*1000).tolist(),
                   tracking_error_deg=d['max_tracking_error_deg'], voltage_v=d['voltage_v'],
                   safety=robot.safety, moving=robot.motion is not None,
                   transitioning=robot.transition is not None, heading=robot.heading.diagnostic(),
                   reply=robot.drain().decode())
        rows.append(row)
        qpos.append(data.qpos.copy())
        qvel.append(data.qvel.copy())
    report = dict(profile=args.profile,input=args.input, lift_mm=args.lift, width_mm=args.width,
                  host_library=str(plant.policy._library._name),
                  heading=not args.no_heading, seconds=args.seconds, physics='estimated',
                  parameters=parameters, summary=summary(rows),
                  source_hash=hashlib.sha256(b''.join((ROOT/'firmware/stm32-learning/Inc'/f).read_bytes() for f in ('navigation_control.h','lateral_instep.h','drive_control.h'))).hexdigest(),
                  rows=rows)
    (args.output/'trajectory.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    np.savez_compressed(args.output/'poses.npz', qpos=qpos, qvel=qvel)
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','parameters')}, indent=2), flush=True)
    return report, np.array(qpos), np.array(qvel), plant


def render(args, report, poses, velocities, plant):
    import imageio_ffmpeg
    from PIL import Image, ImageDraw, ImageFont
    model, data = plant.model, plant.data
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 360
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 18)
    small = ImageFont.truetype('C:/Windows/Fonts/consola.ttf', 15)
    camera = mujoco.MjvCamera()
    # Fixed world position AND azimuth; heading-follow cameras can hide yaw drift.
    data.qpos[:] = poses[99]
    data.qvel[:] = velocities[99]
    mujoco.mj_forward(model, data)
    center = data.subtree_com[model.body('robot').id].copy() + [0, 0, -.08]
    camera.lookat[:] = center
    camera.distance = 1.05
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    path = args.output/'four-views.mp4'
    encoder = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x930', '-r', '25', '-i', '-',
        '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)
    initial = np.array(report['rows'][99]['position_mm'])
    try:
        with mujoco.Renderer(model, height=360, width=640) as renderer:
            for i in range(0, len(poses), 2):
                row = report['rows'][i]
                data.qpos[:] = poses[i]
                data.qvel[:] = velocities[i]
                mujoco.mj_forward(model, data)
                canvas = Image.new('RGB', (1280, 930), '#111b25')
                draw = ImageDraw.Draw(canvas)
                direction = 'RIGHT' if report['input'] > 0 else 'LEFT'
                delta = np.array(row['position_mm']) - initial
                draw.text((12, 7), f'{report.get("profile", "attitudepd_v7")} | {direction} {abs(report["input"])/10:g}% | 2s wait + {report["seconds"]:g}s side + 4s stop | estimated physics', font=font, fill='white')
                draw.text((12, 32), f't={row["t"]:.2f}s [{row["stage"]}]  X {delta[0]:+.1f} / Y {delta[1]:+.1f} mm   Roll {row["roll_deg"]:+.1f} / Pitch {row["pitch_deg"]:+.1f} / Yaw {row["yaw_deg"]:+.1f} deg', font=font, fill='white')
                draw.text((12, 57), f'FL FR RL RR | lift {report["lift_mm"]} mm | width {report["width_mm"]} mm | fixed cameras | mass {model.body_mass.sum():.3f} kg', font=font, fill='#acd1de')
                for panel, (label, azimuth, elevation) in enumerate([('FRONT',180,0),('REAR',0,0),('TOP',90,-90),('SIDE',90,0)]):
                    camera.azimuth = azimuth
                    camera.elevation = elevation
                    renderer.update_scene(data, camera=camera, scene_option=options)
                    tile = Image.fromarray(renderer.render())
                    ImageDraw.Draw(tile).text((10,8), label, font=font, fill='#101010')
                    canvas.paste(tile, ((panel%2)*640,90+(panel//2)*360))
                for leg in range(4):
                    x = 12+leg*320
                    draw.text((x,818), f'{LEGS[leg]} load {row["load_n"][leg]:4.1f}N clear {row["clearance_mm"][leg]:5.1f}mm', font=small, fill='#c2edee')
                    for j in range(3):
                        k=leg*3+j
                        draw.text((x,841+j*20), f'J{j+1} cmd {row["target_deg"][k]:6.1f} / act {row["actual_deg"][k]:6.1f}', font=small, fill='white')
                draw.text((12,905), f'Safety: {row["safety"]} | phase {row["phase"]:.3f} | target tracking peak {row["tracking_error_deg"]:.1f} deg | heading hold {report["heading"]}', font=small, fill='#acd1de')
                encoder.stdin.write(canvas.tobytes())
                if i == 350:
                    canvas.save(args.output/'preview.png')
    finally:
        encoder.stdin.close()
        code=encoder.wait()
        if code:
            raise RuntimeError(f'Video encoding failed {code}')
    print(path, flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',default='attitudepd_v7',choices=('attitudepd_v7','attitudepd_v8','attitudepd_v9','attitudepd_v10'))
    p.add_argument('--transfer-mm',type=float,help='Offline V9 load transfer amplitude, 0..120mm; firmware default 60mm')
    p.add_argument('--input', type=int, choices=(-1000,-588,588,1000), default=1000)
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--lift',type=int,nargs=4,default=[20]*4)
    p.add_argument('--width',type=int,nargs=4,default=[-30,-30,30,30])
    p.add_argument('--no-heading',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--video',action='store_true')
    p.add_argument('--render-existing',action='store_true')
    p.add_argument('--v10-config',type=float,nargs=6,help='Offline only: step m, base lift m, swing phase, push fraction, catch phase, hip delay')
    args=p.parse_args()
    if args.render_existing:
        report=json.loads((args.output/'trajectory.json').read_text(encoding='utf-8'))
        arrays=np.load(args.output/'poses.npz')
        plant=Simulation(report['parameters'])
        render(args, report, arrays['qpos'], arrays['qvel'], plant)
    else:
        result=simulate(args)
        if args.video:render(args,*result)


if __name__=='__main__':
    main()
