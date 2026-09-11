"""Synchronized side/front baseline comparison, with evaluation-only overlays."""
import json
import argparse
import subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from cad_physics import Simulation, build, foot_clearance
from search_gait_profiles import physics
from virtual_robot import RobotController

ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--baseline',default='cushion_forward')
    parser.add_argument('--prefix',default='support-shift')
    parser.add_argument('--candidate',default='cushion_support_shift')
    parser.add_argument('--before-title')
    parser.add_argument('--after-title')
    parser.add_argument('--height-overlay',action='store_true')
    parser.add_argument('--profiles',type=Path,default=ROOT/'upright_profiles.json')
    args=parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    robots=[];renderers=[]
    profiles=json.loads(args.profiles.read_text())['profiles']
    for name in (args.baseline,args.candidate):
        p,_=physics();p['foot_cushion']=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
        xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
        m.vis.global_.offwidth=640;m.vis.global_.offheight=380
        robot=RobotController(Simulation(p,m));robot.load_experimental_profiles(args.profiles);robot.select_profile(name)
        robots.append(robot);renderers.append(mujoco.Renderer(m,height=380,width=640))
    font=ImageFont.truetype('/System/Library/Fonts/Menlo.ttc',16)
    camera=mujoco.MjvCamera();camera.distance=1.05;camera.elevation=-9
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    video=OUT/(args.prefix+'-comparison.mp4')
    writer=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24',
        '-video_size','1280x960','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE)
    try:
        for frame in range(1000):
            now=frame*.02
            for robot in robots:
                if frame==100:robot.command('drive 0 0 1',now)
                if frame>=100 and frame%10==0:robot.command(f'@D {frame} 1000 0',now)
                robot.tick(now);robot.drain()
            if frame%2:continue
            canvas=Image.new('RGB',(1280,960),'#172332');draw=ImageDraw.Draw(canvas)
            for index,(robot,renderer) in enumerate(zip(robots,renderers)):
                m=robot.plant.model;d=robot.plant.data;state=robot.plant.row()
                camera.lookat[:]=d.xipos[m.body('cad_base').id];camera.lookat[2]-=.06
                if args.height_overlay:camera.lookat[2]=.23
                rotation=d.xmat[m.body('robot').id].reshape(3,3)
                yaw=np.degrees(np.arctan2(rotation[1,0],rotation[0,0]))
                title=(('BEFORE: support transfer V1' if args.baseline.endswith('_v1') else 'BASELINE: large step'),
                       ('V2: J1 held, J2/J3 gait (EXPERIMENT)' if args.candidate.endswith('_v2') else 'NEW: support transfer (EXPERIMENT / FAILED)'))[index]
                title=(args.before_title if index==0 else args.after_title) or title
                y=index*480
                profile_name=(args.baseline,args.candidate)[index]
                stride_mm=1000*profiles[profile_name]['params'][2]
                draw.text((12,y+5),f'{title}   t={now:.2f}s   stride={stride_mm:g}mm',font=font,fill='white')
                for col,azimuth in enumerate((90,0)):
                    camera.azimuth=yaw+azimuth
                    renderer.update_scene(d,camera=camera,scene_option=opt)
                    canvas.paste(Image.fromarray(renderer.render()),(col*640,y+30))
                    draw.text((col*640+12,y+35),('SIDE','FRONT')[col],font=font,fill='black')
                feet=[]
                for leg in ('fl','fr','rl','rr'):
                    gid=m.geom(leg+'_foot').id
                    feet.append(f'{leg.upper()} {1000*foot_clearance(m,d,gid):5.1f}mm')
                draw.text((12,y+414),f'roll {state["roll_deg"]:+.2f}deg  pitch {state["pitch_deg"]:+.2f}deg  safety {robot.safety}',font=font,fill='white')
                draw.text((12,y+436),'Clearance: '+' | '.join(feet),font=font,fill='#9fe4ff')
                draw.text((760,y+436),'Contact: '+','.join(state['contacts']),font=font,fill='#b6ffbc')
                footer='Measured simulation contacts only; controller uses delayed IMU + encoders. Estimated physics.'
                if args.height_overlay:
                    h=1000*d.xipos[m.body('cad_base').id,2]
                    diag=robot.support_shift.diagnostic if robot.support_shift else {}
                    target=1000*(diag.get('body_height_target_m',0)+diag.get('body_shift_m',[0,0,0])[2])
                    footer=f'TORSO HEIGHT {h:.1f}mm | reference {target:.1f}mm | camera Z fixed | estimated physics'
                draw.text((12,y+458),footer,font=font,fill='#b4bdc7')
            writer.stdin.write(np.asarray(canvas).tobytes())
            if frame in (330,350,370):canvas.save(OUT/f'{args.prefix}-{now:.1f}s.png')
        print(video,flush=True)
    finally:
        writer.stdin.close()
        status=writer.wait()
        for renderer in renderers:renderer.close()
        if status:raise RuntimeError('video encoding failed')


if __name__=='__main__':main()
