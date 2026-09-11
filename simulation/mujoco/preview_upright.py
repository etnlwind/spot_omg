"""Replay the upright policy in MuJoCo, optionally recording a video; no robot IO."""
import argparse
import json
import time
import subprocess
from pathlib import Path
import mujoco
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation, build
from virtual_robot import RobotController


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--viewer',action='store_true',help='Use mjpython on macOS')
    parser.add_argument('--profile',default='upright')
    parser.add_argument('--video',type=Path)
    parser.add_argument('--seconds',type=float,default=24)
    parser.add_argument('--foot-cushion',type=Path,help='Attached foot cushion parameters')
    args=parser.parse_args()
    if not args.viewer and args.video is None: parser.error('Choose --viewer or --video')
    p,m=physics()
    if args.foot_cushion:
        p['foot_cushion']=json.loads(args.foot_cushion.read_text())
        xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
    r=RobotController(Simulation(p,m))
    r.load_experimental_profiles(Path(__file__).with_name('upright_profiles.json'))
    r.select_profile(args.profile)
    m.vis.global_.offwidth=960;m.vis.global_.offheight=720
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    camera=mujoco.MjvCamera();camera.distance=1.1;camera.azimuth=135;camera.elevation=-12
    viewer=renderer=writer=None
    try:
        if args.viewer:
            from mujoco import viewer as mjviewer
            viewer=mjviewer.launch_passive(m,r.plant.data)
            viewer.cam.distance=1.1;viewer.cam.azimuth=135;viewer.cam.elevation=-12
        if args.video:
            args.video.parent.mkdir(parents=True,exist_ok=True)
            writer=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(args.video)],stdin=subprocess.PIPE)
            renderer=mujoco.Renderer(m,height=720,width=960)
        start=time.monotonic()
        for frame in range(round(args.seconds/.02)):
            if viewer and not viewer.is_running(): break
            now=frame*.02
            if frame==100:r.command('drive 1000 0 1',now)
            if 2<=now<args.seconds-3 and frame%10==0:r.command(f'@D {frame+2} 1000 0',now)
            if frame==round((args.seconds-3)/.02):r.command(f'@S {frame+2}',now)
            r.tick(now);r.drain()
            camera.lookat[:]=r.plant.data.xipos[m.body('cad_base').id];camera.lookat[2]-=.06
            if viewer:
                viewer.cam.lookat[:]=camera.lookat;viewer.sync()
                time.sleep(max(0,start+(frame+1)*.02-time.monotonic()))
            if renderer and frame%2==0:
                renderer.update_scene(r.plant.data,camera=camera,scene_option=opt)
                pixels=renderer.render();writer.stdin.write(pixels.tobytes())
                if frame==600:
                    from PIL import Image
                    Image.fromarray(pixels).save(args.video.with_suffix('.png'))
        print(json.dumps(dict(profile=r.profile,safety=r.safety,pose=r.pose,seconds=float(r.plant.data.time))))
    finally:
        if writer:
            writer.stdin.close()
            if writer.wait()!=0: raise RuntimeError("Video encoding failed")
        if renderer:renderer.close()
        if viewer:viewer.close()

if __name__=='__main__':main()
