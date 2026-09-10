"""Deterministic offscreen MuJoCo capture, 25 fps at 1x simulation time."""
import argparse,json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def record(name,params,output,profile=None):
    p,m=physics();r=RobotController(Simulation(p,m));r.select_profile(profile or 'level')
    if profile is None:r.profiles['level']['params']=params
    else:params=r.profiles[profile]['params']
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    m.vis.global_.offwidth=960;m.vis.global_.offheight=540
    renderer=mujoco.Renderer(m,height=540,width=960)
    camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
    camera.azimuth=135;camera.elevation=-15;camera.distance=1.25
    option=mujoco.MjvOption();option.geomgroup[3]=0
    label=output.with_suffix('.label.txt');label.write_text(f'{name} | command lift {params[3]*1000:.0f}mm | period {params[0]:.2f}s | 1x\n100% forward: 2-20s; stop: 20s\nEstimated physics / delayed BNO055 / quantized servos')
    ffmpeg=subprocess.Popen(['/opt/homebrew/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','960x540','-framerate','25','-i','-','-vf',f'drawtext=expansion=none:textfile={label.resolve()}:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.65:x=12:y=12','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    frames=[];chassis=m.body('cad_base').id;feet=[m.geom(n+'_foot').id for n in ('fl','fr','rl','rr')]
    try:
        for i in range(1150):
            t=i*.02
            if i==100:r.command('drive 0 0 1',t)
            if 100<=i<1000 and i%10==0:r.command(f'@D {i} 1000 0',t)
            if i==1000:r.command('@S 99999',t)
            r.tick(t);row=r.plant.row();r.drain()
            frames.append(dict(time_s=t,roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],clearance_mm=[float((r.plant.data.geom_xpos[g,2]-m.geom_size[g,0])*1000) for g in feet],safety=r.safety))
            if i%2==0:
                camera.lookat[:]=r.plant.data.xipos[chassis];camera.lookat[2]-=.03
                renderer.update_scene(r.plant.data,camera=camera,scene_option=option)
                ffmpeg.stdin.write(renderer.render().tobytes())
        ffmpeg.stdin.close();code=ffmpeg.wait();assert code==0
    finally:
        renderer.close()
        if ffmpeg.poll() is None:
            if ffmpeg.stdin and not ffmpeg.stdin.closed:ffmpeg.stdin.close()
            ffmpeg.terminate()
            try:ffmpeg.wait(timeout=3)
            except subprocess.TimeoutExpired:ffmpeg.kill();ffmpeg.wait()
    output.with_suffix('.json').write_text(json.dumps(dict(name=name,params=params,fps=25,playback_speed=1,duration_s=23,frames=frames),indent=2)+'\n')
    print(output,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--all',action='store_true');parser.add_argument('--profile');parser.add_argument('--output',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]/'artifacts/gait-videos/2026-09-10'
    if args.profile:
        record(args.profile,None,args.output or root/('final-'+args.profile+'.mp4'),profile=args.profile)
        raise SystemExit(0)
    cases=[('baseline-level-7mm',[1.,.64,.065,.007,.20175,-.035,.75]),
           ('candidate-12mm',[1.4,.64,.05,.012,.20175,-.035,.75]),
           ('candidate-15mm',[1.4,.64,.05,.015,.20175,-.035,.75]),
           ('candidate-20mm',[1.4,.64,.05,.02,.20175,-.035,.75])]
    for name,p in cases:record(name,p,root/(name+'.mp4'))
