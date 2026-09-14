from pathlib import Path
import sys,json,time,subprocess
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'simulation/mujoco'))
import mujoco,mujoco.viewer
import numpy as np
from PIL import Image, ImageDraw
from cad_physics import Simulation
from cad_gait import CAD
from virtual_robot import RobotController
p=json.loads((CAD/'physics_parameters.json').read_text());p['timestep_s']=.0005;p['experimental_stow']=True
p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
plant=Simulation(p);r=RobotController(plant);r.select_profile('centerpivot')
from shoulder_aligned_preview import configure
geometry=configure(r);print('GEOMETRY',geometry,flush=True)
out=Path(__file__).with_name('shoulder-aligned-side-top-idle10-forward20.mp4')
m=plant.model;m.vis.global_.offwidth=960;m.vis.global_.offheight=540
renderer=mujoco.Renderer(m,height=540,width=960)
cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);cam.azimuth=90;cam.elevation=0;cam.distance=1.10
top=mujoco.MjvCamera();mujoco.mjv_defaultCamera(top);top.azimuth=90;top.elevation=-90;top.distance=1.10
opt=mujoco.MjvOption();opt.geomgroup[3]=0
ff=subprocess.Popen(['/opt/homebrew/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','1920x540','-framerate','25','-i','-','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
rows=[];start=time.monotonic()
with mujoco.viewer.launch_passive(m,plant.data) as v:
 for i in range(1500):
  t=i*.02
  if i==500:r.command('drive 1000 0 1',t)
  if 500<i<1500 and i%10==0:r.command(f'@D {i} 1000 0',t)
  # Keep forward through the last recorded frame.
  r.tick(t);row=plant.row();rows.append(dict(time_s=t,roll_deg=float(row['roll_deg']),pitch_deg=float(row['pitch_deg']),safety=r.safety,nominal_deg=r.target.tolist(),command_deg=r.command_target.tolist(),actual_deg=np.degrees(plant.data.qpos[plant.q]).tolist()))
  messages=r.drain()
  if messages:print(t,messages,flush=True)
  cam.lookat[:]=row['com_m'];cam.lookat[2]-=.03
  top.lookat[:]=row['com_m']
  if i%2==0:
   if i<1500:
    renderer.update_scene(plant.data,camera=cam,scene_option=opt);side=renderer.render().copy()
    renderer.update_scene(plant.data,camera=top,scene_option=opt);above=renderer.render().copy()
    frame=Image.fromarray(np.concatenate((side,above),axis=1));draw=ImageDraw.Draw(frame)
    for x,label in [(12,'SIDE'),(972,'TOP')]:
     draw.rectangle((x-4,8,x+360,44),fill='black');draw.text((x,17),f'{label} | {t:05.2f}s | '+('IDLE' if t<10 else 'FORWARD 100%'),fill='white')
    ff.stdin.write(np.asarray(frame).tobytes())
   if v.is_running():
    v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance;v.cam.azimuth=cam.azimuth;v.cam.elevation=cam.elevation;v.opt.geomgroup[3]=0;v.sync()
  if i%500==0:print('TIME',t,'SAFETY',r.safety,flush=True)
  delay=start+t-time.monotonic()
  if delay>0:time.sleep(delay)
 ff.stdin.close();assert ff.wait()==0
 out.with_suffix('.json').write_text(json.dumps(dict(geometry=geometry,profile='centerpivot-shoulder-aligned-experiment',idle_s=10,forward_s=20,linear=1,yaw=0,recording_s=30,physics='estimated',rows=rows),indent=2))
 print('PLAYBACK COMPLETE',out,flush=True)
renderer.close()
