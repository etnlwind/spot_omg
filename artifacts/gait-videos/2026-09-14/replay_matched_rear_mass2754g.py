from pathlib import Path
import sys,json,time,subprocess
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'simulation/mujoco'))
import mujoco,mujoco.viewer
import numpy as np
from PIL import Image, ImageDraw
from cad_physics import Simulation, foot_clearance
from cad_gait import CAD
from virtual_robot import RobotController
p=json.loads((CAD/'physics_parameters_measured_total_2754g.json').read_text());p['timestep_s']=.0005;p['experimental_stow']=True
p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
plant=Simulation(p);r=RobotController(plant);r.select_profile('centerpivot')
from shoulder_j2_matched_rear_preview import configure
geometry=configure(r);print('GEOMETRY',geometry,flush=True)
out=Path(__file__).with_name('shoulder-j2-matched-rear-mass2754g-four-views.mp4')
assert abs(float(plant.model.body_mass.sum())-2.754)<1e-6
print('TOTAL MASS',float(plant.model.body_mass.sum()),flush=True)
m=plant.model;m.vis.global_.offwidth=960;m.vis.global_.offheight=540
renderer=mujoco.Renderer(m,height=540,width=960)
cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);cam.azimuth=90;cam.elevation=0;cam.distance=1.10;cam.orthographic=1
top=mujoco.MjvCamera();mujoco.mjv_defaultCamera(top);top.azimuth=90;top.elevation=-90;top.distance=1.10;top.orthographic=1
rear=mujoco.MjvCamera();mujoco.mjv_defaultCamera(rear);rear.azimuth=0;rear.elevation=0;rear.distance=.90;rear.orthographic=1
quarter=mujoco.MjvCamera();mujoco.mjv_defaultCamera(quarter);quarter.azimuth=135;quarter.elevation=-22;quarter.distance=1.15
opt=mujoco.MjvOption();opt.geomgroup[3]=0
ff=subprocess.Popen(['/opt/homebrew/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','1920x1080','-framerate','25','-i','-','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
legs=('fl','fr','rl','rr')
def alignment():
 return [(plant.data.geom_xpos[m.geom(l+'_foot').id,0]-plant.data.xanchor[m.joint(l+'_j2').id,0])*1000 for l in legs]
def guides(scene):
 for l in ('fl','rl'):
  a=plant.data.xanchor[m.joint(l+'_j2').id].copy();a[1]+=.045
  b=a.copy();b[2]=0
  g=scene.geoms[scene.ngeom]
  mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([1.,.15,.1,1.],dtype=np.float32))
  mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,2.,a,b);scene.ngeom+=1
# Body geometric midpoint: midpoint of the four J2 shoulder anchors, not COM.
def body_center():
 return np.mean([plant.data.xanchor[m.joint(l+'_j2').id] for l in legs],axis=0)
def body_heading():
 front=np.mean([plant.data.xanchor[m.joint(l+'_j2').id] for l in legs[:2]],axis=0)
 back=np.mean([plant.data.xanchor[m.joint(l+'_j2').id] for l in legs[2:]],axis=0)
 d=front-back;d[2]=0;return d/np.linalg.norm(d)
initial_center=body_center().copy();initial_direction=body_heading().copy()
lateral=np.array([-initial_direction[1],initial_direction[0],0.])
initial_top=initial_center.copy();overlay_z=initial_center[2]+.25
def top_guides(scene):
 a=initial_center-10*initial_direction;b=initial_center+10*initial_direction
 a[2]=b[2]=overlay_z
 g=scene.geoms[scene.ngeom]
 mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([0.,.8,1.,1.],dtype=np.float32))
 mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,3.,a,b);scene.ngeom+=1
 point=body_center();point[2]=overlay_z+.01
 g=scene.geoms[scene.ngeom]
 mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.009,.009,.009]),point,np.eye(3).ravel(),np.array([1.,.05,.6,1.],dtype=np.float32));scene.ngeom+=1
rows=[];start=time.monotonic()
with mujoco.viewer.launch_passive(m,plant.data) as v:
 for i in range(1500):
  t=i*.02
  if i==500:r.command('drive 1000 0 1',t)
  if 500<i<1500 and i%10==0:r.command(f'@D {i} 1000 0',t)
  # Keep forward through the last recorded frame.
  r.tick(t);row=plant.row();rows.append(dict(time_s=t,roll_deg=float(row['roll_deg']),pitch_deg=float(row['pitch_deg']),safety=r.safety,foot_clearance_mm=[foot_clearance(m,plant.data,m.geom(l+'_foot').id)*1000 for l in legs],body_center_m=body_center().tolist(),lateral_mm=float((body_center()-initial_center)@lateral*1000),yaw_error_deg=float(np.degrees(np.arctan2(body_heading()@lateral,body_heading()@initial_direction))),j2_foot_dx_mm=alignment(),nominal_deg=r.target.tolist(),command_deg=r.command_target.tolist(),actual_deg=np.degrees(plant.data.qpos[plant.q]).tolist()))
  messages=r.drain()
  if messages:print(t,messages,flush=True)
  cam.lookat[:]=row['com_m'];cam.lookat[2]-=.03
  # Follow forward travel only: fixed lateral coordinate and fixed camera heading.
  top.lookat[:]=initial_top+initial_direction*((body_center()-initial_center)@initial_direction)
  rear.lookat[:]=cam.lookat;quarter.lookat[:]=cam.lookat
  if i%2==0:
   if i<1500:
    renderer.update_scene(plant.data,camera=cam,scene_option=opt);guides(renderer.scene);side=renderer.render().copy()
    renderer.update_scene(plant.data,camera=top,scene_option=opt);top_guides(renderer.scene);above=renderer.render().copy()
    renderer.update_scene(plant.data,camera=rear,scene_option=opt);back=renderer.render().copy()
    renderer.update_scene(plant.data,camera=quarter,scene_option=opt);threequarter=renderer.render().copy()
    frame=Image.fromarray(np.concatenate((np.concatenate((side,above),axis=1),np.concatenate((back,threequarter),axis=1)),axis=0));draw=ImageDraw.Draw(frame)
    for x,y,label in [(12,0,'SIDE'),(972,0,'TOP'),(12,540,'REAR'),(972,540,'QUARTER')]:
     draw.rectangle((x-4,y+8,x+600,y+65),fill='black');draw.text((x,y+17),f'{label} | {t:05.2f}s | '+('IDLE' if t<10 else 'FORWARD 100% / MATCHED REAR 32mm'),fill='white')
    draw.text((650,17),f'MASS 2.754kg | {r.safety}',fill='white')
    draw.text((12,42),'J2 vertical guides | actual foot dx mm: '+', '.join(f'{v:+.1f}' for v in alignment()),fill='white')
    draw.text((972,42),f'CYAN: initial axis | PINK: body center | lateral {rows[-1]["lateral_mm"]:+.1f}mm | yaw {rows[-1]["yaw_error_deg"]:+.1f}deg',fill='white')
    ff.stdin.write(np.asarray(frame).tobytes())
   if v.is_running():
    v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance;v.cam.azimuth=cam.azimuth;v.cam.elevation=cam.elevation;v.cam.orthographic=1;v.opt.geomgroup[3]=0;v.sync()
  if i%500==0:print('TIME',t,'SAFETY',r.safety,flush=True)
  delay=start+t-time.monotonic()
  if delay>0:time.sleep(delay)
 ff.stdin.close();assert ff.wait()==0
 out.with_suffix('.json').write_text(json.dumps(dict(initial_center_m=initial_center.tolist(),initial_direction=initial_direction.tolist(),geometry=geometry,profile='centerpivot-shoulder-aligned-experiment',idle_s=10,forward_s=20,linear=1,yaw=0,recording_s=30,physics='measured total mass; estimated distribution and dynamics',total_mass_kg=float(m.body_mass.sum()),rows=rows),indent=2))
 print('PLAYBACK COMPLETE',out,flush=True)
renderer.close()
