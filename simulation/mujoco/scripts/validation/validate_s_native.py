"""Measure actual diagonal contact timing and exchange pauses on the estimated plant.

Run from repository root.
A primary swing is the airborne interval with the greatest height integral;
all other airborne episodes remain in the report, including brief bounces.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json,numpy as np
from pathlib import Path

def runs(mask):
 edges=np.diff(np.r_[False,mask,False].astype(int));return list(zip(np.where(edges==1)[0],np.where(edges==-1)[0]))
def analyze(path):
 a=np.loadtxt(path,delimiter=',');w=a[(a[:,0]>=6)&(a[:,0]<28)];out=[]
 for ia,ib,offset in [(0,3,0),(1,2,.5)]:
  ids=np.floor(w[:,1]-offset).astype(int);steps=[]
  for cid in np.unique(ids):
   v=w[ids==cid];f=v[:,1]-offset-cid;v=v[(f>=0)&(f<.5)];f=f[(f>=0)&(f<.5)]
   if len(f)<4 or f[0]>.025 or f[-1]<.475:continue
   main=[];extra=[]
   for leg in (ia,ib):
    z=v[:,2+leg];episodes=runs(z>1)
    if not episodes:main.append(None);extra.append([]);continue
    best=max(episodes,key=lambda t:np.sum(z[t[0]:t[1]]))
    main.append((float(v[best[0],0]),float(v[best[1]-1,0]+.02),float(z[best[0]:best[1]].max())))
    extra.append([dict(duration_ms=int((e-s)*20),peak_mm=float(z[s:e].max())) for s,e in episodes if (s,e)!=best])
   if any(m is None or m[2]<5 for m in main):steps.append(dict(missed=True));continue
   steps.append(dict(liftoff_ms=round(abs(main[0][0]-main[1][0])*1000),touchdown_ms=round(abs(main[0][1]-main[1][1])*1000),peaks_mm=[m[2] for m in main],extra_airborne=extra))
  valid=[s for s in steps if not s.get('missed')]
  out.append(dict(pair=['FL/RR','FR/RL'][len(out)],missed=sum(s.get('missed',False) for s in steps),max_liftoff_ms=max((s['liftoff_ms'] for s in valid),default=None),max_touchdown_ms=max((s['touchdown_ms'] for s in valid),default=None),median_touchdown_ms=float(np.median([s['touchdown_ms'] for s in valid])) if valid else None,steps=steps))
 paused=[(e-s)*20 for s,e in runs(np.all(w[:,2:6]<=1,axis=1)) if s>0 and e<len(w)]
 return dict(pairs=out,all_ground_median_ms=float(np.median(paused)) if paused else 0.,all_ground_max_ms=int(max(paused,default=0)),peak_tilt_deg=float(abs(a[:,-3:-1]).max()))


def main():
 import argparse,copy
 from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
 from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
 from simulation.mujoco.runtime.standing_pose import ContactKinematics, SoleKinematics
 from simulation.mujoco.runtime.s_native_gait import NAME, PROFILES
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--profile',choices=list(PROFILES),default=NAME)
 parser.add_argument('--command',type=int,default=600)
 parser.add_argument('--period',type=float)
 parser.add_argument('--hold',type=float,help='Total hold at each exchange, at full input (seconds)')
 parser.add_argument('--output',type=Path,default=None)
 parser.add_argument('--gif',action='store_true')
 parser.add_argument('--allow-fall',action='store_true')
 args=parser.parse_args()
 if args.output is None:args.output=Path('artifacts')/args.profile.replace('_','-')/'transition-current'
 PROFILE=PROFILES[args.profile]
 if not 1<=args.command<=1000:parser.error('command must be 1..1000')
 if args.period is not None:PROFILE['params'][0]=args.period
 if args.hold is not None:PROFILE['transfer_fraction']=args.hold/(2*PROFILE['params'][0])
 PROFILE['params'][1]=.5+2*PROFILE['transfer_fraction']
 if not 0<=PROFILE['transfer_fraction']<.25:parser.error('hold must be shorter than half a cycle')
 plant=Simulation(load_parameters(parse_args(['--allow-fall'] if args.allow_fall else [])));robot=RobotController(plant);robot.profile=args.profile;kin=ContactKinematics(plant.model)
 sole=SoleKinematics(plant.model,plant.stand_target);sole.set_angles(plant.stand_target)
 origin=np.array([sole.foot(i) for i in range(4)]);reach_rows=[]
 rows=[];phase=0.;previous=None;errors=[];frames=[];renderer=None
 if args.gif:
  import mujoco
  from PIL import Image,ImageDraw
  plant.model.vis.global_.offwidth=640;plant.model.vis.global_.offheight=400
  renderer=mujoco.Renderer(plant.model,height=400,width=640)
  camera=mujoco.MjvCamera();camera.azimuth=90;camera.elevation=0;camera.distance=1.05
  option=mujoco.MjvOption();option.geomgroup[3]=0
 try:
  for i in range(1600):
   t=i*.02
   if i==100:robot.command(f'drive {args.command} 0 1',t)
   elif 100<i<1450 and i%10==0:robot.command(f'@D {i} {args.command} 0',t)
   if i==1450:robot.command('@S 2000',t)
   robot.tick(t)
   if robot.motion and not robot.transition:
    current=robot.nominal_phase
    phase=phase+(current-previous)%1 if previous is not None else current
    previous=current
   state=plant.row()
   sole.set_angles(np.degrees(plant.data.qpos[plant.q]))
   reach_rows.append([t,phase,*[(sole.foot(leg)[0]-origin[leg,0])*1000 for leg in range(4)]])
   rows.append([t,phase,*[foot_clearance(plant.model,plant.data,f)*1000 for f in kin.feet],state['roll_deg'],state['pitch_deg'],plant.data.qpos[0]])
   reply=robot.drain().decode()
   if 'ERROR' in reply:errors.append(reply)
   if renderer and 6<=t<10 and i%2==0:
    camera.lookat[:]=np.asarray(state['com_m'])-[0,0,.1]
    renderer.update_scene(plant.data,camera=camera,scene_option=option)
    frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
    draw.rectangle((0,0,640,28),fill='#17232f')
    draw.text((10,8),f"Period {PROFILE['params'][0]:.1f}s | exchange hold {2*PROFILE['params'][0]*PROFILE['transfer_fraction']:.2f}s | input {args.command/10:.0f}%",fill='white')
    frames.append(frame)
 finally:
  if renderer:renderer.close()
 args.output.parent.mkdir(parents=True,exist_ok=True)
 csv=args.output.with_suffix('.csv');a=np.array(rows)
 np.savetxt(csv,a,delimiter=',',header='time,phase,fl_z_mm,fr_z_mm,rl_z_mm,rr_z_mm,roll,pitch,x')
 reach=np.array(reach_rows);window=reach[(reach[:,0]>=6)&(reach[:,0]<28),2:]
 np.savetxt(args.output.with_name(args.output.name+'-reach').with_suffix('.csv'),reach,delimiter=',',header='time,phase,fl_x_from_s_mm,fr_x_from_s_mm,rl_x_from_s_mm,rr_x_from_s_mm')
 report=analyze(csv)
 report['actual_body_relative_sole_x_mm']={leg:dict(min=float(window[:,i].min()),max=float(window[:,i].max())) for i,leg in enumerate(('FL','FR','RL','RR'))}
 report.update(allow_fall=args.allow_fall,command=args.command,profile=copy.deepcopy(PROFILE),safety=robot.safety,stopped=robot.motion is None and robot.transition is None,finite=bool(np.isfinite(a).all()),errors=errors,
  speed_m_s=float((a[1399,-1]-a[300,-1])/(a[1399,0]-a[300,0])),
  note='Estimated free-body physics, 20ms samples, 6..28s analysis; individual timing metrics do not establish complete gait or hardware validation.')
 args.output.with_suffix('.json').write_text(json.dumps(report,indent=2))
 if frames:frames[0].save(args.output.with_suffix('.gif'),save_all=True,append_images=frames[1:],duration=40,loop=0)
 print(json.dumps({**report,'pairs':[{k:v for k,v in pair.items() if k!='steps'} for pair in report['pairs']]},indent=2))

if __name__=='__main__':main()
