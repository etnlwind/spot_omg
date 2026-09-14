from pathlib import Path
import sys,json,numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[3];sys.path[:0]=[str(ROOT/'simulation/mujoco'),str(ROOT/'artifacts/gait-videos/2026-09-14')]
from cad_physics import Simulation,foot_clearance
from cad_gait import CAD
from virtual_robot import RobotController
case=sys.argv[1]
p=json.loads((CAD/'physics_parameters_measured_total_2754g.json').read_text());p['timestep_s']=.0005;p['experimental_stow']=True;p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
plant=Simulation(p);r=RobotController(plant);r.select_profile('centerpivot')
from ground_frame_preview import configure
config=json.loads(sys.argv[2]) if len(sys.argv)>2 else {}
geometry=configure(r,config)
m,d=plant.model,plant.data;feet=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')];floor=m.geom('floor').id
records=[];fault=None
for i in range(int(config.get("duration_s",30)/.02)):
 t=i*.02
 if i==500:r.command('drive 1000 0 1',t)
 if 500<i and i%10==0:r.command(f'@D {i} 1000 0',t)
 phase=r.phase
 r.tick(t);r.drain()
 force=np.zeros(4)
 for j,c in enumerate(d.contact):
  if floor not in (c.geom1,c.geom2):continue
  other=c.geom2 if c.geom1==floor else c.geom1
  if other in feet:
   f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f);force[feet.index(other)]+=max(0,f[0])
 row=plant.row();rot=d.xmat[m.body('robot').id].reshape(3,3)
 records.append(dict(t=t,phase=getattr(r,"ground_evaluated_phase",phase),linear=r.linear,roll=row['roll_deg'],pitch=row['pitch_deg'],yaw=float(np.degrees(np.arctan2(rot[1,0],rot[0,0]))),com=row['com_m'],force=force.tolist(),clearance=[foot_clearance(m,d,f)*1000 for f in feet],command=r.command_target.tolist(),actual=np.degrees(d.qpos[plant.q]).tolist(),sat=plant.saturated,voltage=plant.voltage,capture=getattr(r,'ground_nominal_info',{}).get('capture',{}).copy(),ground=getattr(getattr(r,'ground_frame',None),'diagnostic',{}).copy()))
 if r.safety!='ok':fault=dict(t=t,reason=r.safety);break
rs=[x for x in records if x['t']>=12] or records[-1:];arr=lambda k:np.array([x[k] for x in rs]);a=arr('actual');c=arr('command');f=arr('force');ph=(arr('phase')[:,None]+[0,.5,.5,0])%1;duty=float(config.get("duty",.52));u=(ph-duty)/(1-duty);mid=(u>1/6)&(u<5/6)
result=dict(case=case,mass_kg=float(m.body_mass.sum()),fault=fault,roll_max=float(abs(arr('roll')).max()),pitch_max=float(abs(arr('pitch')).max()),roll_rms=float(np.sqrt(np.mean(arr('roll')**2))),yaw_peak_to_peak=float(np.ptp(arr('yaw'))),lateral_peak_to_peak_mm=float(np.ptp(arr('com')[:,1])*1000),mean_voltage=float(arr('voltage').mean()),j1_command_abs_max_deg=float(abs(c[:,::3]).max()),joint_tracking_rms=np.sqrt(np.mean((c-a)**2,axis=0)).reshape(4,3).tolist(),mid_swing_contact_fraction=[float(np.mean(f[mid[:,j],j]>1)) if mid[:,j].any() else None for j in range(4)],mid_swing_peak_clearance_mm=[float(arr('clearance')[mid[:,j],j].max()) if mid[:,j].any() else None for j in range(4)],mean_saturation=float(arr('sat').mean()),forward_speed_m_s=float((arr('com')[-1,0]-arr('com')[0,0])/max(.02,rs[-1]['t']-rs[0]['t'])))
Path(__file__).with_name('ground-'+case+'.json').write_text(json.dumps(dict(summary=result,config=config,geometry=geometry,records=records),indent=2));print(json.dumps(result),flush=True)
