import sys,json,csv
from pathlib import Path
import numpy as np
sys.path.insert(0,'simulation/mujoco')
from diagnose_turn_clearance import run
out=Path('artifacts/audits/turn-cause-2026-09-12')
pad=json.load(open('simulation/mujoco/foot_cushion_10mm.json'))
base=json.load(open('simulation/mujoco/cad_300mm/physics_parameters.json'))['mass_kg']
for mass in [4.418,6.0]:
 for yaw in [-1000,1000]:
  frames=[]
  def observe(t,r):
   p=r.plant;s=p.row()
   frames.append(dict(t=t,actual_deg=s['actual_deg'],command_deg=np.degrees(p.desired).tolist(),filtered_deg=np.degrees(p.filtered).tolist(),torque_nm=s['torque_nm'],limits_nm=p.limits.tolist(),roll=s['roll_deg'],pitch=s['pitch_deg'],voltage=s['voltage_v'],phase=r.phase,safety=r.safety,mass=float(p.model.body_mass.sum())))
  params={'tracking_feedback_enabled':False}
  if mass==6:
   m=base.copy();m['chassis_without_battery_or_leg_groups']=2.6
   for j in range(1,4):m[f'j{j}_group']*=2.78/1.998
   params['mass_kg']=m
  summary,rows=run(0,yaw,20,profile='arcsupport',cushion=pad,observer=observe,parameter_overrides=params)
  f=[x for x in frames if 5<=x['t']<19.5];a=np.array([x['actual_deg'] for x in f]);q=np.array([x['command_deg'] for x in f]);torque=np.abs([x['torque_nm'] for x in f]);lim=np.array([x['limits_nm'] for x in f]);dt=.02
  summary['diagnosis']={'mass_kg':frames[0]['mass'],'max_roll':max(abs(x['roll']) for x in f),'max_pitch':max(abs(x['pitch']) for x in f),'voltage_min':min(x['voltage'] for x in f),'tracking_rms_deg':np.sqrt(np.mean((q-a)**2,axis=0)).reshape(4,3).tolist(),'max_torque_nm':np.max(torque,axis=0).reshape(4,3).tolist(),'torque_limit_fraction':np.mean(torque>=lim*.995,axis=0).reshape(4,3).tolist(),'target_speed_max_deg_s':np.max(abs(np.diff(q,axis=0)/dt),axis=0).reshape(4,3).tolist()}
  name=f'mass{mass}_yaw{yaw}'
  (out/(name+'.json')).write_text(json.dumps(summary,indent=2,default=float))
  (out/(name+'-frames.json')).write_text(json.dumps(frames,default=float))
  with open(out/(name+'.csv'),'w') as fp:
   w=csv.DictWriter(fp,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
  print(name,json.dumps(summary['diagnosis'],default=float),flush=True)
