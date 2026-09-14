from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulation/mujoco'))
from cad_physics import Simulation
from support_shift import SupportShift
p=json.loads((ROOT/'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text());p['experimental_stow']=True
kin=SupportShift(Simulation(p).model)
results={}
for name in ('highlift32','matched-rear'):
 f=ROOT/f'artifacts/gait-videos/2026-09-14/shoulder-j2-{name}-mass2754g-four-views.json';d=json.loads(f.read_text());rows=[r for r in d['rows'] if 12<=r['time_s']<=20 and r['safety']=='ok'];goals=np.array(d['geometry']['foot_goals_m']);stages={};arrays={}
 for field in ('nominal_deg','command_deg','actual_deg'):
  a=[]
  for r in rows:
   kin.set_angles(r[field]);a.append([kin.foot(i) for i in range(4)])
  a=np.array(a)-goals;arrays[field]=a
  stages[field]={'x_range_mm':(np.ptp(a[:,:,0],axis=0)*1000).tolist(),'z_range_mm':(np.ptp(a[:,:,2],axis=0)*1000).tolist(),'diagonal_error_rms_xz_mm':[np.sqrt(np.mean(((a[:,rear,:]-a[:,front,:])[:,[0,2]]*1000)**2,axis=0)).tolist() for rear,front in ((2,1),(3,0))]}
 stages['balance_displacement_rms_xyz_mm']=np.sqrt(np.mean(((arrays['command_deg']-arrays['nominal_deg'])*1000)**2,axis=0)).tolist()
 stages['tracking_displacement_rms_xyz_mm']=np.sqrt(np.mean(((arrays['actual_deg']-arrays['command_deg'])*1000)**2,axis=0)).tolist()
 stages['roll_rms_deg']=float(np.sqrt(np.mean([r['roll_deg']**2 for r in rows])))
 stages['roll_max_deg']=max(abs(r['roll_deg']) for r in rows)
 stages['ground_clearance_max_mm']=np.max([r['foot_clearance_mm'] for r in rows],axis=0).tolist()
 stages['snapshot13.12']=min(rows,key=lambda r:abs(r['time_s']-13.12))
 results[name]=stages
Path(__file__).with_name('trajectory_stages.json').write_text(json.dumps(results,indent=2))
for name,s in results.items():
 print(name,json.dumps({k:v for k,v in s.items() if k!='snapshot13.12'}))
