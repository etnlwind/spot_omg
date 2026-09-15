"""Audit larger rear endpoints against V622 joint travel; no hardware commands."""
import sys,json,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import numpy as np
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
p=Path('artifacts/s-native-v6-2-3');p.mkdir(exist_ok=True)
plant=Simulation(load_parameters(parse_args([])))
rows=[]
for rear in [.125,.130,.135,.140]:
 profile=copy.deepcopy(PROFILES['s_native_v6_2_2'])
 profile['rear_extension_m']=rear-.020;profile['params'][2]=rear+.020
 gait=SNativeGait(plant.model,plant.stand_target,profile)
 try:
  gait.prepare_support(1.,0.)
  qs=[]
  for phase in np.arange(250)/100:
   q=gait.targets((.5+phase)%1,1.,1.,0.)
   if phase>=1.5:qs.append(q.copy())
  qs=np.array(qs);path=np.abs(np.diff(np.vstack([qs,qs[:1]]),axis=0)).sum(axis=0)
  rate=np.max(np.abs(np.diff(np.vstack([qs,qs[:1]]),axis=0)),axis=0)/.005
  row=dict(rear_m=rear,reachable=True,joint_min_deg=qs.min(axis=0).tolist(),joint_max_deg=qs.max(axis=0).tolist(),travel_deg=path.tolist(),max_rate_at_half_second_deg_s=rate.tolist())
 except ValueError as e:row=dict(rear_m=rear,reachable=False,error=str(e))
 rows.append(row);print(json.dumps(row),flush=True)
(p/'reach-candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
