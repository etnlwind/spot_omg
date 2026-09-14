from pathlib import Path
import sys,json,numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'simulation/mujoco'))
from cad_physics import Simulation
from support_shift import SupportShift
p=json.loads((ROOT/'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text());p['experimental_stow']=True
kin=SupportShift(Simulation(p).model)
for filename in sys.argv[1:]:
 d=json.loads(Path(filename).read_text());rows=[r for r in d['rows'] if 12<=r['time_s']<=20 and r['safety']=='ok'];out={}
 for field in ('command_deg','actual_deg'):
  points=[]
  for r in rows:
   kin.set_angles(r[field]);points.append([kin.foot(i) for i in range(4)])
  a=np.array(points);out[field+'_x_range_mm']=np.ptp(a[:,:,0],axis=0).tolist();out[field+'_x_range_mm']=[v*1000 for v in out[field+'_x_range_mm']]
 out['first_fault']=next((r['time_s'] for r in d['rows'] if r['safety']!='ok'),None)
 out['geometry']=d['geometry'];print(Path(filename).name,json.dumps(out))
