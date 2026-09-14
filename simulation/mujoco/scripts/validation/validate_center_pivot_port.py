"""Replay the deployed centerpivot C kernel in the estimated physical plant."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import sys,json,math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np

from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
OUT=Path('artifacts/audits/pivot-center-2026-09-12')
def test(direction):
 states=[]
 def obs(t,r):
  d,m=r.plant.data,r.plant.model;a=r.plant.row();rot=d.xmat[m.body('robot').id].reshape(3,3)
  states.append([t,*d.xipos[m.body('cad_base').id,:2],math.atan2(rot[1,0],rot[0,0]),a['roll_deg'],a['pitch_deg'],r.safety])
 p=json.loads(Path('simulation/mujoco/config/measured_response_plant.json').read_text());p['timestep_s']=.0005
 summary,feet=run(0,-1000 if direction=='left' else 1000,27,profile='centerpivot',parameter_overrides=p,observer=obs,stop_at=25)
 v=np.array([s[:6] for s in states]);steady=v[(v[:,0]>=5)&(v[:,0]<25)];xy=steady[:,1:3];yaw=np.unwrap(steady[:,3]);active=v[(v[:,0]>=2)&(v[:,0]<25)]
 r=dict(direction=direction,profile='centerpivot',seconds=27,yaw_deg_s=float(np.degrees(yaw[-1]-yaw[0])/(steady[-1,0]-steady[0,0])),center_max_mm=float(np.linalg.norm(xy-xy[0],axis=1).max()*1000),from_start_max_mm=float(np.linalg.norm(active[:,1:3]-v[99,1:3],axis=1).max()*1000),peak_tilt_deg=float(abs(steady[:,4:6]).max()),faults=sorted(set(s[6] for s in states if s[6]!='ok')),stop_ok=summary['safety']=='ok' and not feet[-1]['moving'])
 (OUT/f'c-port-{direction}-states.json').write_text(json.dumps(states));return r
if __name__=='__main__':
 with ProcessPoolExecutor(max_workers=2) as pool:rows=list(pool.map(test,['left','right']))
 (OUT/'c-port-validation.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows),flush=True)
