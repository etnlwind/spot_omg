"""Offline powered-clearance and unpowered-settling regression; no transport."""
import json,numpy as np
from cad_gait import CAD
from cad_physics import Simulation
from virtual_robot import RobotController
from stow_clearance import LegClearance
from stow_clearance import MIN_DYNAMIC_CLEARANCE_M
from pathlib import Path
OUT=Path(__file__).parent/"diagnostics/stow"
OUT.mkdir(parents=True,exist_ok=True)
def main():
    results=[]
    for name,path in [('nominal',CAD/'physics_parameters.json'),('right-drift',CAD.parent/'scenarios/right_drift.json')]:
     p=json.loads(path.read_text());p['experimental_stow']=True
     r=RobotController(Simulation(p));checker=LegClearance(r.plant.model)
     minimum=(float('inf'),None);frames=[];labels=[];release=None;resttorque=0
     r.command('stow',0)
     for i in range(850):
      r.tick(float(r.plant.data.time));frames.append(r.plant.data.qpos.copy());labels.append('fold' if r.torque else 'gravity-settle')
      if r.torque:
       dist,pair=checker.measure(r.plant.data,ceiling=.03)
       if dist<minimum[0]:minimum=(dist,pair)
      elif release is None:release=np.degrees(r.plant.data.qpos[r.plant.q]).tolist()
      if not r.torque:resttorque=max(resttorque,float(np.max(abs(r.plant.data.ctrl))))
     out=r.drain().decode();settled=np.degrees(r.plant.data.qpos[r.plant.q]).tolist()
     r.command('landing',float(r.plant.data.time))
     for i in range(705):
      r.tick(float(r.plant.data.time));frames.append(r.plant.data.qpos.copy());labels.append('unfold')
     result=dict(scenario=name,min_powered_fold_clearance_mm=minimum[0]*1000,closest_pair=minimum[1],release_actual_deg=release,settled_actual_deg=settled,max_settling_motor_torque_nm=resttorque,fold_reply=out,unfold_reply=r.drain().decode(),final_pose=r.pose)
     results.append(result);print(json.dumps(result),flush=True)
     np.savez_compressed(OUT/f'{name}-safe-cycle.npz',qpos=frames,labels=labels)
     assert minimum[0]>=MIN_DYNAMIC_CLEARANCE_M, result
     assert resttorque==0 and release is not None, result
     assert 'OK stow' in out and r.pose=='landing', result
    (OUT/'dynamic-clearance.json').write_text(json.dumps(results,indent=2))


if __name__ == "__main__":
    main()
