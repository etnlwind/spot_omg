"""Offline powered-clearance and unpowered-settling regression; no transport."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json,numpy as np
from simulation.mujoco.runtime.cad_gait import CAD
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.runtime.stow_clearance import LegClearance
from simulation.mujoco.runtime.stow_clearance import MIN_DYNAMIC_CLEARANCE_M
from pathlib import Path
OUT=SIM_ROOT/"diagnostics/stow"
OUT.mkdir(parents=True,exist_ok=True)
def main():
    results=[]
    for name,path in [('nominal',CAD/'physics_parameters.json'),('right-drift',CAD.parent/'scenarios/right_drift.json')]:
     p=json.loads(path.read_text());p['experimental_stow']=True
     r=RobotController(Simulation(p));checker=LegClearance(r.plant.model)
     minimum=(float('inf'),None);frames=[];labels=[];release=None;resttorque=0
     r.command('stow',0)
     for i in range(1100):
      r.tick(float(r.plant.data.time));frames.append(r.plant.data.qpos.copy());labels.append('fold' if r.torque else 'gravity-settle')
      if r.torque:
       dist,pair=checker.measure(r.plant.data,ceiling=.03)
       if dist<minimum[0]:minimum=(dist,pair)
      elif release is None:release=np.degrees(r.plant.data.qpos[r.plant.q]).tolist()
      if not r.torque:resttorque=max(resttorque,float(np.max(abs(r.plant.data.ctrl))))
     out=r.drain().decode();settled=np.degrees(r.plant.data.qpos[r.plant.q]).tolist()
     r.command('landing',float(r.plant.data.time))
     for i in range(1200):
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
