"""Repeated full-command start/stop through the deployed control path."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController

def run(seed):
    p,m=physics();p['bno055']={**p.get('bno055',{}),'seed':seed}
    r=RobotController(Simulation(p,m));r.select_profile('jointsport');peak=0.;nonfoot=False;stops=[]
    for i in range(1850):
        t=i*.02
        if i in (100,700,1300):
            stops.append(r.motion is None and r.transition is None)
            r.command(f'drive 0 0 {i}',t)
        if any(start<=i<start+400 for start in (100,700,1300)) and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i in (500,1100,1700):r.command(f'@S {i}',t)
        r.tick(t);row=r.plant.row();r.drain()
        peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));nonfoot |= any(not n.endswith('_foot') for n in row['contacts'])
    return dict(seed=seed,peak_tilt_deg=peak,safety=r.safety,nonfoot_contact=nonfoot,stopped_before_restart=stops,
        passed=r.safety=='ok' and not nonfoot and all(stops) and r.motion is None and r.transition is None and peak<6)
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,(55,77)))
    print(json.dumps(rows,indent=2));(RESULTS_ROOT / 'jointsport_restart_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
