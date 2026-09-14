"""Evaluate the exact embedded arc/drive/balance C path before installation."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.scripts.validation.validate_support_shift import Recorder, metrics
OUT=REPO_ROOT/'artifacts/upright/2026-09-11/cushion'
def check(yaw,prefix="arcturn-shared-c"):
    pad=json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text());rec=Recorder();headings=[]
    def observe(now,robot):
        rec(now,robot)
        R=robot.plant.data.xmat[robot.plant.model.body("robot").id].reshape(3,3)
        if 5<=now<20:headings.append([now,float(np.arctan2(R[1,0],R[0,0])),robot.yaw])
    r,rows=run(0,yaw,24.02,profile='arcturn',cushion=pad,observer=observe,stop_at=20.)
    r['rotation_deg_s']=float(np.degrees(np.unwrap([h[1] for h in headings])[-1]-np.unwrap([h[1] for h in headings])[0])/(headings[-1][0]-headings[0][0]))
    r['applied_yaw']=headings[-1][2]
    r['metrics']=metrics(rec.frames,rows)
    r['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    name='left' if yaw<0 else 'right'
    (OUT/f'{prefix}-{name}.json').write_text(json.dumps(r,indent=2,default=float))
    print(name,r['safety'],r['metrics'],r['stop_completed'],flush=True)
    return r
if __name__=='__main__':
    import argparse
    from functools import partial
    parser=argparse.ArgumentParser();parser.add_argument('--prefix',default='arcturn-shared-c');args=parser.parse_args()
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(partial(check,prefix=args.prefix),[-1000,1000]))
