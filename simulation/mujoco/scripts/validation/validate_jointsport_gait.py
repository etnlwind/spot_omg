"""Validate deployed Level15, including actual foot clearance."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.tuning.search_raised_level import run

def validate(case):
    profile,scenario,seed=case
    row=run((.015,1.4,.065,.64,60.),profile=profile,scenario=scenario,seed=seed)
    return dict(profile=profile,scenario=scenario,seed=seed,duration_s=60,**row)

if __name__=='__main__':
    cases=[('jointsport','nominal',seed) for seed in (55,77,101)]
    cases += [('jointsport',s,55) for s in ('delay60','heavy_slippery','com_offset')]
    cases += [('jointfast','nominal',55)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(validate,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            (RESULTS_ROOT / 'jointsport_gait_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
    assert all(r['rms_tilt_deg']<1 and r['p10_peak_clearance_mm']>5 for r in rows if r['profile']=='jointsport' and r['scenario']=='nominal')
