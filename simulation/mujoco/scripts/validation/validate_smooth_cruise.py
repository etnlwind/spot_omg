"""Validate the deployed cruise on the exact shared-C virtual controller."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulation.mujoco.scripts.validation.validate_high_speed_balance import run_case
from simulation.mujoco.runtime.gait_profiles import load_profiles

PARAMS=load_profiles()['cruise']['params']


def run(case):
    return run_case(case)


if __name__=='__main__':
    cases=[('cruise','nominal',m,.02) for m in ('straight','turn_forward','reverse','arc','sweep')]
    cases += [('cruise',s,m,d) for s,d in (('heavy_slippery',.02),('com_offset',.02),('nominal',.06))
              for m in ('straight','turn_forward','reverse','arc')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(run,cases):
            rows.append(r);print(json.dumps(r),flush=True)
            (RESULTS_ROOT / 'cruise_symmetric_validation.json').write_text(json.dumps(dict(params=PARAMS,cases=rows),indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in rows)
