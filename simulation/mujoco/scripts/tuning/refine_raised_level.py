
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
if __name__=='__main__':
    cases=[(h,p,.05,d,18.) for h in (.015,.02) for p in (1.6,1.8) for d in (.6,.64,.7)]
    cases +=[(.02,2.,.065,.64,18.),(.02,1.8,.065,.64,18.),(.012,1.8,.05,.64,18.)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);(RESULTS_ROOT / 'raised_level_refined.json').write_text(json.dumps(rows,indent=2)+'\n')
