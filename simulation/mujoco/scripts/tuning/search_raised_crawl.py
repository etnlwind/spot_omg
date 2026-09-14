
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
    cases=[(.02,p,s,d,18.,.20175,-.035,'crawl') for p in (1.4,1.8,2.2) for s in (.05,.075) for d in (.78,.84)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);(RESULTS_ROOT / 'raised_crawl_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
