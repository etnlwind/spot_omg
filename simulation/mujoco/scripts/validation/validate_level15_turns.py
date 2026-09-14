
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulation.mujoco.scripts.validation.validate_pivot_turns import run
if __name__=='__main__':
    cases=[('level15','nominal',m,d) for m in ('pivot','arc','turn_forward','reverse_arc') for d in (-1,1)]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(run,cases))
    print(json.dumps(rows,indent=2));(RESULTS_ROOT / 'level15_turn_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
