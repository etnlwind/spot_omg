
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.analysis.diagnose_right_drift import run, OUT
if __name__=='__main__':
    cases=[(-.6,h,b,seed) for b in (False,True) for h in (False,True) for seed in (55,77)]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,cases))
    print(json.dumps(rows,indent=2));(OUT/'validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
