"""Check heading hold against mechanical asymmetry with delayed/noisy IMU only.

Truth coordinates are used by diagnose_right_drift for scoring, never control.
Results include full-path error, not only the last sample of an oscillating gait.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import simulation.mujoco.scripts.analysis.diagnose_right_drift as diagnostic

OUT=(RESULTS_ROOT / 'diagnostics')/'heading-v24'
ROOT=REPO_ROOT


def evaluate(case):
    bias,balance,seed,duration,linear=case
    diagnostic.OUT=OUT/f'{duration}s-{linear}'
    result=diagnostic.run((bias,True,balance,seed),duration_s=duration,command_linear=linear)
    result['controller_sha256']=hashlib.sha256((ROOT/'firmware/stm32-learning/Inc/heading_control.h').read_bytes()).hexdigest()
    return result


if __name__=='__main__':
    cases=[(-.6,b,s,30,1000) for b in (False,True) for s in (55,77)]
    cases += [(bias,b,91,30,1000) for bias in (0.,.6) for b in (False,True)]
    cases += [(-.6,b,91,60,1000) for b in (False,True)]
    cases += [(-.6,b,91,30,600) for b in (False,True)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(evaluate,cases):
            rows.append(row)
            print(json.dumps(row),flush=True)
            OUT.mkdir(parents=True,exist_ok=True)
            (OUT/'validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows), 'Unsafe motion or incomplete stop'
    # Straightness is reported separately from safety. No assertion of zero drift.
