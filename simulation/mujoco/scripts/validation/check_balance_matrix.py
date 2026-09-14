"""Regression matrix for the active BNO055 feedback adapter."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulation.mujoco.scripts.validation.check_profile_runtime import run
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics


def case(args):
    profile,scenario,motion,delay=args
    p,m=physics(scenario);p['bno055']={'fusion_delay_s':delay}
    row=run(profile,scenario,motion,p,m);row['fusion_delay_ms']=delay*1000
    return row


if __name__=='__main__':
    cases=[(p,s,m,.02) for p in ('legacy','crawl','cruise','trot','highstep')
           for s in ('nominal','heavy_slippery','com_offset')
           for m in ('straight','turn_forward','reverse_forward','arc')]
    cases += [(p,'nominal','turn_forward',.06) for p in ('legacy','crawl','cruise','trot','highstep')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(case,cases):
            rows.append(row)
            print(row['profile'],row['scenario'],row['motion'],row['safety'],row['stopped'],flush=True)
    (RESULTS_ROOT / 'balance_matrix.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in rows)
