
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import copy,json
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.validation.check_wide80_turns import trial, ROOT, OUT

def check(job):
    ramp,direction=job;period=1.2
    cfg=copy.deepcopy(json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80'])
    cfg['support_shift'].update(turn_path='arc',turn_sweep_rad=.2,turn_lift_m=.02,turn_input_limit=1.,turn_period_s=period,turn_lift_ramp=ramp)
    return trial(direction,cfg,f'wide80-arc-hold{round(ramp*100)}')

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:r=list(pool.map(check,[(r,d) for r in (.35,.45) for d in (-1,1)]))
    (OUT/'wide80-arc-hold-candidates.json').write_text(json.dumps(r,indent=2))
