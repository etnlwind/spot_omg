"""Bounded free-body experiment; does not modify live profiles."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import json
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.validation.check_body_height import trial, OUT

def evaluate(gain):
    cfg=json.loads((OUT/'diagonal-lift20-lead40.json').read_text())['profile']
    cfg['support_shift']['load_preload_gain']=gain
    return trial((f'diagonal-preload-{gain:g}',cfg,20.02))

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(evaluate,[.5,1.]))
