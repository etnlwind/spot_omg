
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import copy,json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulation.mujoco.scripts.tuning.develop_support_v2 import evaluate
ROOT=SIM_ROOT

def trial(case):
    name,cfg=case
    return evaluate(name,cfg,20.02)

def main():
    v1=json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['cushion_support_shift_v1']
    fixed=copy.deepcopy(v1);fixed['support_shift']['lock_j1']=True
    planar=copy.deepcopy(fixed);planar['support_shift']['lateral_m']=0
    noff=copy.deepcopy(planar);noff['support_shift']['feedforward_coefficients']=[[0]*7,[0]*7]
    overlap=copy.deepcopy(noff);overlap['params'][1]=.85
    quick=copy.deepcopy(overlap);quick['params'][0]=2.4
    faster=copy.deepcopy(v1);faster['params'][0]=3.6
    cases=[('v2-fixed-no-lateral',planar),('v2-fixed-no-feedforward',noff),('v2-fixed-long-support',overlap),('v2-fixed-quick',quick),('v2-v1-period36',faster)]
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,cases))
if __name__=='__main__':main()
