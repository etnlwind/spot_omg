"""Controlled V1 ablation. Preserve baseline and record every candidate."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import copy,json
from pathlib import Path
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.scripts.validation.validate_support_shift import Recorder, metrics
ROOT=SIM_ROOT
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def evaluate(name,profile,seconds=20.02):
    pad=json.loads((ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text());rec=Recorder()
    result,rows=run(1000,0,seconds,profile=name,override=profile,cushion=pad,observer=rec)
    result['metrics']=metrics(rec.frames,rows)
    result['first_fault_s']=next((f['time_s'] for f in rec.frames if f['safety']!='ok'),None)
    (OUT/(name+'.json')).write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    print(name,result['safety'],result['first_fault_s'],result['metrics'],flush=True)
    return result

def main():
    v1=json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['cushion_support_shift_v1']
    evaluate('v2-control-v1',v1)
    fixed=copy.deepcopy(v1);fixed['support_shift']['lock_j1']=True
    evaluate('v2-control-j1-only',fixed)
if __name__=='__main__':main()
