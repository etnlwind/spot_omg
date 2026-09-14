"""10mm attached cushion sensitivity; contact softness is an estimate, not foam FEM."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
ROOT=SIM_ROOT
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    policy=json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['upright']
    base=json.loads((ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
    results=[]
    for name,friction,softness in [('bare',None,None),('nominal',1.,.02),('firm',.8,.008),('soft_grippy',1.4,.03)]:
        pad=None if name=='bare' else dict(base,friction=[friction,.005,.0001],contact_time_constant_s=softness)
        summary,_=run(1000,0,40,profile='upright',override=policy,cushion=pad)
        summary['case']=name;results.append(summary)
        (OUT/'validation.json').write_text(json.dumps(results,indent=2))
        print(name,summary['safety'],round(summary['speed_m_s'],3),summary['legs'],flush=True)

if __name__=='__main__':main()
