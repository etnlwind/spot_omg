
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json,sys
from pathlib import Path
sys.path.insert(0,'simulation/mujoco');from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
root=Path('simulation/mujoco');p=json.load(open(root/'config/upright_profiles.json'))['profiles']['cushion_wbc'];pad=json.load(open(root/'config/foot_cushion_d37p3_l27mm.json'));results=[]
for scenario,seconds in [('nominal',60),('com_offset',30)]:
 s,_=run(1000,0,seconds,profile='cushion_wbc',override=p,cushion=pad,scenario=scenario);results.append(s);Path('artifacts/upright/2026-09-11/cushion/position-wbc-validation.json').write_text(json.dumps(results,indent=2));print(scenario,s['safety'],s['speed_m_s'],s['legs']['FL'],flush=True)
