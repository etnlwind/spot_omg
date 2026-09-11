import json,sys
from pathlib import Path
sys.path.insert(0,'simulation/mujoco');from diagnose_turn_clearance import run
root=Path('simulation/mujoco');p=json.load(open(root/'upright_profiles.json'))['profiles']['cushion_wbc'];pad=json.load(open(root/'foot_cushion_10mm.json'));results=[]
for scenario,seconds in [('nominal',60),('com_offset',30)]:
 s,_=run(1000,0,seconds,profile='cushion_wbc',override=p,cushion=pad,scenario=scenario);results.append(s);Path('artifacts/upright/2026-09-11/cushion/position-wbc-validation.json').write_text(json.dumps(results,indent=2));print(scenario,s['safety'],s['speed_m_s'],s['legs']['FL'],flush=True)
