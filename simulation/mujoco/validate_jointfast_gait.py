"""Validate deployed Level15, including actual foot clearance."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from search_raised_level import run

def validate(case):
    profile,scenario,seed=case
    row=run((.015,1.4,.065,.64,60.),profile=profile,scenario=scenario,seed=seed)
    return dict(profile=profile,scenario=scenario,seed=seed,duration_s=60,**row)

if __name__=='__main__':
    cases=[('jointfast','nominal',seed) for seed in (55,77,101)]
    cases += [('jointfast',s,55) for s in ('delay60','heavy_slippery','com_offset')]
    cases += [('joint','nominal',55)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(validate,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('jointfast_gait_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
    assert all(r['rms_tilt_deg']<1 and r['p10_peak_clearance_mm']>5 for r in rows if r['profile']=='jointfast' and r['scenario']=='nominal')
