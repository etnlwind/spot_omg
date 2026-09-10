import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from refine_joint_sport import run
OUT=Path(__file__).with_name('diagnostics')/'joint-sport-refined'
def verify(case):
    candidate,seed=case
    return dict(seed=seed,**run(candidate,duration=60,seed=seed))
if __name__=='__main__':
    candidates=json.loads((OUT/'libraries.json').read_text())
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(verify,[(candidates[i],seed) for i in (6,7) for seed in (55,77,101)]):
            rows.append(row);print(json.dumps(row),flush=True);(OUT/'long_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
