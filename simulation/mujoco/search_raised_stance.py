import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from search_raised_level import run
if __name__=='__main__':
    cases=[(.02,p,.05,.64,18.,z,x) for p in (1.2,1.4) for z in (.215,.225,.235) for x in (-.015,-.035,-.05)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);Path(__file__).with_name('raised_stance_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
