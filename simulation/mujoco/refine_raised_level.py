import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from search_raised_level import run
if __name__=='__main__':
    cases=[(h,p,.05,d,18.) for h in (.015,.02) for p in (1.6,1.8) for d in (.6,.64,.7)]
    cases +=[(.02,2.,.065,.64,18.),(.02,1.8,.065,.64,18.),(.012,1.8,.05,.64,18.)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);Path(__file__).with_name('raised_level_refined.json').write_text(json.dumps(rows,indent=2)+'\n')
