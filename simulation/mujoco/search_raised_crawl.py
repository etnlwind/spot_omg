import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from search_raised_level import run
if __name__=='__main__':
    cases=[(.02,p,s,d,18.,.20175,-.035,'crawl') for p in (1.4,1.8,2.2) for s in (.05,.075) for d in (.78,.84)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);Path(__file__).with_name('raised_crawl_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
