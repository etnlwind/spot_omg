import json
from concurrent.futures import ProcessPoolExecutor
from diagnose_right_drift import run,OUT
if __name__=='__main__':
    cases=[(-.6,h,b,seed) for b in (False,True) for h in (False,True) for seed in (55,77)]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,cases))
    print(json.dumps(rows,indent=2));(OUT/'validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
