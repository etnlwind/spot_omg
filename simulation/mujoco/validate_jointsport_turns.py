import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from validate_pivot_turns import run
if __name__=='__main__':
    cases=[('jointsport','nominal',m,d) for m in ('pivot','arc','turn_forward','reverse_arc') for d in (-1,1)]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(run,cases))
    print(json.dumps(rows,indent=2));Path(__file__).with_name('jointsport_turn_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows)
