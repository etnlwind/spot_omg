"""Validate the faster smooth cruise on the exact virtual controller."""
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from validate_high_speed_balance import run_case

PARAMS=[.8,.64,.065,.007,.20175,-.035,.75]


def run(case):
    return run_case(case,PARAMS,[1.05,.6,.08,.012,.20175,-.035,.75])


if __name__=='__main__':
    cases=[('cruise','nominal',m,.02) for m in ('straight','turn_forward','reverse','arc','sweep')]
    cases += [('cruise',s,m,d) for s,d in (('heavy_slippery',.02),('com_offset',.02),('nominal',.06))
              for m in ('straight','turn_forward','reverse','arc')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(run,cases):
            rows.append(r);print(json.dumps(r),flush=True)
            Path(__file__).with_name('smooth_cruise_directional_validation.json').write_text(json.dumps(dict(params=PARAMS,cases=rows),indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in rows)
