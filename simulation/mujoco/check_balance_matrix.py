"""Regression matrix for the active BNO055 feedback adapter."""
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from check_profile_runtime import run
from search_gait_profiles import physics


def case(args):
    profile,scenario,motion,delay=args
    p,m=physics(scenario);p['bno055']={'fusion_delay_s':delay}
    row=run(profile,scenario,motion,p,m);row['fusion_delay_ms']=delay*1000
    return row


if __name__=='__main__':
    cases=[(p,s,m,.02) for p in ('legacy','crawl','cruise','trot','highstep')
           for s in ('nominal','heavy_slippery','com_offset')
           for m in ('straight','turn_forward','reverse_forward','arc')]
    cases += [(p,'nominal','turn_forward',.06) for p in ('legacy','crawl','cruise','trot','highstep')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(case,cases):
            rows.append(row)
            print(row['profile'],row['scenario'],row['motion'],row['safety'],row['stopped'],flush=True)
    Path(__file__).with_name('balance_matrix.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in rows)
