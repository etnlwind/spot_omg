"""Shorten four-foot overlap while keeping the 720ms swing duration."""
import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,ROOT,OUT

def evaluate(gap):
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift_v2'])
    swing=.72;period=2*(swing+gap);duty=1-swing/period
    cfg['params'][0]=period;cfg['params'][1]=duty
    name=f'continuous-gap{round(gap*1000)}'
    result=trial((name,cfg,20.02))
    result.update(swing_seconds=swing,overlap_seconds=gap,step_interval_seconds=period/2)
    (OUT/f'{name}.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    print('TIMING',gap,result['safety'],result['first_fault_s'],result['speed_m_s'],flush=True)
    return result
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(evaluate,[0.,.12,.3,.6,.9,1.2]))
