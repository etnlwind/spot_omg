"""Strict alternating diagonal schedule: no planned four-foot waiting."""
import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,ROOT,OUT

def evaluate(job):
    stride,height=job
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift_v2'])
    cfg['params'][0]=1.44;cfg['params'][1]=.5;cfg['params'][2]=stride;cfg['params'][4]=height
    return trial((f'diagonal-s{round(stride*1000)}-h{round(height*1000)}',cfg,16.02))
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(evaluate,[(s,h) for s in (.06,.10) for h in (.20175,.22,.24)]))
