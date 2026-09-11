import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_wide80_turns import trial,ROOT,OUT

def check(job):
    stride,lift,direction=job
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80'])
    cfg['support_shift'].update(turn_lift_m=lift,turn_input_limit=1.,turn_stride_m=stride)
    return trial(direction,cfg,f'wide80-turn-s{round(stride*1000)}-h{round(lift*1000)}')

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:r=list(pool.map(check,[(s,h,d) for s in (.03,.04) for h in (.02,.025) for d in (-1,1)]))
    (OUT/'wide80-turn-lift-candidates.json').write_text(json.dumps(r,indent=2))
