import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_wide80_turns import trial,ROOT,OUT

def check(job):
    period,direction=job
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80'])
    cfg['support_shift'].update(turn_path='arc',turn_sweep_rad=.2,turn_lift_m=.02,turn_input_limit=1.,turn_period_s=period)
    return trial(direction,cfg,f'wide80-arc-t{round(period*100)}')

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:r=list(pool.map(check,[(t,d) for t in (.8,1.,1.2) for d in (-1,1)]))
    (OUT/'wide80-arc-cadence-candidates.json').write_text(json.dumps(r,indent=2))
