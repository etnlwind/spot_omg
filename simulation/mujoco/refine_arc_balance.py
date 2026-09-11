import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_wide80_turns import trial,ROOT,OUT

def check(job):
    bg,sg,direction=job
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80'])
    cfg['support_shift'].update(turn_path='arc',turn_sweep_rad=.2,turn_lift_m=.02,turn_input_limit=1.,turn_balance_gain=bg,turn_swing_gain=sg)
    return trial(direction,cfg,f'wide80-arc-b{round(bg*100)}-s{round(sg*100)}')

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:r=list(pool.map(check,[(b,s,d) for b in (.15,.35) for s in (.5,1.) for d in (-1,1)]))
    (OUT/'wide80-arc-balance-candidates.json').write_text(json.dumps(r,indent=2))
