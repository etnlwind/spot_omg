import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,ROOT
if __name__=='__main__':
    profiles=json.loads((ROOT/'upright_profiles.json').read_text())['profiles'];jobs=[]
    for key,short in [('cushion_support_shift_v2','v2'),('cushion_v2_push','push')]:
        for gain in (.6,1.):
            cfg=copy.deepcopy(profiles[key]);cfg['support_shift'].update(constant_body_height=True,height_feedback_gain=gain)
            jobs.append((f'height-{short}-feedback{gain}',cfg,20.02))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,jobs))
