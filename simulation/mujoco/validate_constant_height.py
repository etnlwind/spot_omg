import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,ROOT
if __name__=='__main__':
    p=ROOT/'upright_profiles.json';profiles=json.loads(p.read_text())['profiles'];jobs=[]
    for key,short,gain in [('cushion_support_shift_v2','v2',2.),('cushion_v2_push','push',1.)]:
        cfg=copy.deepcopy(profiles[key]);cfg['support_shift'].update(constant_body_height=True,height_feedback_gain=gain,lower_m=0.)
        jobs.append((f'height-{short}-final-60s',cfg,65.02))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,jobs))
