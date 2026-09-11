import copy,json
from concurrent.futures import ProcessPoolExecutor
from develop_support_v2 import evaluate,OUT

def trial(job):
    name,profile,seconds=job
    return evaluate(name,profile,seconds)
if __name__=='__main__':
    cfg=json.loads((OUT/'v2-fixed-long-support.json').read_text())['profile']
    jobs=[('v2-fixed-support85-60s',copy.deepcopy(cfg),65.02)]
    for duty in (.8,.82):
        c=copy.deepcopy(cfg);c['params'][1]=duty
        jobs.append((f'v2-fixed-support{round(duty*100)}',c,20.02))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,jobs))
