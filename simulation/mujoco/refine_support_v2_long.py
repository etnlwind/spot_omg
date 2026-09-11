import copy,json
from concurrent.futures import ProcessPoolExecutor
from develop_support_v2 import evaluate,OUT

def trial(job):return evaluate(job[0],job[1],65.02)
if __name__=='__main__':
    cfg=json.loads((OUT/'v2-fixed-long-support.json').read_text())['profile']
    jobs=[]
    for lift in (.025,.03):
        c=copy.deepcopy(cfg);c['params'][3]=lift
        jobs.append((f'v2-fixed-lift{round(lift*1000)}-60s',c))
    c=copy.deepcopy(cfg);c['params'][0]=6.4;jobs.append(('v2-fixed-period64-60s',c))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,jobs))
