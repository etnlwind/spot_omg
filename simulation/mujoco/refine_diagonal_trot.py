import copy,json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,ROOT,OUT

def evaluate(job):
    lift,lead=job
    cfg=json.loads((OUT/'diagonal-s60-h202.json').read_text())['profile']
    cfg['params'][3]=lift;cfg['support_shift']['kinematic_lead_s']=lead
    return trial((f'diagonal-lift{round(lift*1000)}-lead{round(lead*1000)}',cfg,20.02))
if __name__=='__main__':
    jobs=[(.025,.04),(.02,0),(.02,.04),(.015,0),(.015,.04),(.025,.06)]
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(evaluate,jobs))
