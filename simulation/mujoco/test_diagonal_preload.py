"""Bounded free-body experiment; does not modify live profiles."""
import json
from concurrent.futures import ProcessPoolExecutor
from check_body_height import trial,OUT

def evaluate(gain):
    cfg=json.loads((OUT/'diagonal-lift20-lead40.json').read_text())['profile']
    cfg['support_shift']['load_preload_gain']=gain
    return trial((f'diagonal-preload-{gain:g}',cfg,20.02))

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(evaluate,[.5,1.]))
