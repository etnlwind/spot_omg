"""Retain forward lift-hold, test shorter/faster turn arcs without slowing yaw command."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import copy,json,itertools
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.tuning.search_measured_gait import trial, OUT

def main():
    base=json.loads((OUT/'profiles.json').read_text())['profiles']['measured_lift']
    candidates=[]
    for i,(stride,lift,period) in enumerate(itertools.product((.04,.055,.07),(.022,.03),(.8,1.05))):
        p=copy.deepcopy(base);p['measured_swing']['forward_only']=True
        p['turn_reverse_params']=[period,.52,stride,lift,.20175,-.035,.75]
        candidates.append((f'turn_{i:02d}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,'left',12,'nominal') for n,p in candidates]):
            rows.append(r);print(r['name'],round(r['score'],2),round(r.get('min_peak_clearance_mm',0),1),round(r.get('max_mid_contact',1),2),round(r.get('yaw_rate_deg_s',0),1),flush=True)
            (OUT/'turn-search.json').write_text(json.dumps(rows,indent=2))
        top=sorted(rows,key=lambda r:r['score'])[:3]
        more=list(pool.map(trial,[(r['name'],r['profile'],d,14,'nominal') for r in top for d in ('forward','right')]))
    (OUT/'turn-validation.json').write_text(json.dumps(more,indent=2))
    for r in more:print(r['name'],r['direction'],round(r['score'],2),flush=True)
    selected=min(top,key=lambda r:r['score']+sum(v['score'] for v in more if v['name']==r['name']))
    p=selected['profile'];p['source_candidate']=selected['name'];p['validation_status']='failed_pending_robust_validation'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_lift':p}},indent=2,ensure_ascii=False))
    print('SELECTED',selected['name'],flush=True)
if __name__=='__main__':main()
