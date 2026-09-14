"""Compare broader swing lift with separate forward/turn foot-space settings."""
import copy,json,itertools
from concurrent.futures import ProcessPoolExecutor
from search_measured_gait import trial,OUT

def main():
    records=json.loads((OUT/'gait-search.json').read_text())
    forward=next(r['profile'] for r in records if r['name']=='measured_09')
    turn=next(r['profile'] for r in records if r['name']=='measured_00')
    candidates=[]
    for i,(lift,rise,height) in enumerate(itertools.product((.024,.032),(.25,.4),(.20175,.215))):
        p=copy.deepcopy(forward);p['turn_reverse_params']=turn['params'].copy()
        p['turn_reverse_params'][3]=lift;p['turn_reverse_params'][4]=height
        p['measured_swing']={'rise_fraction':rise,'fall_fraction':.3}
        candidates.append((f'hold_{i:02d}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,'left',14,'nominal') for n,p in candidates]):
            rows.append(r);print(r['name'],round(r['score'],2),round(r.get('min_peak_clearance_mm',0),1),round(r.get('max_mid_contact',1),2),flush=True)
            (OUT/'hold-search.json').write_text(json.dumps(rows,indent=2))
        top=sorted(rows,key=lambda r:r['score'])[:3]
        validation=list(pool.map(trial,[(r['name'],r['profile'],d,14,'nominal') for r in top for d in ('forward','right')]))
    (OUT/'hold-validation.json').write_text(json.dumps(validation,indent=2))
    for r in validation:print(r['name'],r['direction'],round(r['score'],2),flush=True)
    selected=min(top,key=lambda r:r['score']+sum(v['score'] for v in validation if v['name']==r['name']))
    profile=selected['profile'];profile['source_candidate']=selected['name'];profile['validation_status']='failed_pending_robust_validation'
    profile['label']='실측 기반 · 선행 발 들기 · 실험 (검증실패)'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_lift':profile}},indent=2,ensure_ascii=False))
    print('SELECTED',selected['name'],flush=True)
if __name__=='__main__':main()
