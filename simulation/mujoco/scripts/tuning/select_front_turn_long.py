"""Choose among stable short-run front-lift candidates using long-run results."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json,copy
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.tuning.search_front_turn_lift import OUT
from simulation.mujoco.scripts.tuning.search_measured_gait import trial

def main():
    candidates=[]
    for file,name in [('level-search.json','level_6_25'),('refined-search.json','refine_6_20'),('refined-search.json','refine_7_30')]:
        p=copy.deepcopy(next(r['profile'] for r in json.loads((OUT/file).read_text()) if r['name']==name))
        p['position_wbc']['turn_only']=True;candidates.append((name,p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,d,67,'final_nominal') for n,p in candidates for d in ('left','right')]):
            rows.append(r);print(r['name'],r['direction'],r.get('faults'),r.get('peak_tilt_deg'),r.get('max_mid_contact'),flush=True)
            (OUT/'long-selection.json').write_text(json.dumps(rows,indent=2))
    def score(r):
        if r.get('error') or r.get('faults') or not r.get('stop_ok'):return 10000
        legs=r['cycle_metrics']['phase_bases']['target']['legs'];front=min(legs[k]['swing_peak_clearance_mm']['median'] for k in ('FL','FR'))
        return 4*max(0,15-front)+2*max(0,r['peak_tilt_deg']-5)+50*r['max_mid_contact']+5*max(0,24-abs(r['yaw_rate_deg_s']))
    winner=min(candidates,key=lambda item:sum(score(r) for r in rows if r['name']==item[0]))
    p=winner[1];p['label']='실측 기반 · 앞발 높이 보강 · 실험 (검증실패)';p['validation_status']='failed'
    p['source_candidate']=winner[0];p['validation']='artifacts/audits/front-turn-lift-2026-09-12/long-selection.json'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_front_lift':p}},indent=2,ensure_ascii=False))
    print('SELECTED',winner[0],flush=True)
if __name__=='__main__':main()
