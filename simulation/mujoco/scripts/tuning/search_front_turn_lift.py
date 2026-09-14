"""Front-only Cartesian lift comparison, retaining turn period/stride/command."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import copy,json,itertools
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulation.mujoco.scripts.tuning.search_measured_gait import trial
ROOT=REPO_ROOT
OUT=ROOT/'artifacts/audits/front-turn-lift-2026-09-12'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    base=json.loads((SIM_ROOT / 'config/measured_response_profiles.json').read_text())['profiles']['measured_lift']
    candidates=[('before',base)]
    for extra,offset in itertools.product((.006,.008),(.015,.025,.035)):
        height=.21;gain=.2
        p=copy.deepcopy(base);p['measured_swing']['cad_turn_lift_m']=[extra,extra,0.,0.]
        p['turn_reverse_params'][4]=height;p['turn_reverse_params'][5]=offset
        p['position_wbc']={'gain':gain,'joint_limit_deg':6.,'turn_only':True}
        candidates.append((f'load_{round(extra*1000)}_{round(offset*1000)}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,d,14,'final_nominal') for n,p in candidates for d in ('left','right')]):
            rows.append(r)
            print(r['name'],r['direction'],r.get('error',''),r.get('min_peak_clearance_mm'),r.get('max_mid_contact'),r.get('peak_tilt_deg'),r.get('yaw_rate_deg_s'),flush=True)
            (OUT/'load-search.json').write_text(json.dumps(rows,indent=2))
    def score(r):
        if r.get('error') or r.get('faults') or not r['stop_ok']:return 10000
        legs=r['cycle_metrics']['phase_bases']['target']['legs']
        front=min(legs[k]['swing_peak_clearance_mm']['median'] or 0 for k in ('FL','FR'))
        return 100*max(0,r['max_mid_contact']-.1)+2*max(0,r['peak_tilt_deg']-4)+3*max(0,18-front)+5*max(0,24-abs(r['yaw_rate_deg_s']))
    names=[n for n,p in candidates if n!='before' and all(not r.get('faults') and not r.get('error') and r.get('stop_ok') for r in rows if r['name']==n)]
    if not names:
        print('NO STABLE CANDIDATE',flush=True);return
    winner=min(names,key=lambda n:sum(score(r) for r in rows if r['name']==n))
    selected=copy.deepcopy(next(r['profile'] for r in rows if r['name']==winner))
    selected['label']='실측 기반 · 앞발 높이 보강 · 실험 (검증실패)';selected['source_candidate']=winner
    selected['validation']='artifacts/audits/front-turn-lift-2026-09-12/validation.json'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_front_lift':selected}},indent=2,ensure_ascii=False))
    print('SELECTED',winner,flush=True)
if __name__=='__main__':main()
