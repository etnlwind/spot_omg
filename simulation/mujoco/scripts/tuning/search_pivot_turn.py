"""Search true common-center foot arcs using chassis excursion as a selection metric."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json,copy,itertools
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.scripts.analysis.audit_pivot_center import OUT
ROOT=SIM_ROOT

def trial(task):
    name,profile,direction,seconds=task
    p=json.loads((ROOT/'config/measured_response_plant.json').read_text());p['timestep_s']=.001
    states=[]
    def obs(t,r):
        m,d=r.plant.model,r.plant.data;center=d.xipos[m.body('cad_base').id].copy();rot=d.xmat[m.body('robot').id].reshape(3,3)
        states.append((t,center.tolist(),float(np.arctan2(rot[1,0],rot[0,0])),r.safety))
    try:
        summary,feet=run(0,-1000 if direction=='left' else 1000,seconds,profile='cruise',override=profile,parameter_overrides=p,observer=obs,stop_at=seconds-2)
        steady=[x for x in states if 5<=x[0]<seconds-2];xy=np.array([x[1][:2] for x in steady]);yaw=np.unwrap([x[2] for x in steady]);t=np.array([x[0] for x in steady])
        legs=summary['legs'];tilt=max(v['peak_tilt_deg'] or 0 for v in legs.values());faults=sorted(set(x[3] for x in states if x[3]!='ok'))
        result=dict(name=name,profile=profile,direction=direction,seconds=seconds,net_mm=float(np.linalg.norm(xy[-1]-xy[0])*1000),
            excursion_mm=float(np.linalg.norm(xy-xy[0],axis=1).max()*1000),yaw_rate_deg_s=float(np.degrees(yaw[-1]-yaw[0])/(t[-1]-t[0])),
            peak_tilt_deg=tilt,faults=faults,stop_ok=summary['safety']=='ok' and not feet[-1]['moving'],legs=legs)
        result['score']=10000*bool(faults)+result['excursion_mm']+10*max(0,24-abs(result['yaw_rate_deg_s']))+5*max(0,tilt-5)
        return result
    except Exception as e:return dict(name=name,profile=profile,direction=direction,score=1e6,error=str(e))

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    base=json.loads((ROOT/'config/measured_response_profiles.json').read_text())['profiles']['measured_lift']
    candidates=[('before',base)]
    for period,lift in itertools.product((1.2,1.6),(.03,.045)):
        p=copy.deepcopy(base);p['turn_reverse_params'][0]=period;p['turn_reverse_params'][1]=.75
        p['pivot_turn']={'entry_arc':True,'retain_entry_foot_xy':True,'sweep_rad':.75*period*.46,'lift_m':lift,'offsets':[0.,.5,.75,.25]}
        candidates.append((f'crawl_{round(period*100)}_{round(lift*1000)}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,d,17) for n,p in candidates for d in ('left','right')]):
            rows.append(r);print(r['name'],r['direction'],r.get('excursion_mm'),r.get('yaw_rate_deg_s'),r.get('faults',r.get('error')),flush=True)
            (OUT/'crawl-search.json').write_text(json.dumps(rows,indent=2))
    valid=[(n,p) for n,p in candidates[1:] if all(not r.get('faults') and not r.get('error') and r.get('stop_ok') for r in rows if r['name']==n)]
    if not valid:print('NO VALID CANDIDATE',flush=True);return
    chosen=min(valid,key=lambda c:sum(r['score'] for r in rows if r['name']==c[0]));p=chosen[1]
    p['source_candidate']=chosen[0];p['label']='몸체 중심 · 제자리 회전 · 실험 (검증실패)';p['validation_status']='failed'
    p['validation']='artifacts/audits/pivot-center-2026-09-12/crawl-search.json'
    (OUT/'crawl-search-best.json').write_text(json.dumps({'profiles':{'center_pivot':p}},indent=2,ensure_ascii=False))
    print('LOWEST SCORE, NOT VALIDATED',chosen[0],flush=True)
if __name__=='__main__':main()
