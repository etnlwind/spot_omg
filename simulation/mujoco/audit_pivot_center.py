"""Measure torso-center translation independently of camera and free-joint origin."""
import json,math,csv
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/audits/pivot-center-2026-09-12'

def trial(task):
    name,direction,seconds=task[:3]
    supplied_profile=task[3] if len(task)>3 else None
    profiles=json.loads(Path(__file__).with_name('measured_response_profiles.json').read_text())['profiles']
    p=json.loads(Path(__file__).with_name('measured_response_plant.json').read_text());p['timestep_s']=.0005
    states=[]
    def obs(t,r):
        m,d=r.plant.model,r.plant.data
        center=d.xipos[m.body('cad_base').id].copy()
        rot=d.xmat[m.body('robot').id].reshape(3,3)
        row=r.plant.row()
        states.append(dict(time_s=t,x_m=float(center[0]),y_m=float(center[1]),z_m=float(center[2]),
            yaw_rad=math.atan2(rot[1,0],rot[0,0]),roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],safety=r.safety))
    summary,feet=run(0,-1000 if direction=='left' else 1000,seconds,profile='cruise',override=(supplied_profile if supplied_profile is not None else profiles[name]),parameter_overrides=p,observer=obs,stop_at=seconds-2)
    steady=[s for s in states if 5<=s['time_s']<seconds-2];xy=np.array([[s['x_m'],s['y_m']] for s in steady]);ts=np.array([s['time_s'] for s in steady]);a=np.unwrap([s['yaw_rad'] for s in steady])
    before=states[99];start=np.array([before['x_m'],before['y_m']])
    active=np.array([[s['x_m'],s['y_m']] for s in states if 2<=s['time_s']<seconds-2])
    slope=np.linalg.lstsq(np.column_stack((ts-ts.mean(),np.ones(len(ts)))),xy,rcond=None)[0][0]
    result=dict(profile=name,direction=direction,seconds=seconds,torso_reference='CAD chassis bounding-box center / cad_base xipos',
        net_translation_mm=float(np.linalg.norm(xy[-1]-xy[0])*1000),max_center_excursion_mm=float(np.linalg.norm(xy-xy[0],axis=1).max()*1000),
        from_motion_start_max_mm=float(np.linalg.norm(active-start,axis=1).max()*1000),
        drift_trend_mm_s=float(np.linalg.norm(slope)*1000),yaw_rate_deg_s=float(np.degrees(a[-1]-a[0])/(ts[-1]-ts[0])),
        max_tilt_deg=max(max(abs(s['roll_deg']),abs(s['pitch_deg'])) for s in steady),
        faults=sorted(set(s['safety'] for s in states if s['safety']!='ok')),stop_ok=not feet[-1]['moving'] and summary['safety']=='ok')
    if seconds>=60:
        from arc_contact_metrics import arc_contact_metrics
        profile=supplied_profile if supplied_profile is not None else profiles[name]
        period,duty=profile.get('turn_reverse_params',profile['params'])[:2]
        result['cycles']=arc_contact_metrics(feet,start_s=5,end_s=seconds-2,duty=duty,period_s=period,target_lead_s=0)
    OUT.mkdir(parents=True,exist_ok=True)
    with (OUT/f'{name}-{direction}.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(states[0]));w.writeheader();w.writerows(states)
    return json.loads(json.dumps(result,default=lambda v:v.item() if isinstance(v,np.generic) else str(v)))

def main():
    rows=[]
    with ProcessPoolExecutor(max_workers=2) as pool:
        for r in pool.map(trial,[(p,d,27) for p in ('measured_lift','measured_front_lift') for d in ('left','right')]):
            rows.append(r);print(json.dumps(r),flush=True)
            (OUT/'audit.json').write_text(json.dumps(rows,indent=2))
if __name__=='__main__':main()
