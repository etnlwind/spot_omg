"""Fixed-estimate preload validation; preserves failed cases and Stop results."""
import csv
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
from validate_support_shift import Recorder, metrics
OUT=Path(__file__).resolve().parents[2]/'artifacts/audits/arc-preload-validation'

def trial(case):
    delay,yaw,enabled=case
    rec=Recorder();headings=[]
    def observe(now,r):
        rec(now,r)
        if 5<=now<65:
            R=r.plant.data.xmat[r.plant.model.body('robot').id].reshape(3,3)
            headings.append((now,float(np.arctan2(R[1,0],R[0,0]))))
    cushion=json.loads(Path(__file__).with_name('foot_cushion_d37p3_l27mm.json').read_text())
    result,rows=run(0,yaw,69.02,profile='arcturn',cushion=cushion,stop_at=65,observer=observe,
        parameter_overrides=dict(arc_trial=[.021 if enabled else .02,.5,.04,0],
        arc_preload_trial=[.3,.04] if enabled else None,command_delay_s=delay,tracking_feedback_enabled=False))
    steady=[f for f in rec.frames if 5<=f['time_s']<65]
    steady_rows=[r for r in rows if 5<=r['time_s']<65]
    result['metrics']=metrics(steady,steady_rows)
    result['rotation_deg_s']=float(np.degrees(np.unwrap([h[1] for h in headings])[-1]-headings[0][1])/(headings[-1][0]-headings[0][0]))
    result['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    result['stop_safety_faults']=sorted(set(f['safety'] for f in rec.frames if f['time_s']>=65 and f['safety']!='ok'))
    result['estimated_controller_mass_kg']=4.418
    result['enabled']=enabled;result['delay_s']=delay
    result['steady_legs']={}
    for leg in ('FL','FR','RL','RR'):
        swing=[r for r in steady_rows if r['leg']==leg and r['moving'] and r['swing']]
        middle=[r for r in swing if r['middle_swing']]
        result['steady_legs'][leg]=dict(peak_clearance_mm=max((r['clearance_mm'] for r in swing),default=None),
            middle_swing_contact_fraction=float(np.mean([r['force_n']>.2 for r in middle])) if middle else None)
    OUT.mkdir(parents=True,exist_ok=True)
    name=f'd{round(delay*1000)}-y{yaw}-enabled{int(enabled)}'
    (OUT/f'{name}.json').write_text(json.dumps(result,indent=2,default=float))
    with (OUT/f'{name}.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(name,result['safety'],result['stop_completed'],round(result['rotation_deg_s'],2),round(result['metrics']['max_roll_deg'],2),result['steady_legs'],flush=True)
def summarize():
    for path in OUT.glob('d*-enabled1.json'):
        result=json.loads(path.read_text())
        baseline=json.loads(path.with_name(path.name.replace('enabled1','enabled0')).read_text())
        m=result['metrics'];b=baseline['metrics'];legs=result['steady_legs'].values()
        result['gates']=dict(
            uninterrupted=not m['safety_faults'] and m['moving_fraction']==1.,
            tilt=m['max_roll_deg']<=3 and m['max_pitch_deg']<=3 and m['rms_roll_deg']<=1.5 and m['rms_pitch_deg']<=1.5,
            clearance=all(v['peak_clearance_mm'] is not None and v['peak_clearance_mm']>=15 for v in legs),
            contact=all(v['middle_swing_contact_fraction'] is not None and v['middle_swing_contact_fraction']<.1 for v in legs),
            tracking=m['tracking_rms_deg']<=b['tracking_rms_deg'],
            rolling_inclusive_slip_proxy=m['stance_center_speed_proxy_m_s'] is not None and b['stance_center_speed_proxy_m_s'] is not None and m['stance_center_speed_proxy_m_s']<=b['stance_center_speed_proxy_m_s'],
            rotation_speed=abs(result['rotation_deg_s'])>=abs(baseline['rotation_deg_s']),
            stop=result['stop_completed'] and not result['stop_safety_faults'])
        result['validated']=all(result['gates'].values())
        path.write_text(json.dumps(result,indent=2))

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=3) as pool:
        list(pool.map(trial,[(d,y,e) for d in (.02,.08) for y in (-1000,1000) for e in (False,True)]))
    summarize()
