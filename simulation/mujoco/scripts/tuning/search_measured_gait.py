"""Simulator-only foot-space policy search constrained by recorded response uncertainty."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import copy,json,itertools,argparse,math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.runtime.gait_profiles import load_profiles
ROOT=REPO_ROOT
OUT=ROOT/'artifacts/audits/measured-gait-2026-09-12'

def trial(task):
    name,profile,direction,seconds,scenario=task
    p=json.loads((OUT/'plant.json').read_text());p['timestep_s']=.001
    if scenario=='delay60':p['command_delay_s']=.06
    if scenario=='motor85':
        for k in ('motor_3215','motor_3250'):
            for key in ('stall_nm','speed_rad_s'):p[k][key]*=.85
    if scenario=='heavy':
        for key in p['mass_kg']:p['mass_kg'][key]*=1.2
    if scenario=='soft':p['foot_cushion']['contact_time_constant_s']*=1.5
    if scenario=='fit_candidate':
        p['servo_kp']=[35.,45.,45.]*4;p['servo_kd']=[.8,1.6,1.6]*4
    if scenario.startswith('final_'):
        p['timestep_s']=.0005
        if scenario=='final_delay60':p['command_delay_s']=.06
    if scenario=='fine':p['timestep_s']=.0005
    seq=[]
    def observe(t,r):
        row=r.plant.row();rot=r.plant.data.xmat[r.plant.model.body('robot').id].reshape(3,3)
        seq.append((t,math.atan2(rot[1,0],rot[0,0]),row['position_m'],row['roll_deg'],row['pitch_deg'],row['torque_limit_fraction'],row['max_tracking_error_deg'],r.safety))
    linear,yaw={'forward':(1000,0),'left':(0,-1000),'right':(0,1000)}[direction]
    try:
        summary,rows=run(linear,yaw,seconds,profile='cruise',override=profile,parameter_overrides=p,observer=observe,stop_at=seconds-2)
        measured=[x for x in seq if 5<=x[0]<seconds-2]
        angles=np.unwrap([x[1] for x in measured]);dt=measured[-1][0]-measured[0][0]
        legs=summary['legs'];clearance=min(v['peak_clearance_mm'] or 0 for v in legs.values())
        contact=max(v['middle_swing_contact_fraction'] if v['middle_swing_contact_fraction'] is not None else 1 for v in legs.values())
        tilt=max(max(abs(x[3]),abs(x[4])) for x in measured)
        turn=float(np.degrees(angles[-1]-angles[0])/dt)
        xy=np.array(measured[-1][2][:2])-measured[0][2][:2]
        signed_speed=float((xy[0]*math.cos(measured[0][1])+xy[1]*math.sin(measured[0][1]))/dt)
        faults=sorted(set(x[7] for x in seq if x[7]!='ok'))
        result=dict(name=name,profile=profile,direction=direction,scenario=scenario,seconds=seconds,
            safety=summary['safety'],faults=faults,legs=legs,min_peak_clearance_mm=clearance,max_mid_contact=contact,
            peak_tilt_deg=tilt,rms_roll_deg=float(np.sqrt(np.mean([x[3]**2 for x in measured]))),
            rms_pitch_deg=float(np.sqrt(np.mean([x[4]**2 for x in measured]))),yaw_rate_deg_s=turn,
            forward_speed_m_s=signed_speed,drift_m_s=float(np.linalg.norm(xy)/dt),
            saturation_fraction=float(np.mean([x[5] for x in measured])),peak_tracking_deg=max(x[6] for x in measured),
            stop_ok=summary['safety']=='ok' and not rows[-1]['moving'])
        result['height_range_mm']=1000*(max(x[2][2] for x in measured)-min(x[2][2] for x in measured))
        if seconds>=60 or scenario.startswith('final_'):
            from simulation.mujoco.runtime.arc_contact_metrics import arc_contact_metrics
            import csv
            period,duty=profile.get('turn_reverse_params',profile['params'])[:2] if direction!='forward' else profile['params'][:2]
            result['cycle_metrics']=arc_contact_metrics(rows,start_s=5,end_s=seconds-2,duty=duty,target_lead_s=0,period_s=period)
            folder=OUT/'validation';folder.mkdir(exist_ok=True)
            with (folder/f'{name}-{direction}-{scenario}.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        result['eligible']=not faults and result['stop_ok'] and tilt<10 and clearance>=15 and contact<.1 and (signed_speed>.015 if direction=='forward' else turn*yaw<0 and abs(turn)>3)
        # Lexicographic feasibility followed by clearance/contact/tilt and useful movement.
        result['score']=(1000*bool(faults)+100*max(0,contact-.1)+max(0,15-clearance)+2*max(0,tilt-5)
                         - (signed_speed*20 if direction=='forward' else min(abs(turn),30)*.1))
        return json.loads(json.dumps(result,default=lambda value:value.item() if isinstance(value,np.generic) else (_ for _ in ()).throw(TypeError(type(value).__name__))))
    except Exception as e:return dict(name=name,profile=profile,direction=direction,scenario=scenario,seconds=seconds,eligible=False,score=100000,error=str(e))

def main():
    base=copy.deepcopy(load_profiles()['cruise']);base.pop('turn_reverse_params',None)
    candidates=[('baseline',base)]
    for i,(period,duty,lift,height) in enumerate(itertools.product((1.05,1.35),(.52,.6),(.024,.04),(.20175,.22))):
        p=copy.deepcopy(base);p['params']=[period,duty,.08,lift,height,-.035,.75]
        p['balance_base']='cruise';p['label']='실측 기반 · 발 들림 · 실험 (검증실패)'
        candidates.append((f'measured_{i:02d}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,'left',10,'nominal') for n,p in candidates]):
            rows.append(r);print(r['name'],round(r['score'],2),r.get('min_peak_clearance_mm'),r.get('yaw_rate_deg_s'),flush=True)
            (OUT/'gait-search.json').write_text(json.dumps(rows,indent=2))
        chosen=sorted(rows[1:],key=lambda r:r['score'])[:3]
        tasks=[(r['name'],r['profile'],d,14,'nominal') for r in [rows[0],*chosen] for d in ('forward','left','right')]
        validations=[]
        for r in pool.map(trial,tasks):
            validations.append(r);print('validate',r['name'],r['direction'],round(r['score'],2),flush=True)
            (OUT/'gait-selection.json').write_text(json.dumps(validations,indent=2))
    scores={r['name']:sum(x['score'] for x in validations if x['name']==r['name']) for r in chosen}
    selected=min(chosen,key=lambda r:scores[r['name']])
    profile=copy.deepcopy(selected['profile']);profile['validation_status']='failed_pending_robust_validation'
    profile['source_candidate']=selected['name']
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_lift':profile}},indent=2,ensure_ascii=False))
    print('SELECTED',selected['name'],flush=True)
if __name__=='__main__':main()
