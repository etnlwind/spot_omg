"""Phase-aligned joint/foot telemetry; ablations are diagnostic only."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController

OUT=Path(__file__).with_name('diagnostics')/'j3-lift'

def run(mode,profile="level15"):
    p,m=physics()
    if mode=='zero_command_delay':p['command_delay_s']=0
    r=RobotController(Simulation(p,m));r.select_profile(profile)
    apply=r.balance.apply
    def wrapped(nominal,*args):
        out=apply(nominal,*args)
        if mode=='no_j23_feedback':
            out=out.copy()
            for j in range(12):
                if j%3:out[j]=nominal[j]
        return out
    r.balance.apply=wrapped
    if mode=='balance_off':r.balance.enabled=False
    feet=[m.geom(n+'_foot').id for n in ('fl','fr','rl','rr')]
    rows=[]
    for i in range(1150):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<1000 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==1000:r.command('@S 99999',t)
        r.tick(t);row=r.plant.row();r.drain()
        rows.append(dict(time_s=t,phase=(r.phase-.02/1.4)%1,nominal_deg=r.target.tolist(),
            command_deg=r.command_target.tolist(),quantized_deg=row['target_deg'],actual_deg=row['actual_deg'],
            filtered_deg=np.degrees(r.plant.filtered).tolist(),correction_deg=(r.command_target-r.target).tolist(),
            torque_nm=row['torque_nm'],torque_limits_nm=r.plant.limits.tolist(),contacts=row['contacts'],
            clearance_mm=[float((r.plant.data.geom_xpos[g,2]-m.geom_size[g,0])*1000) for g in feet],
            roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],safety=r.safety))
        if r.safety!='ok':break
    steady=[v for v in rows if 5<=v['time_s']<20]
    summary=dict(mode=mode,profile=profile,safety=r.safety,legs={})
    a=np.array([[v['roll_deg'],v['pitch_deg']] for v in steady]);summary['rms_tilt_deg']=float(np.sqrt(np.mean(np.sum(a*a,axis=1))))
    for leg,name in enumerate(('fl','fr','rl','rr')):
        swing=[v for v in steady if (v['phase']+(0,.5,.5,0)[leg])%1>=.64]
        j=leg*3+2;cmd=np.array([v['command_deg'][j] for v in steady]);act=np.array([v['actual_deg'][j] for v in steady])
        lag=min(range(16),key=lambda k:np.mean((cmd[:len(cmd)-k or None]-act[k:])**2))
        summary['legs'][name]=dict(j3_lag_ms=lag*20,
            swing_tracking_rms_deg=float(np.sqrt(np.mean([(v['command_deg'][j]-v['actual_deg'][j])**2 for v in swing]))),
            swing_feedback_mean_deg=float(np.mean([v['correction_deg'][j] for v in swing])),
            swing_feedback_peak_deg=float(max(abs(v['correction_deg'][j]) for v in swing)),
            swing_contact_fraction=float(np.mean([name+'_foot' in v['contacts'] for v in swing])),
            clearance_p95_mm=float(np.percentile([v['clearance_mm'][leg] for v in swing],95)),
            j3_command_span_deg=float(np.ptp(cmd)),j3_actual_span_deg=float(np.ptp(act)),
            j2_tracking_rms_deg=float(np.sqrt(np.mean([(v['command_deg'][j-1]-v['actual_deg'][j-1])**2 for v in swing]))),
            j2_torque_peak_nm=float(max(abs(v['torque_nm'][j-1]) for v in steady)),
            j3_torque_peak_nm=float(max(abs(v['torque_nm'][j]) for v in steady)),
            j3_torque_limit_fraction=float(np.mean([abs(v['torque_nm'][j])>=.98*v['torque_limits_nm'][j] for v in swing])))
    OUT.mkdir(exist_ok=True,parents=True);(OUT/((mode if profile=='level15' else profile+'-'+mode)+'.json')).write_text(json.dumps(dict(summary=summary,frames=rows),indent=2)+'\n')
    return summary

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=4) as pool:results=list(pool.map(run,['baseline','no_j23_feedback','balance_off','zero_command_delay']))
    print(json.dumps(results,indent=2));(OUT/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
