"""Evaluate FR clearance against perfect joint tracking; no control truth input."""
import json
from pathlib import Path
import mujoco
import numpy as np
from cad_physics import foot_clearance
from diagnose_turn_clearance import run
OUT=Path(__file__).resolve().parents[2]/'artifacts/audits/fr-forward-2026-09-12'
def main():
    rows=[];cache={}
    def observe(t,r):
        if t<5 or t>=15:return
        m,d=r.plant.model,r.plant.data
        if not cache:cache['data']=mujoco.MjData(m)
        ideal=cache['data'];ideal.qpos[:]=d.qpos
        ideal.qpos[r.plant.q]=np.radians(r.command_target)
        mujoco.mj_kinematics(m,ideal)
        # phase already advanced; use the phase that generated the current frame.
        phase=(r.phase-.02/(1.35*(1.35-.35*min(1,abs(r.linear)+abs(r.yaw)))))%1
        for i,leg in enumerate(('fl','fr','rl','rr')):
            q=(phase+(0,.5,.5,0)[i])%1;u=(q-.52)/.48
            if not .2<u<.8:continue
            actual=np.degrees(d.qpos[r.plant.q])[i*3:i*3+3]
            target=r.command_target[i*3:i*3+3]
            rows.append(dict(time_s=t,leg=leg,actual_mm=1000*foot_clearance(m,d,m.geom(leg+'_foot').id),
                perfect_tracking_same_body_mm=1000*foot_clearance(m,ideal,m.geom(leg+'_foot').id),
                target_deg=target.tolist(),actual_deg=actual.tolist()))
    p=json.loads(Path(__file__).with_name('measured_response_plant.json').read_text());p['timestep_s']=.0005
    run(1000,0,17,profile='centerpivot',parameter_overrides=p,observer=observe,stop_at=15)
    result={}
    for leg in ('fl','fr','rl','rr'):
        all_rows=[r for r in rows if r['leg']==leg];low=[r for r in all_rows if r['actual_mm']<1]
        result[leg]=dict(middle_samples=len(all_rows),below_1mm_samples=len(low),
            perfect_tracking_median_when_low_mm=float(np.median([r['perfect_tracking_same_body_mm'] for r in low])) if low else None,
            rms_joint_error_deg=np.sqrt(np.mean([(np.array(r['target_deg'])-r['actual_deg'])**2 for r in all_rows],axis=0)).tolist())
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/'tracking-counterfactual.json').write_text(json.dumps(dict(summary=result,rows=rows),indent=2))
    print(json.dumps(result),flush=True)
if __name__=='__main__':main()
