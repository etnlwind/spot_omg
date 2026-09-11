import copy,json
from concurrent.futures import ProcessPoolExecutor
import mujoco
import numpy as np
from develop_support_v2 import ROOT,OUT
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics

def trial(bias,pulse=0.,peak=.7,shape="sine"):
    key=str(bias) if not pulse else f"pulse{round(pulse*1000)}"
    if peak!=.7:key+=f"-phase{round(peak*100)}"
    if shape!="sine":key+="-"+shape
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift_v2'])
    cfg['support_shift']['push_bias']=bias
    cfg['support_shift']['forward_pulse_m']=pulse
    cfg['support_shift']['forward_pulse_peak_phase']=peak
    cfg['support_shift']['forward_pulse_shape']=shape
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());rec=Recorder();force_rows=[]
    def observe(now,r):
        rec(now,r)
        if now<5:return
        m=r.plant.model;d=r.plant.data;axis=d.xmat[m.body('robot').id].reshape(3,3)[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
        floor=m.geom('floor').id;feet=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')];forces=np.zeros((4,3))
        for i,c in enumerate(d.contact):
            if floor not in (c.geom1,c.geom2):continue
            foot=c.geom2 if c.geom1==floor else c.geom1
            if foot not in feet:continue
            f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
            forces[feet.index(foot)]+=(1 if c.geom2==foot else -1)*(c.frame.reshape(3,3).T@f[:3])
        force_rows.append(dict(time_s=now,forward_force_n=float(forces.sum(axis=0)@axis),forward_speed_m_s=float(d.qvel[:3]@axis),leg_forward_force_n=(forces@axis).tolist()))
    result,rows=run(1000,0,20.02,profile='push_candidate',override=cfg,cushion=pad,observer=observe)
    window=[r for r in force_rows if 6<=r['time_s']<=7]
    result.update(metrics=metrics(rec.frames,rows),first_fault_s=next((r['time_s'] for r in rec.frames if r['safety']!='ok'),None),window_6_7=dict(mean_net_forward_force_n=float(np.mean([r['forward_force_n'] for r in window])),mean_forward_speed_m_s=float(np.mean([r['forward_speed_m_s'] for r in window]))))
    (OUT/f'v2-push-{key}.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    (OUT/f'v2-push-{key}-forces.json').write_text(json.dumps(force_rows,indent=2))
    print(key,result['safety'],result['first_fault_s'],result['window_6_7'],result['metrics']['max_roll_deg'],result['metrics']['tracking_peak_deg'],flush=True)
    return result
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,[0,6,12]))
