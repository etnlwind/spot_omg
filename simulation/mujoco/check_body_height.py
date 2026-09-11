"""Rigid torso height, not whole-robot COM (which moves with the limbs)."""
import copy,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from develop_support_v2 import ROOT,OUT
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics

def trial(job):
    name,cfg,seconds=job
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());rec=Recorder();height=[]
    def observe(now,r):
        rec(now,r);state=r.plant.row();d=r.plant.data;m=r.plant.model
        diag=r.support_shift.diagnostic if r.support_shift else {}
        height.append(dict(time_s=now,body_height_m=float(d.xipos[m.body('cad_base').id,2]),
            target_m=diag.get('body_height_target_m'),command_shift_z_m=diag.get('body_shift_m',[0,0,0])[2],
            safety=r.safety))
    result,rows=run(1000,0,seconds,profile=name,override=cfg,cushion=pad,observer=observe)
    measured=[h for h in height if h['time_s']>=5]
    z=np.array([h['body_height_m'] for h in measured]);target=np.array([h['target_m'] for h in measured])
    result.update(metrics=metrics(rec.frames,rows),height=dict(peak_to_peak_mm=float(np.ptp(z)*1000),
        std_mm=float(np.std(z)*1000),rms_target_error_mm=float(np.sqrt(np.mean((z-target)**2))*1000),
        mean_height_mm=float(z.mean()*1000),target_mm=float(target[0]*1000)),
        first_fault_s=next((h['time_s'] for h in height if h['safety']!='ok'),None))
    result['feedback_task_residual_max_mm']=1000*max((f['support_shift'].get('feedback_residual_m',0) for f in rec.frames if f['time_s']>=5),default=0)
    result['applied_task_residual_max_mm']=1000*max((f['support_shift'].get('applied_task_residual_m',0) for f in rec.frames if f['time_s']>=5),default=0)
    (OUT/f'{name}.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    (OUT/f'{name}-height.json').write_text(json.dumps(height,indent=2))
    print(name,result['safety'],result['first_fault_s'],result['height'],flush=True)
    return result
if __name__=='__main__':
    profiles=json.loads((ROOT/'upright_profiles.json').read_text())['profiles'];jobs=[]
    for key,short in [('cushion_support_shift_v2','v2'),('cushion_v2_push','push')]:
        for fixed in (False,True):
            cfg=copy.deepcopy(profiles[key]);cfg['support_shift']['constant_body_height']=fixed
            jobs.append((f'height-{short}-'+('fixed' if fixed else 'before'),cfg,20.02))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,jobs))
