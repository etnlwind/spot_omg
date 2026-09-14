"""Additional V2 robustness checks, not a certification of physical hardware."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.scripts.validation.validate_support_shift import Recorder, metrics, fingerprints
from simulation.mujoco.scripts.tuning.develop_support_v2 import ROOT, OUT

def trial(job):
    name,scenario,override,stop=job
    cfg=json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['cushion_support_shift_v2']
    pad=json.loads((ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
    if name=='cushion':pad.update(contact_time_constant_s=.04,damping_ratio=.8,friction=[.6,.005,.0001])
    rec=Recorder();r,rows=run(1000,0,24.02,profile='cushion_support_shift_v2',override=cfg,scenario=scenario,cushion=pad,parameter_overrides=override,stop_at=stop,observer=rec)
    r.update(case=name,metrics=metrics(rec.frames,rows),controller_sha256=fingerprints(),
        first_fault_s=next((f['time_s'] for f in rec.frames if f['safety']!='ok'),None),
        stop_completed=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition'] if stop else None)
    (OUT/f'v2-condition-{name}.json').write_text(json.dumps(r,indent=2,default=lambda x:x.item()))
    print(name,r['safety'],r['first_fault_s'],r['stop_completed'],flush=True)
    return r
if __name__=='__main__':
    jobs=[('stop','nominal',{},18.),('com','com_offset',{},None),('servo_delay','nominal',{'command_delay_s':.08},None),('motor_loss','motor_loss',{},None),('cushion','nominal',{},None),('imu_delay','nominal',{'bno055':{'fusion_delay_s':.06}},None)]
    with ProcessPoolExecutor(max_workers=2) as pool:results=list(pool.map(trial,jobs))
    (OUT/'v2-conditions.json').write_text(json.dumps(results,indent=2,default=lambda x:x.item()))
