"""A/B estimated-physics trial of the shared timestamped feedback supervisor."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics
OUT=Path(__file__).resolve().parents[2]/'artifacts/audits/tracking-v42'

def trial(case):
    scenario,enabled,yaw=case
    overrides=dict(tracking_feedback_enabled=enabled)
    if scenario=='delay80':overrides['command_delay_s']=.08
    if scenario=='missing':overrides['tracking_feedback_drop']=True
    recorder=Recorder();rates=[];faults=[]
    def observe(now,r):
        recorder(now,r)
        if r.motion:
            rates.append(r.tracking.rate)
            if r.tracking.diagnostic.get('fault'):faults.append(r.tracking.diagnostic.copy())
    cushion=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    result,rows=run(0,yaw,14.02,profile='arcturn',cushion=cushion,stop_at=10,
                    parameter_overrides=overrides,observer=observe)
    result['metrics']=metrics(recorder.frames,rows)
    result['tracking_rate_min']=min(rates,default=1);result['tracking_faults']=faults
    result['stop_completed']=not recorder.frames[-1]['moving'] and not recorder.frames[-1]['transition']
    OUT.mkdir(parents=True,exist_ok=True)
    name=f'{scenario}-{int(enabled)}-{yaw}'
    (OUT/f'{name}.json').write_text(json.dumps(result,indent=2,default=float))
    print(name,result['safety'],result['stop_completed'],result['tracking_rate_min'],result['metrics']['tracking_rms_deg'],flush=True)
if __name__=='__main__':
    cases=[(s,e,y) for s in ('nominal','delay80') for e in (False,True) for y in (-1000,1000)]
    cases+=[('missing',True,-1000)]
    with ProcessPoolExecutor(max_workers=3) as pool:list(pool.map(trial,cases))
