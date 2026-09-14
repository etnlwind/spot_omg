"""Compare Cartesian swing shapes with unchanged arc sweep and cycle period."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics
OUT=Path(__file__).resolve().parents[2]/'artifacts/audits/arc-shift-2026-09-12'
CONFIGS=[(.02,.5,.04,0),(.02,.5,.04,1),(.03,.5,.04,0),(.03,.5,.04,1),(.04,.5,.04,1),(.03,.55,.04,1),(.03,.5,.08,1),(.04,.5,.08,1),(.03,.6,.04,0),(.04,.6,.04,0),(.03,.65,.04,0),(.03,.6,.08,0),(.02,.6,.04,0),(.025,.55,.04,0),(.022,.5,.06,0),(.022,.5,.12,0),(.022,.5,.06,.15),(.022,.5,.12,.15),(.021,.5,.04,0),(.022,.5,.04,0)]
def trial(case):
    index,delay,yaw=case[:3];options=CONFIGS[index]
    balance=case[3] if len(case)>3 else None
    preload=case[4] if len(case)>4 else None
    shift=case[5]
    rec=Recorder();headings=[]
    def observe(now,r):
        rec(now,r)
        if 5<=now<10:
            R=r.plant.data.xmat[r.plant.model.body('robot').id].reshape(3,3)
            headings.append((now,float(np.arctan2(R[1,0],R[0,0]))))
    pad=json.loads(Path(__file__).with_name('foot_cushion_d37p3_l27mm.json').read_text())
    result,rows=run(0,yaw,14.02,profile='arcturn',cushion=pad,stop_at=10,observer=observe,
        parameter_overrides=dict(arc_trial=options,arc_balance_trial=balance,arc_preload_trial=preload,command_delay_s=delay,tracking_feedback_enabled=False,arc_shift_trial=shift))
    result['metrics']=metrics(rec.frames,rows);result['options']=options
    result['rotation_deg_s']=float(np.degrees(np.unwrap([x[1] for x in headings])[-1]-np.unwrap([x[1] for x in headings])[0])/(headings[-1][0]-headings[0][0]))
    result['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    # Evaluate phase labels with trial duty, never claim target height as clearance.
    for leg in ('FL','FR','RL','RR'):
        swing=[r for r in rows if r['leg']==leg and r['moving'] and r['phase']>=options[1]]
        middle=[r for r in swing if .2<=(r['phase']-options[1])/(1-options[1])<=.8]
        result['legs'][leg]['peak_clearance_mm']=max((r['clearance_mm'] for r in swing),default=None)
        result['legs'][leg]['middle_swing_contact_fraction']=float(np.mean([r['force_n']>.2 for r in middle])) if middle else None
    OUT.mkdir(parents=True,exist_ok=True);name=f'c{index}-d{round(delay*1000)}-y{yaw}'+(f'-b{balance[0]}-{balance[1]}' if balance else '')+(f'-p{preload[0]}-{preload[1]}' if preload else '')
    (OUT/f'{name}-shift{shift}.json').write_text(json.dumps(result,indent=2,default=float))
    print(name,result['safety'],round(result['rotation_deg_s'],1),round(result['metrics']['max_roll_deg'],2),[round(v['peak_clearance_mm'] or 0,1) for v in result['legs'].values()],[round(v['middle_swing_contact_fraction'] or 0,2) for v in result['legs'].values()],flush=True)

if __name__=='__main__':
    cases=[(18,.02,-1000,None,p,(w,g,l)) for p in (None,(.3,.04)) for w,g,l in ((.12,1.,0),(.24,1.,0),(.24,.5,.04),(.12,1.,.08))]
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,cases))
