"""A/B scheduled CAD control trials without changing plant parameters."""
import json,sys
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics
from arc_contact_metrics import arc_contact_metrics
OUT=Path(__file__).resolve().parents[2]/'artifacts/audits/arc-control-2026-09-12'

def trial(case):
    rec=Recorder();headings=[];end=case.get('stop_at',14)
    def observe(now,r):
        rec(now,r)
        if 5<=now<end:
            R=r.plant.data.xmat[r.plant.model.body('robot').id].reshape(3,3)
            headings.append((now,float(np.arctan2(R[1,0],R[0,0]))))
    pad=json.loads(Path(__file__).with_name('foot_cushion_d37p3_l27mm.json').read_text())
    overrides=dict(case['parameters']);overrides['tracking_feedback_enabled']=False
    result,rows=run(case.get('linear',0),case.get('yaw',-1000),end+4.02,profile=case.get('profile','arcturn'),cushion=pad,stop_at=end,observer=observe,parameter_overrides=overrides,scenario=case.get('scenario','nominal'))
    steady=[r for r in rows if 5<=r['time_s']<end]
    result['metrics']=metrics([f for f in rec.frames if 5<=f['time_s']<end],steady)
    trajectory=overrides.get('arc_trial',[.02,.5,.04,0,1.2,.2])
    result['contact_metrics']=arc_contact_metrics(rows,start_s=5,end_s=end,duty=trajectory[1],target_lead_s=trajectory[2],period_s=trajectory[4] if len(trajectory)>4 else 1.2)
    h=np.unwrap([v[1] for v in headings]);result['rotation_deg_s']=float(np.degrees(h[-1]-h[0])/(headings[-1][0]-headings[0][0]))
    result['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    result['case']=case
    OUT.mkdir(parents=True,exist_ok=True);(OUT/(case['name']+'.json')).write_text(json.dumps(result,indent=2,default=float))
    if case.get('save_rows'):
        import csv
        with (OUT/(case['name']+'.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    cm=result['contact_metrics'];m=result['metrics']
    print(case['name'],result['safety'],round(result['rotation_deg_s'],2),round(m['max_roll_deg'],2),round(m['max_pitch_deg'],2),json.dumps(cm)[:300],flush=True)
    return result


def safe_trial(case):
    """Keep infeasible controller candidates visible without aborting a batch."""
    try:
        return trial(case)
    except (ValueError,RuntimeError) as exc:
        result=dict(case=case,evaluation_status='infeasible',error=str(exc),
                    estimated_physics=True,stop_completed=False)
        OUT.mkdir(parents=True,exist_ok=True)
        (OUT/(case['name']+'.json')).write_text(json.dumps(result,indent=2,default=float))
        print(case['name'],'INFEASIBLE',str(exc),flush=True)
        return result

if __name__=='__main__':
    if len(sys.argv)>1:cases=json.loads(Path(sys.argv[1]).read_text())
    else:
        cases=[]
        for gain in (0,.15,.4,.8):
            for height in (.021,.025):
                cases.append(dict(name=f'cad-g{gain}-h{height}',parameters=dict(arc_trial=[height,.5,.04,0],arc_preload_trial=[.3,.04],arc_attitude_trial=[gain,1 if gain else 0,0,.006,.04,.08])))
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(safe_trial,cases))
