"""Read-only, headless turn evaluation of the app's existing 80 mm policy."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import csv,json,hashlib
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from simulation.mujoco.scripts.tuning.develop_support_v2 import ROOT, OUT
from simulation.mujoco.scripts.analysis.diagnose_turn_clearance import run
from simulation.mujoco.scripts.validation.validate_support_shift import Recorder, metrics


def trial(direction,config=None,prefix="wide80-turn",linear=0,stop_at=20.):
    cfg=json.loads((ROOT/'config/upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80']
    if config is not None:cfg=config
    pad=json.loads((ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text());rec=Recorder();heading=[]
    def observe(now,robot):
        rec(now,robot)
        d=robot.plant.data;m=robot.plant.model;R=d.xmat[m.body('robot').id].reshape(3,3)
        heading.append(dict(time_s=now,yaw_rad=float(np.arctan2(R[1,0],R[0,0])),
                            x_m=float(d.xipos[m.body('cad_base').id,0]),y_m=float(d.xipos[m.body('cad_base').id,1]),
                            applied_yaw=robot.yaw,applied_linear=robot.linear))
    name='right' if direction>0 else 'left'
    result,rows=run(linear,1000*direction,stop_at+4.02,profile='cushion_diagonal_sync_wide80',override=cfg,cushion=pad,observer=observe,stop_at=stop_at)
    h=[r for r in heading if 5<=r['time_s']<stop_at]
    angles=np.unwrap([r['yaw_rad'] for r in h]);delta=float(np.degrees(angles[-1]-angles[0]))
    pos=np.array([[r['x_m'],r['y_m']] for r in h])
    result.update(direction=name,measurement_start_s=5,measurement_end_s=h[-1]['time_s'],
        rotation_deg=delta,rotation_deg_s=delta/(h[-1]['time_s']-h[0]['time_s']),
        planar_displacement_mm=float(np.linalg.norm(pos[-1]-pos[0])*1000),
        max_radius_from_start_mm=float(np.linalg.norm(pos-pos[0],axis=1).max()*1000),
        applied_yaw_range=[min(r['applied_yaw'] for r in h),max(r['applied_yaw'] for r in h)],
        first_fault_s=next((f['time_s'] for f in rec.frames if f['safety']!='ok'),None),
        metrics=metrics([f for f in rec.frames if f['time_s']<stop_at],[r for r in rows if r['time_s']<stop_at]),
        stop_completed=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition'],
        source_hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in ('runtime/support_shift.py','runtime/virtual_robot.py','runtime/cad_physics.py')})
    # Foot data excludes the decelerating stop transition for comparable turns.
    for leg in ('FL','FR','RL','RR'):
        swing=[r for r in rows if r['leg']==leg and r['time_s']<stop_at and r['swing']]
        mid=[r for r in swing if r['middle_swing']]
        result['legs'][leg]['turn_peak_clearance_mm']=max((r['clearance_mm'] for r in swing),default=None)
        result['legs'][leg]['turn_middle_swing_contact_fraction']=float(np.mean([r['force_n']>.2 for r in mid])) if mid else None
    (OUT/f'{prefix}-{name}.json').write_text(json.dumps(result,indent=2))
    (OUT/f'{prefix}-{name}-heading.json').write_text(json.dumps(heading))
    with (OUT/f'{prefix}-{name}.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    print(name,delta,result['rotation_deg_s'],result['first_fault_s'],result['stop_completed'],flush=True)
    return result

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as p:r=list(p.map(trial,[-1,1]))
    (OUT/'wide80-turn-summary.json').write_text(json.dumps(r,indent=2))
