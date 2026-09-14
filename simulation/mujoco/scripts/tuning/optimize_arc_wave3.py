"""Local offline shooting around the 60-second-stable rank3, fixed 24mm lift.

Evaluates the unchanged position-servo/physics pipeline for 25 steady seconds.
No simulator observation is fed into the controller; observations only score trials.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import hashlib
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import differential_evolution
from simulation.mujoco.scripts.tuning.search_arc_control import trial, OUT
from simulation.mujoco.runtime.cad_physics import CAD

ROOT=REPO_ROOT
AUDIT=ROOT/'artifacts/audits/arc-transfer-2026-09-12'
BASELINE_PATH=ROOT/'artifacts/audits/arc-preload-validation/d20-y-1000-enabled0.json'
BASELINE=json.loads(BASELINE_PATH.read_text())
TRACKING_BASELINE=BASELINE['metrics']['tracking_rms_deg']
SEED=np.array([.0017557040361956876,-.0009905324626316228,.0007808772327605116,.0012395485171061073,.0013113643286768628])
SOURCE_FILES=[*sorted((ROOT/'firmware/stm32-learning/Inc').glob('arc*.h')),
 ROOT/'tools/servo_tool/servo/gait_policy_host.c',ROOT/'tools/servo_tool/servo/shared_gait.py',
 *[(SIM_ROOT / s) for s in ('scripts/tuning/optimize_arc_wave3.py','scripts/tuning/search_arc_control.py','runtime/arc_contact_metrics.py','runtime/cad_physics.py','runtime/drive_controller.py','runtime/virtual_robot.py','scripts/tuning/search_gait_profiles.py','config/foot_cushion_d37p3_l27mm.json')],
 CAD/'physics_parameters.json',BASELINE_PATH]

def source_hashes():
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE_FILES}

def evaluate(x):
    x=np.asarray(x,dtype=float)
    name='wave3_'+hashlib.sha256(x.astype('<f8').tobytes()).hexdigest()[:12]
    path=OUT/(name+'.json')
    if path.exists():
        old=json.loads(path.read_text())
        if 'optimization_cost' in old:return float(old['optimization_cost'])
    before=source_hashes();bx,xc,xs,yc,ys=x
    params=[bx,0.,xc,xs,yc,ys]
    case=dict(name=name,stop_at=30,parameters=dict(arc_trial=[.024,.5,.04,0],arc_transfer_trial=[.12,0,.3],arc_attitude_trial=[0,0,0,.006,.04,.08],arc_wave_trial=params))
    phi=np.linspace(0,2*np.pi,1441)
    bound=float(np.max(np.hypot(bx+xc*np.cos(2*phi)+xs*np.sin(2*phi),yc*np.cos(phi)+ys*np.sin(phi))))
    if not np.isfinite(x).all() or bound>.00999:
        r=dict(case=case,exception='body-translation-bound',maximum_planned_xy_m=bound,optimization_cost=3000.+1000.*max(0.,bound-.01),validated=False)
    else:
        try:
            r=trial(case)
            m=r['metrics'];legs=list(r['contact_metrics']['phase_bases']['target']['legs'].values())
            peaks=[l['swing_peak_clearance_mm']['min'] for l in legs]
            p10=[l['swing_peak_clearance_mm']['p10'] for l in legs]
            contacts=[l['middle_swing_contact_fraction'] for l in legs]
            finite=all(v is not None and np.isfinite(v) for v in peaks+p10+contacts)
            uninterrupted=r['safety']=='ok' and not m['safety_faults'] and m['moving_fraction']>=.999
            if not uninterrupted:
                cost=1000.+100*(1.-m['moving_fraction'])+max(m['max_roll_deg'],m['max_pitch_deg'])
            elif not finite:
                cost=2000.
            else:
                # Violations dominate; smooth terms retain a useful direction below caps.
                peak=np.array(peaks);ct=np.array(contacts)
                roll=m['max_roll_deg'];pitch=m['max_pitch_deg'];rr=m['rms_roll_deg'];rp=m['rms_pitch_deg']
                speed=abs(r['rotation_deg_s']);track=m['tracking_rms_deg']
                terms=dict(
                    max_tilt=20*(max(0.,roll/3.-1.)**2+max(0.,pitch/3.-1.)**2),
                    rms_tilt=20*(max(0.,rr/1.5-1.)**2+max(0.,rp/1.5-1.)**2),
                    tilt_shape=.5*(rr*rr+rp*rp)+.2*(roll+pitch),
                    worst_clearance=40*(max(0.,15.-min(peak))/15.)**2,
                    clearance_distribution=20*float(np.mean((np.maximum(0.,15.-np.array(p10))/15.)**2)),
                    worst_contact=10*(max(0.,float(max(ct))-.1)/.1)**2,
                    contact_distribution=5*float(np.mean((np.maximum(0.,ct-.1)/.1)**2)),
                    speed=30*max(0.,17.4-speed)**2,
                    tracking=15*(max(0.,track/TRACKING_BASELINE-1.)/.05)**2,
                    rolling_inclusive_slip=3*(max(0.,m['stance_center_speed_proxy_m_s']/BASELINE['metrics']['stance_center_speed_proxy_m_s']-1.))**2,
                )
                cost=sum(terms.values());r['optimization_terms']=terms
            r['gates']=dict(uninterrupted=uninterrupted,
              max_tilt=m['max_roll_deg']<=3 and m['max_pitch_deg']<=3,
              rms_tilt=m['rms_roll_deg']<=1.5 and m['rms_pitch_deg']<=1.5,
              clearance=finite and all(v>=15 for v in peaks),contact=finite and all(v<.1 for v in contacts),
              speed=abs(r['rotation_deg_s'])>=17.4,tracking=m['tracking_rms_deg']<=TRACKING_BASELINE,
              stop=r['stop_completed'])
            r['validated']=all(r['gates'].values());r['optimization_cost']=float(cost)
        except (ValueError,RuntimeError) as e:
            r=dict(case=case,exception=f'{type(e).__name__}: {e}',optimization_cost=2500.,validated=False)
    r.update(optimization_x=x.tolist(),source_hashes_before=before,source_hashes_after=source_hashes(),tracking_baseline_deg=TRACKING_BASELINE,maximum_planned_xy_m=bound,evaluation_seconds=25.)
    r['source_changed_during_trial']=r['source_hashes_before']!=r['source_hashes_after']
    OUT.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(r,indent=2,default=float))
    print('WAVE3_COST',name,round(r['optimization_cost'],3),list(np.round(x,7)),r.get('gates',{}),flush=True)
    return float(r['optimization_cost'])

if __name__=='__main__':
    AUDIT.mkdir(parents=True,exist_ok=True)
    bounds=[(float(v-.002),float(v+.002)) for v in SEED]
    (AUDIT/'wave3-manifest.json').write_text(json.dumps(dict(seed=SEED.tolist(),bounds=bounds,source_hashes=source_hashes(),baseline_tracking_rms_deg=TRACKING_BASELINE,case_budget=60,steady_window_s=[5,30],stop_at=30,physics_changes=False),indent=2))
    with ProcessPoolExecutor(max_workers=3) as pool:
        result=differential_evolution(evaluate,bounds,popsize=4,maxiter=2,workers=pool.map,updating='deferred',polish=False,seed=12092026,x0=SEED,tol=0,atol=0)
    trials=[]
    for p in OUT.glob('wave3_*.json'):
        data=json.loads(p.read_text());trials.append((float(data.get('optimization_cost',9999)),p.name))
    (AUDIT/'wave3-optimization.json').write_text(json.dumps(dict(x=result.x.tolist(),cost=float(result.fun),evaluations=int(result.nfev),success=bool(result.success),message=str(result.message),ranked=sorted(trials)),indent=2))
