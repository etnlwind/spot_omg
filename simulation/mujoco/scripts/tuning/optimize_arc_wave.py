"""Offline direct shooting of a bounded periodic body translation, fixed physics."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.optimize import differential_evolution
from simulation.mujoco.scripts.tuning.search_arc_control import trial, OUT

def evaluate(x):
    bx,xc,xs,yc,ys=x
    phi=np.linspace(0,2*np.pi,361)
    if np.max(np.hypot(bx+xc*np.cos(2*phi)+xs*np.sin(2*phi),yc*np.cos(phi)+ys*np.sin(phi)))>.00999:return 2000.
    params=[bx,0,xc,xs,yc,ys]
    name='wave2_'+hashlib.sha256(np.array(x).tobytes()).hexdigest()[:12]
    path=OUT/(name+'.json')
    case=dict(name=name,stop_at=12,parameters=dict(arc_trial=[.022,.5,.04,0],arc_transfer_trial=[.12,0,.3],arc_attitude_trial=[0,0,0,.006,.04,.08],arc_wave_trial=params))
    try:r=json.loads(path.read_text()) if path.exists() else trial(case)
    except (ValueError,RuntimeError):return 2000.
    m=r['metrics'];legs=r['contact_metrics']['phase_bases']['target']['legs'].values()
    if r['safety']!='ok' or m['safety_faults'] or m['moving_fraction']<.999:return 1000+max(m['max_roll_deg'],m['max_pitch_deg'])
    clearance=[v['swing_peak_clearance_mm']['min'] for v in legs]
    contacts=[v['middle_swing_contact_fraction'] for v in legs]
    if any(v is None for v in clearance+contacts):return 1500.
    cost=3*max(m['max_roll_deg'],m['max_pitch_deg'])+4*(m['rms_roll_deg']+m['rms_pitch_deg'])+2*max(0,15-min(clearance))+20*max(contacts)+5*max(0,17.4-abs(r['rotation_deg_s']))
    r['optimization_cost']=float(cost);r['optimization_x']=list(x);path.write_text(json.dumps(r,indent=2,default=float))
    print('COST',round(cost,2),list(np.round(x,5)),flush=True)
    return cost

if __name__=='__main__':
    r=differential_evolution(evaluate,[(-.006,.006),(-.004,.004),(-.004,.004),(-.006,.006),(-.006,.006)],popsize=4,maxiter=4,workers=3,updating='deferred',polish=False,seed=1209,x0=np.zeros(5))
    (OUT/'wave-optimization.json').write_text(json.dumps(dict(x=r.x.tolist(),cost=float(r.fun),evaluations=r.nfev,success=bool(r.success),message=r.message),indent=2))
