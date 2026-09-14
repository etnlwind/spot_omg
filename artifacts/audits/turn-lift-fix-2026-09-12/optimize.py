import sys,json,hashlib
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'simulation/mujoco'))
import numpy as np
from scipy.optimize import differential_evolution
from concurrent.futures import ProcessPoolExecutor
from search_arc_control import safe_trial
seed=np.array([.001755704,-.000990532,.000780877,.001239549,.001311364])
def evaluate(x):
 name='turnlift_opt_'+hashlib.sha256(np.array(x).tobytes()).hexdigest()[:10]
 r=safe_trial(dict(name=name,stop_at=12,parameters=dict(arc_trial=[.025,.5,.04,0],arc_lift_first_trial=.1,arc_transfer_trial=[.12,0,.3],arc_attitude_trial=[0,0,0,.006,.04,.08],arc_wave_trial=[x[0],0,*x[1:]])))
 if 'metrics' not in r:return 5000
 m=r['metrics'];ls=list(r['contact_metrics']['phase_bases']['target']['legs'].values());peak=[v['swing_peak_clearance_mm']['min'] for v in ls];ct=[v['middle_swing_contact_fraction'] for v in ls]
 if r['safety']!='ok' or m['moving_fraction']<.99 or any(v is None for v in peak+ct):c=1000+100*(1-m['moving_fraction'])
 else:c=10*max(0,m['max_roll_deg']-3)**2+10*max(0,m['max_pitch_deg']-3)**2+sum(max(0,15-p)**2 for p in peak)+400*sum(max(0,v-.1)**2 for v in ct)+10*max(0,17.4-abs(r['rotation_deg_s']))**2+m['rms_roll_deg']
 print('COST',name,c,flush=True);return c
if __name__=='__main__':
 with ProcessPoolExecutor(max_workers=2) as pool:
  r=differential_evolution(evaluate,[(v-.003,v+.003) for v in seed],x0=seed,seed=921,popsize=4,maxiter=2,polish=False,workers=pool.map,updating='deferred')
 Path('/private/tmp/spot-turnlift-optimum.json').write_text(json.dumps(dict(x=r.x.tolist(),cost=r.fun)))
