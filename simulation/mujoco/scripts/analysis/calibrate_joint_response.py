"""Fit effective joint-type PD response, never manufacturer torque or exact latency.
Forward is training, left selects candidates, right is held out until selection.
Mass/contact are fixed estimates. All evaluated candidates are persisted.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import argparse, json, itertools
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from simulation.mujoco.scripts.visualization.replay_joint_trace import replay

ROOT=REPO_ROOT
DATA=ROOT/'artifacts/audits/joint-capability-2026-09-12/hardware'
OUT=ROOT/'artifacts/audits/measured-gait-2026-09-12'

def evaluate(task):
    direction,override=task
    paths={'forward':'forward-02','left':'left-02','right':'right-01'}
    result,rows=replay(DATA/paths[direction]/'trace.log',parameter_overrides={**override,'timestep_s':.001})
    e=np.array([r['sim_minus_hardware_deg'] for r in rows if r['time_ms']>=1000])
    return {'direction':direction,'overrides':override,'rmse_deg':float(np.sqrt(np.mean(e**2))),'joint_rmse':result['joints']}

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    # Coarse effective response models: no independently unidentifiable per-servo parameters.
    candidates=[{}]
    for kp,kd in itertools.product((15.,25.,45.),(.8,1.6)):
        candidates.append({'servo_kp':[35.,kp,kp]*4,'servo_kd':[.8,kd,kd]*4})
    candidates += [{'servo_kp':[35.,35.,k]*4,'servo_kd':[.8,.8,d]*4} for k,d in ((15.,.8),(25.,1.6),(35.,1.6),(45.,1.6))]
    trials=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for result in pool.map(evaluate,[('forward',c) for c in candidates]):
            trials.append(result);print('train',len(trials),round(result['rmse_deg'],3),flush=True)
            (OUT/'calibration-trials.json').write_text(json.dumps(trials,indent=2))
        finalists=sorted(trials,key=lambda r:r['rmse_deg'])[:3]
        if trials[0] not in finalists:finalists.append(trials[0])
        validation=list(pool.map(evaluate,[('left',r['overrides']) for r in finalists]))
        baseline=next(r for r in validation if not r['overrides'])
        eligible=[(tr,va) for tr,va in zip(finalists,validation) if va['rmse_deg']<=baseline['rmse_deg'] and tr['rmse_deg']<=trials[0]['rmse_deg']]
        selected=min(eligible,key=lambda pair:pair[0]['rmse_deg']+pair[1]['rmse_deg'])[0]
        heldout=list(pool.map(evaluate,[('right',{}),('right',selected['overrides'])]))
    accepted=bool(selected['overrides']) and heldout[1]['rmse_deg']<=heldout[0]['rmse_deg']
    result=dict(training=trials,validation=validation,heldout=heldout,selected=selected['overrides'],accepted=accepted,
        limitations=['Effective PD response conditional on estimated mass/contact; not measured internal gains.',
        '240ms sampling cannot identify exact delay or peak velocity. 20ms nominal delay retained.',
        'No per-servo torque calibration; torque and speed limits unchanged.'])
    (OUT/'calibration.json').write_text(json.dumps(result,indent=2))
    from simulation.mujoco.scripts.tuning.search_gait_profiles import physics
    p,_=physics();p['foot_cushion']=json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    p.update(selected['overrides'] if accepted else {})
    p['empirical_fit_status']='effective response fitted on forward, selected on left, checked on right' if accepted else 'candidate rejected; original parameters retained'
    (OUT/'plant.json').write_text(json.dumps(p,indent=2))
    print(json.dumps({'selected':result['selected'],'accepted':accepted,'heldout_rmse':[r['rmse_deg'] for r in heldout]},indent=2),flush=True)
if __name__=='__main__':main()
