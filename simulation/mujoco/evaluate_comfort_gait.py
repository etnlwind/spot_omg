"""Body/deck motion and effective J1 commands, not just a no-fall check."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def run(case):
    name,params,family=case
    p,m=physics();r=RobotController(Simulation(p,m));r.select_profile(name if params is None else 'crawl')
    if params is not None:
        r.profiles['crawl']['params']=params;r.profiles['crawl']['family']=family
    angles=[];deck=[];actual=[];commands=[];ticks=[];feedback=[];peak=0.;bad=False;start=None
    body=m.body('robot').id;chassis=m.body('cad_base').id
    for i in range(1300):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<1200 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==1200:r.command('@S 1300',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 250<=i<1200:
            if start is None:start=r.plant.data.qpos[:2].copy()
            last=r.plant.data.qpos[:2].copy()
            angles.append([row['roll_deg'],row['pitch_deg']])
            deck.append((r.plant.data.xipos[chassis]+r.plant.data.xmat[body].reshape(3,3)@np.array([0.,0.,.08])).tolist())
            actual.append(row['actual_deg'][::3]);commands.append(np.degrees(r.plant.desired)[::3].tolist());ticks.append(r.plant.servo_ticks[::3]);feedback.append(r.balance.correction[::3].tolist())
        r.drain()
    a=np.array(angles);xyz=np.array(deck);acc=np.diff(xyz,n=2,axis=0)/.02**2
    return dict(name=name,params=params,family=family,safety=r.safety,nonfoot_contact=bad,stopped=r.motion is None and r.transition is None,
                speed_m_s=float(np.linalg.norm(last-start)/((len(a)-1)*.02)),peak_tilt_deg=peak,
                rms_tilt_deg=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),
                rms_angular_rate_deg_s=float(np.sqrt(np.mean(np.sum((np.diff(a,axis=0)/.02)**2,axis=1)))),
                vertical_accel_rms=float(np.sqrt(np.mean(acc[:,2]**2))),horizontal_accel_rms=float(np.sqrt(np.mean(np.sum(acc[:,:2]**2,axis=1)))),
                body_bounce_mm=float(np.std(xyz[:,2])*1000),j1_feedback_peak_deg=float(np.max(np.abs(feedback))),
                j1_command_span_deg=np.ptp(commands,axis=0).tolist(),j1_actual_span_deg=np.ptp(actual,axis=0).tolist(),
                j1_unique_ticks=[len(set(np.array(ticks)[:,i])) for i in range(4)])

if __name__=='__main__':
    cases=[(name,None,None) for name in ('imu','lift','crawl','cruise')]
    cases += [(f'crawl_{period}_{stride}_{height}',[period,.85,stride,height,.20175,-.035,1.5],'crawl') for period,stride,height in ((2.,.04,.008),(2.,.03,.008),(2.4,.04,.008),(1.8,.03,.01))]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('comfort_gait_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
