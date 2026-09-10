"""Long full-command level gait verification and isolated IMU-latency comparison."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def run(case):
    profile,scenario,seed=case;p,m=physics(scenario if scenario in ('heavy_slippery','com_offset') else 'nominal')
    p['bno055']={**p.get('bno055',{}),'seed':seed}
    if scenario=='delay60':p['bno055']['fusion_delay_s']=.06
    if scenario=='delay0':p['bno055']['fusion_delay_s']=0.
    r=RobotController(Simulation(p,m));r.select_profile(profile)
    if scenario=='no_j1':
        apply=r.balance.apply
        def no_j1(*args):
            r.balance.profile='cruise'
            return apply(*args)
        r.balance.apply=no_j1
    body=m.body('robot').id;chassis=m.body('cad_base').id
    angles=[];xyz=[];j1=[];actual=[];ticks=[];peak=0;bad=False;first=None
    for i in range(3250):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<3100 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if scenario=='roll_push':r.plant.data.xfrc_applied[body,3]=.25 if 600<=i<650 or 1500<=i<1550 else 0
        if i==3100:r.command('@S 3200',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 250<=i<3100:
            if first is None:first=r.plant.data.qpos[:2].copy()
            last=r.plant.data.qpos[:2].copy();angles.append([row['roll_deg'],row['pitch_deg']]);xyz.append(r.plant.data.xipos[chassis].copy());j1.append(r.balance.correction[::3].tolist());actual.append(row['actual_deg'][::3]);ticks.append(r.plant.servo_ticks[::3])
        r.drain()
    a=np.array(angles);xyz=np.array(xyz);acc=np.diff(xyz,n=2,axis=0)/.02**2;stopped=r.motion is None and r.transition is None
    return dict(profile=profile,scenario=scenario,seed=seed,command=1000,duration_s=60,
                rms_tilt_deg=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),peak_tilt_deg=peak,
                angular_rate_rms_deg_s=float(np.sqrt(np.mean(np.sum((np.diff(a,axis=0)/.02)**2,axis=1)))),
                vertical_acceleration_rms_m_s2=float(np.sqrt(np.mean(acc[:,2]**2))),bounce_mm=float(np.std(xyz[:,2])*1000),
                speed_m_s=float(np.linalg.norm(last-first)/((len(a)-1)*.02)),j1_feedback_peak_deg=float(np.max(np.abs(j1))),
                j1_actual_span_deg=np.ptp(actual,axis=0).tolist(),j1_command_span_deg=(np.ptp(ticks,axis=0)*360/4096).tolist(),j1_distinct_ticks=[len(set(np.array(ticks)[:,i])) for i in range(4)],
                safety=r.safety,nonfoot_contact=bad,stopped=stopped,passed=r.safety=='ok' and not bad and stopped)

if __name__=='__main__':
    cases=[('imu',s,55) for s in ('nominal','delay0')]
    cases += [('level','nominal',seed) for seed in (55,77,101)]
    cases += [('level',s,55) for s in ('delay0','delay60','no_j1','roll_push','heavy_slippery','com_offset')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);Path(__file__).with_name('level_gait_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows if r['profile']=='level')
    assert all(r['rms_tilt_deg']<1 and r['peak_tilt_deg']<2 for r in rows if r['profile']=='level' and r['scenario']=='nominal')
