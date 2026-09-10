"""Compare equal trajectories with old PI versus phase-aware delayed-IMU feedback."""
import json, math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def run(case):
    profile,mode,scenario=case
    p,m=physics('nominal' if scenario in ('roll_push','pitch_push','delay') else scenario)
    if scenario=='delay':p['bno055']={'fusion_delay_s':.06}
    r=RobotController(Simulation(p,m));r.select_profile(profile)
    body=m.body('robot').id
    peak=0.;bad=False;tilts=[];j1=0.;correction=0.;start=None
    for i in range(1200):
        t=i*.02
        if i==100:
            start=r.plant.data.qpos[:2].copy();r.command('drive 0 0 1',t)
        if 100<=i<1100 and i%10==0:
            linear,yaw=800,0
            if mode=='pivot':linear,yaw=0,500
            elif mode=='arc':linear,yaw=700,500
            elif mode=='reverse':linear,yaw=-500,-300
            elif mode=='turn_forward' and (i-100)%400<200:linear,yaw=0,-500
            r.command(f'@D {i} {linear} {yaw}',t)
        if scenario in ('roll_push','pitch_push'):
            axis=3 if scenario=='roll_push' else 4
            r.plant.data.xfrc_applied[body,axis]=.25 if 350<=i<400 or 650<=i<700 else 0
        if i==1100:r.command('@S 1200',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 200<=i<1100:tilts.append(row['roll_deg']**2+row['pitch_deg']**2)
        j1=max(j1,float(max(abs(r.balance.correction[::3]))));correction=max(correction,float(max(abs(r.balance.correction))))
        r.drain()
    stopped=r.motion is None and r.transition is None
    return dict(profile=profile,mode=mode,scenario=scenario,peak_tilt_deg=peak,
                rms_tilt_deg=float(np.sqrt(np.mean(tilts))),j1_correction_deg=j1,max_correction_deg=correction,
                displacement_m=float(np.linalg.norm(r.plant.data.qpos[:2]-start)),
                safety=r.safety,nonfoot_contact=bad,stopped=stopped,passed=r.safety=='ok' and not bad and stopped)


if __name__=='__main__':
    cases=[(p,mode,'nominal') for mode in ('forward','pivot','arc','reverse','turn_forward') for p in ('lift','imu')]
    cases += [(p,'forward',s) for s in ('roll_push','pitch_push','delay','heavy_slippery','com_offset') for p in ('lift','imu')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('imu_policy_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows if r['profile']=='imu')
