"""60 seconds at full forward command, including stop and sensor variations."""
import ctypes,json
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController
from diagnose_imu_wobble import build_variants


def run(case):
    name,seed,scenario,library=case
    p,m=physics('nominal' if scenario=='delay60' else scenario)
    p['bno055']={**p.get('bno055',{}),'seed':seed}
    if scenario=='delay60':p['bno055']['fusion_delay_s']=.06
    r=RobotController(Simulation(p,m));r.select_profile('lift' if name=='lift' else 'imu')
    if library:r.balance.policy=SimpleNamespace(_library=ctypes.CDLL(library))
    angles=[];peak=0.;bad=False;first=None;last=None
    for i in range(3250):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<3100 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==3100:r.command('@S 3200',t)
        r.tick(t);row=r.plant.row()
        peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 250<=i<3100:
            if first is None:first=r.plant.data.qpos[:2].copy()
            last=r.plant.data.qpos[:2].copy()
            angles.append([row['roll_deg'],row['pitch_deg']])
        r.drain()
    a=np.array(angles);rate=np.diff(a,axis=0)/.02;stopped=r.motion is None and r.transition is None
    return dict(variant=name,seed=seed,scenario=scenario,command=1000,command_duration_s=60,
                rms_tilt_deg=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),
                rms_rate_deg_s=float(np.sqrt(np.mean(np.sum(rate*rate,axis=1)))),
                peak_tilt_deg=peak,speed_m_s=float(np.linalg.norm(last-first)/((len(a)-1)*.02)),
                safety=r.safety,nonfoot_contact=bad,stopped=stopped,passed=r.safety=='ok' and not bad and stopped)

if __name__=='__main__':
    old=build_variants()['current']
    cases=[('original',55,'nominal',old),('lift',55,'nominal',None)]
    cases += [('fixed',seed,'nominal',None) for seed in (55,77,101)]
    cases += [('fixed',55,s,None) for s in ('delay60','heavy_slippery','com_offset')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('imu_full_speed_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['passed'] for r in rows if r['variant']=='fixed')
