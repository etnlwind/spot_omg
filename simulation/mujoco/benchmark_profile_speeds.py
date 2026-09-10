"""Identical 60s maximum-forward benchmark for every app policy."""
import json,math,hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController
from gait_profiles import load_profiles
ROOT=Path(__file__).resolve().parents[2]

def run(profile):
    p,m=physics();p['bno055']={**p.get('bno055',{}),'seed':55}
    r=RobotController(Simulation(p,m));r.select_profile(profile)
    first=last=None;direction=None;count=0;nonfoot=False;peak=0.
    for i in range(3250):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<3100 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==3100:r.command('@S 3200',t)
        r.tick(t);row=r.plant.row();r.drain()
        peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));nonfoot |= any(not n.endswith('_foot') for n in row['contacts'])
        if 250<=i<3100:
            if first is None:
                first=r.plant.data.qpos[:2].copy();direction=r.plant.data.xmat[m.body('robot').id].reshape(3,3)[:2,0].copy();direction/=np.linalg.norm(direction)
            last=r.plant.data.qpos[:2].copy();count+=1
        if r.safety!='ok':break
    passed=r.safety=='ok' and not nonfoot and count==2850 and r.motion is None and r.transition is None
    speed=float(np.dot(last-first,direction)/((count-1)*.02)) if passed else None
    return dict(profile=profile,speed_m_s=speed,passed=passed,safety=r.safety,nonfoot_contact=nonfoot,peak_tilt_deg=peak,observed_steady_samples=count)

if __name__=='__main__':
    result=dict(protocol='nominal estimated MuJoCo physics; 60s maximum forward; steady measurement 5-62s; seed55; successful stop required',manifest_sha256=hashlib.sha256((ROOT/'config/locomotion_profiles.json').read_bytes()).hexdigest(),heading_controller_sha256=hashlib.sha256((ROOT/'firmware/stm32-learning/Inc/heading_control.h').read_bytes()).hexdigest(),profiles={})
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,('legacy',*load_profiles())):
            print(json.dumps(row),flush=True);result['profiles'][row['profile']]=row
            (ROOT/'config/locomotion_speed_benchmarks.json').write_text(json.dumps(result,indent=2)+'\n')
