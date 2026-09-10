"""A/B heading hold with a persistent yaw disturbance; truth is metrics only."""
import json, math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def run(case):
    profile, enabled, torque = case
    p,m=physics();r=RobotController(Simulation(p,m));r.select_profile(profile)
    r.heading.enabled=enabled;angles=[];peak=0.;contact=False;corrections=[]
    body=m.body('robot').id
    for i in range(1700):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t);start=r.plant.data.qpos[:2].copy()
        if 100<=i<1600 and i%10==0:r.command(f'@D {i} 800 0',t)
        if i>=250:r.plant.data.xfrc_applied[body,5]=torque
        if i==1600:r.command('@S 1700',t);r.plant.data.xfrc_applied[body,5]=0
        if i>1600:r.plant.data.xfrc_applied[body,5]=0
        r.tick(t);rot=r.plant.data.xmat[body].reshape(3,3)
        angles.append(math.atan2(rot[1,0],rot[0,0]));row=r.plant.row()
        peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        contact |= any(not c.endswith('_foot') for c in row['contacts'])
        corrections.append(r.heading.diagnostic()['correction']);r.drain()
    angles=np.degrees(np.unwrap(angles));errors=angles[250:1600]-angles[160]
    return dict(profile=profile,enabled=enabled,torque_nm=torque,final_heading_deg=float(angles[1600]-angles[160]),
       rms_heading_deg=float(np.sqrt(np.mean(errors**2))),peak_heading_deg=float(max(abs(errors))),
       displacement_m=(r.plant.data.qpos[:2]-start).tolist(),peak_tilt_deg=peak,safety=r.safety,
       nonfoot_contact=contact,stopped=r.motion is None and r.transition is None,
       max_correction=float(max(abs(np.array(corrections)))))


if __name__=='__main__':
    cases=[(p,on,torque) for p in ('cruise','trot') for torque in (-.1,.1) for on in (False,True)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('heading_hold_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['safety']=='ok' and not r['nonfoot_contact'] and r['stopped'] for r in rows)
    for off,on in zip(rows[::2],rows[1::2]):
        assert on['rms_heading_deg'] < off['rms_heading_deg']*.5
