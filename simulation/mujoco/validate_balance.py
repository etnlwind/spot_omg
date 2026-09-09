"""A/B closed-loop tests with force/torque disturbances, no state teleportation."""
import json,math,argparse
from pathlib import Path
import numpy as np
from search_gait_profiles import physics
from virtual_robot import RobotController
from cad_physics import Simulation


def run(enabled,profile='cruise',kp=.25,kd=.015,delay=.02,scenario='nominal',moving=True,axis=0,force=.6,slope=0):
    p,m=physics(scenario);p['bno055']={'fusion_delay_s':delay}
    if slope:
        import mujoco
        from cad_physics import build
        p['floor_roll_deg' if axis==0 else 'floor_pitch_deg']=slope
        xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
    r=RobotController(Simulation(p,m));r.balance.enabled=enabled;r.balance.kp=kp;r.balance.kd=kd
    r.select_profile(profile);body=m.body('robot').id
    rows=[];seq=1
    for i in range(900):
        t=i*.02
        if moving and i==100:r.command('drive 700 0 1',t)
        if moving and 100<=i<500 and i%10==0:
            seq+=1;r.command(f'@D {seq} 700 0',t)
        if moving and i==500:seq+=1;r.command(f'@S {seq}',t)
        r.plant.data.xfrc_applied[body,:]=0
        if 5<=t<5.4:r.plant.data.xfrc_applied[body,3+axis]=force
        r.tick(t)
        row=r.plant.row()
        if t>=4:rows.append([t,row['roll_deg'],row['pitch_deg'],row['position_m'][0],r.balance.diagnostic()['max_correction_deg']])
    a=np.array(rows);angle=a[:,1+axis];post=a[:,0]>=5
    return dict(enabled=enabled,profile=profile,kp=kp,kd=kd,delay=delay,scenario=scenario,moving=moving,axis=axis,force=force,slope=slope,
        rms_deg=float(np.sqrt(np.mean(angle[post]**2))),peak_deg=float(max(abs(angle[post]))),
        final_deg=float(angle[-1]),max_correction_deg=float(max(a[:,4])),safety=r.safety,
        stopped=r.motion is None and r.transition is None,position_m=r.plant.data.qpos[:3].tolist())


def execute_case(kwargs):
    return run(**kwargs)


if __name__=='__main__':
    from concurrent.futures import ProcessPoolExecutor
    cases=[dict(enabled=e,moving=m,axis=a,slope=s,force=2)
           for e in (False,True) for m in (False,True)
           for a in (0,1) for s in (0,4)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(execute_case,cases):
            rows.append(row);print(json.dumps(row),flush=True)
    Path(__file__).with_name('balance_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] for r in rows)
