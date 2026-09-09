"""V15/V16 preparation, steering, reverse and stop on estimated STEP dynamics.
Shared C command policy. 0.5ms physics, IMU feedback OFF; not full firmware.
"""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from cad_physics import Simulation,KEYS,build
from optimize_cad_gait import make_scenario


def evaluate(p,m,sequence,walk_stance):
    s=Simulation(p,m);rows=[]
    policy=s.policy.drive_walk_targets if walk_stance else s.policy.drive_targets
    def step(*args,**kwargs):
        s.step(*args,balance=False,**kwargs);r=s.row();rows.append(r)
        if max(abs(r['roll_deg']),abs(r['pitch_deg']))>40:raise RuntimeError('TILT_LIMIT')
        if any(not c.endswith('_foot') for c in r['contacts']):raise RuntimeError('NON_FOOT_CONTACT')
    try:
        for _ in range(100):step()
        old=np.degrees(s.data.qpos[s.q]).copy();neutral,_=policy(0,0,0,0);neutral=np.array([neutral[k] for k in KEYS])
        for i in range(50):step(targets_deg=old+(neutral-old)*s.policy.smootherstep((i+1)/50))
        for _ in range(15):step(targets_deg=neutral)
        start=len(rows)
        for i in range(600):
            if sequence=='straight':linear,yaw=1,0
            elif sequence=='left_forward':linear,yaw=(0,-1) if i<200 else (1,0)
            else:linear,yaw=(-1,0) if i<200 else (1,0)
            yaw=s.policy.drive_yaw_limit(round(yaw*1000))/1000
            lin=float(s.linear+np.clip(linear-s.linear,-.04,.04));rot=float(s.yaw+np.clip(yaw-s.yaw,-.04,.04))
            targets,_=policy(s.phase,s.policy.smootherstep(min(1,i*.02/.7)),lin,rot)
            step(linear,yaw,targets_deg=[targets[k] for k in KEYS])
        for _ in range(25):
            lin=float(s.linear+np.clip(-s.linear,-.04,.04));rot=float(s.yaw+np.clip(-s.yaw,-.04,.04))
            targets,_=policy(s.phase,1,lin,rot)
            step(0,0,targets_deg=[targets[k] for k in KEYS])
        old=np.degrees(s.data.qpos[s.q]).copy();stand=np.tile([0,45,90],4)
        for i in range(50):step(targets_deg=old+(stand-old)*s.policy.smootherstep((i+1)/50))
        for _ in range(50):step(targets_deg=stand)
        status='UPRIGHT'
    except (RuntimeError,ValueError) as error:status=str(error)
    return dict(walk_stance=walk_stance,sequence=sequence,status=status,elapsed_s=s.data.time,
        peak_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows),
        peak_tracking_error_deg=max(r['max_tracking_error_deg'] for r in rows),
        final_actual_deg=rows[-1]['actual_deg'],final_contacts=rows[-1]['contacts'])


def main():
    ap=argparse.ArgumentParser(__doc__);ap.add_argument('--output',type=Path,default=Path('/private/tmp/spot-v16-validation'));args=ap.parse_args();out=[]
    for scene in ('nominal','heavy_slippery','com_offset'):
        p,_=make_scenario(scene);p['timestep_s']=.0005;xml,_=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
        for sequence in ('straight','left_forward','reverse_forward'):
            for enabled in (False,True):
                r=evaluate(p,m,sequence,enabled);r['scenario']=scene;out.append(r);print(json.dumps(r),flush=True)
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
