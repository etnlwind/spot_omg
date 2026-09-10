"""Rightward curvature from small mechanical joint-zero errors, no yaw commands."""
import json,math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController
OUT=Path(__file__).with_name('diagnostics')/'right-drift'

def run(case, duration_s=30, command_linear=1000, profile="jointsport"):
    bias,heading,balance,seed=case
    p,m=physics();p['joint_zero_error_deg']=[0.,0.,0.,0.,bias,0.,0.,0.,0.,0.,bias,0.]
    p['bno055']={**p.get('bno055',{}),'seed':seed}
    r=RobotController(Simulation(p,m));r.select_profile(profile);r.heading.enabled=heading;r.balance.enabled=balance
    frames=[];nonfoot=False;peak=0
    stop_frame=100+round(duration_s/.02)
    for i in range(stop_frame+150):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<stop_frame and i%10==0:r.command(f'@D {i} {command_linear} 0',t)
        if i==stop_frame:r.command(f'@S {stop_frame+100}',t)
        r.tick(t);row=r.plant.row();rot=r.plant.data.xmat[m.body('robot').id].reshape(3,3)
        frames.append(dict(time_s=t,x=float(r.plant.data.qpos[0]),y=float(r.plant.data.qpos[1]),yaw_deg=math.degrees(math.atan2(rot[1,0],rot[0,0])),heading_correction=r.heading.diagnostic()['correction'],roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg']))
        nonfoot |= any(not n.endswith('_foot') for n in row['contacts']);peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));r.drain()
        if r.safety!='ok':break
    a=frames[min(100,len(frames)-1)];b=frames[min(stop_frame,len(frames)-1)];angle=math.radians(a['yaw_deg']);dx=b['x']-a['x'];dy=b['y']-a['y']
    yaw=np.degrees(np.unwrap(np.radians([x['yaw_deg'] for x in frames])))
    summary=dict(right_j2_zero_error_deg=bias,heading=heading,balance=balance,seed=seed,profile=profile,command_linear=command_linear,command_yaw=0,duration_s=duration_s,
      forward_m=dx*math.cos(angle)+dy*math.sin(angle),right_m=dx*math.sin(angle)-dy*math.cos(angle),
      right_turn_deg=float(-(yaw[min(stop_frame,len(frames)-1)]-yaw[min(100,len(frames)-1)])),peak_tilt_deg=peak,safety=r.safety,
      passed=r.safety=='ok' and not nonfoot and r.motion is None and r.transition is None)
    moving=frames[100:min(stop_frame,len(frames))]
    lateral=np.array([(f['x']-a['x'])*math.sin(angle)-(f['y']-a['y'])*math.cos(angle) for f in moving])
    if len(lateral):
        summary.update(path_rms_cm=float(np.sqrt(np.mean(lateral**2))*100),
                       path_max_cm=float(max(abs(lateral))*100))
    OUT.mkdir(parents=True,exist_ok=True);(OUT/f'bias{bias}-h{int(heading)}-b{int(balance)}-seed{seed}.json').write_text(json.dumps(dict(summary=summary,frames=frames),indent=2)+'\n')
    return summary

if __name__=='__main__':
    cases=[(v,False,False,55) for v in (0.,-.1,.1,-.3,.3,-.6,.6)]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,cases))
    print(json.dumps(rows,indent=2));(OUT/'search.json').write_text(json.dumps(rows,indent=2)+'\n')
