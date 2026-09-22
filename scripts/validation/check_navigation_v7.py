"""Run V7 in the actual MuJoCo controller, with watchdog heartbeats and stop."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args

def trial(linear,yaw,heading=True,disturbance=0,stored_feet=False):
    p=load_parameters(parse_args([]));plant=Simulation(p);r=RobotController(plant)
    r.command('gaitprofile attitudepd_v7',0)
    if stored_feet:r.command('footlift save 20 20 20 20 -30 -30 30 30',0)
    r.command('heading '+('on' if heading else 'off'),0)
    r.command('stand',0)
    for _ in range(100):r.tick(float(plant.data.time))
    initial=plant.data.qpos[:3].copy();rows=[]
    r.command(f'drive {linear} {yaw} 1',float(plant.data.time))
    for frame in range(400):
        now=float(plant.data.time)
        plant.data.xfrc_applied[plant.model.body('robot').id,5]=disturbance if frame>=100 else 0
        if frame%10==0:r.command(f'@D {frame+2} {linear} {yaw}',now)
        r.tick(now)
        d=plant.row();rows.append(dict(t=d['time_s'],roll=d['roll_deg'],pitch=d['pitch_deg'],
            yaw=r.imu_reading['yaw_tenths']/10 if r.imu_reading else None,safety=r.safety,
            phase=r.phase,heading=r.heading.diagnostic(),navigation=getattr(r,'navigation_frame',{})))
        if r.motion is None:break
    moved=(plant.data.qpos[:3]-initial).tolist()
    r.command('@S 10000',float(plant.data.time))
    stop_rows=[]
    for _ in range(200):
        r.tick(float(plant.data.time));d=plant.row()
        stop_rows.append(dict(t=d['time_s'],roll=d['roll_deg'],pitch=d['pitch_deg'],safety=r.safety,
            moving=r.motion is not None,transition=r.transition is not None))
    return dict(input=[linear,yaw],heading=heading,disturbance_nm=disturbance,stored_feet=stored_feet,walk_frames=len(rows),displacement_m=moved,
        peak_tilt_deg=max(max(abs(v['roll']),abs(v['pitch'])) for v in rows),
        final_yaw_deg=rows[-1]['yaw'],
        safety=r.safety,stopped=r.motion is None,rows=rows,stop_rows=stop_rows)

if __name__=='__main__':
    dest=ROOT/'artifacts/navigation-v7';dest.mkdir(parents=True,exist_ok=True)
    results=[]
    for x,y in [(588,0),(-588,0),(0,588),(0,-588),(600,400),(600,-400),(-600,400),(-600,-400)]:
        result=trial(x,y);results.append(result)
        print({k:v for k,v in result.items() if k not in ('rows','stop_rows')},flush=True)
    (dest/'simulation.json').write_text(json.dumps(results,indent=2)+'\n')
