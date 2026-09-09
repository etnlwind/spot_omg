"""Test the exact virtual console controller, including entry and stopping."""
import json
import math
import numpy as np
from cad_physics import Simulation
from virtual_robot import RobotController
from search_gait_profiles import physics,RESULTS


def run(profile,scenario,motion,p,model):
    robot=RobotController(Simulation(p,model));robot.select_profile(profile)
    now=0.
    for _ in range(100):robot.tick(now);now+=.02
    start=robot.plant.data.qpos[:3].copy()
    sequence=1;robot.command(f'drive 800 0 {sequence}',now)
    max_tilt=0.;bad=False;stop_reason='';samples=[]
    for i in range(550):
        t=i*.02
        linear,yaw=(800,0)
        if motion=='reverse_forward' and t<4:linear=-600
        elif motion=='turn_forward' and t<4:linear,yaw=0,-400
        elif motion=='arc':linear,yaw=650,150
        if i%10==0 and t<8:
            sequence+=1;robot.command(f'@D {sequence} {linear} {yaw}',now)
        if i==400:
            sequence+=1;robot.command(f'@S {sequence}',now)
        robot.tick(now);now+=.02
        r=robot.plant.row();max_tilt=max(max_tilt,abs(r['roll_deg']),abs(r['pitch_deg']))
        bad=bad or any(not c.endswith('_foot') for c in r['contacts'])
        text=robot.drain().decode()
        if 'stopped' in text:stop_reason=text.strip()
        if i>=100 and i<400:samples.append(r['position_m'])
        if max_tilt>40:break
    r=robot.plant.row();rotation=robot.plant.data.xmat[model.body('robot').id].reshape(3,3)
    return {'profile':profile,'scenario':scenario,'motion':motion,'max_tilt_deg':max_tilt,'nonfoot_contact':bad,
            'safety':robot.safety,'stopped':robot.motion is None and robot.transition is None,
            'stop_response':stop_reason,'displacement_m':(robot.plant.data.qpos[:3]-start).tolist(),
            'yaw_deg':math.degrees(math.atan2(rotation[1,0],rotation[0,0])),
            'moving_speed_m_s':(samples[-1][0]-samples[0][0])/((len(samples)-1)*.02) if len(samples)>1 else 0}


def main():
    rows=[]
    for scenario in ('nominal','heavy_slippery','com_offset'):
        p,m=physics(scenario)
        for profile in ('legacy','crawl','cruise','trot','highstep'):
            for motion in ('straight','turn_forward','reverse_forward','arc'):
                r=run(profile,scenario,motion,p,m);rows.append(r)
                print(scenario,profile,motion,r['safety'],round(r['max_tilt_deg'],1),'contact',r['nonfoot_contact'],'speed',round(r['moving_speed_m_s'],3),flush=True)
                (RESULTS/'runtime.json').write_text(json.dumps(rows,indent=2))


if __name__=='__main__':main()
