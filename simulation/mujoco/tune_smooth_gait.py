"""Closed-loop search: unchanged mass, torque, contacts, sensor and servo dynamics."""
import json,math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def measure(args):
    params,scenario,duration=args[:3]
    p,m=physics(scenario);r=RobotController(Simulation(p,m))
    r.profiles['cruise']['params']=params
    if len(args)>3:r.profiles['cruise']['turn_reverse_params']=args[3]
    data=[];peak=0.;bad=False
    for i in range(round((duration+2)/.02)):
        t=i*.02
        if i==100:r.command('drive 1000 0 1',t)
        elif i>100 and i%10==0:r.command(f'@D {i} 1000 0',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad=bad or any(not c.endswith('_foot') for c in row['contacts'])
        if r.safety!='ok':break
        if t>=5:
            data.append([t,*r.plant.data.qpos[:3],float(r.plant.data.qvel[0]),row['roll_deg'],row['pitch_deg'],float(r.plant.data.qvel[2])])
    a=np.array(data)
    if len(a)<2:return dict(params=params,scenario=scenario,safety=r.safety,score=-10)
    speed=(a[-1,1]-a[0,1])/(a[-1,0]-a[0,0]);side=(a[-1,2]-a[0,2])/(a[-1,0]-a[0,0])
    tilt=float(np.sqrt(np.mean(a[:,5]**2+a[:,6]**2)))
    ripple=float(np.std(a[:,4]));bounce=float(np.std(a[:,7]))
    R=r.plant.data.xmat[m.body('robot').id].reshape(3,3)
    yaw=math.degrees(math.atan2(R[1,0],R[0,0]))
    eligible=r.safety=='ok' and not bad and abs(yaw)<8 and abs(side)<.025 and peak<8
    score=speed-1.0*ripple-.03*tilt-.4*bounce-.4*abs(side)
    return dict(params=params,scenario=scenario,safety=r.safety,eligible=eligible,speed=float(speed),side=float(side),
                tilt_rms=tilt,speed_ripple=ripple,bounce=bounce,yaw=yaw,peak=peak,score=float(score) if eligible else -10)


if __name__=='__main__':
    baseline=[1.05,.6,.08,.012,.20175,-.035,.75]
    trials=[baseline]+[[period,duty,stride,lift,.20175,-.035,.75]
        for period in (.65,.8,.95) for duty in (.58,.64)
        for stride in (.065,.085) for lift in (.007,.010)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(measure,[(p,'nominal',14) for p in trials]):
            rows.append(row);print(json.dumps(row),flush=True)
    Path(__file__).with_name('smooth_gait_search.json').write_text(json.dumps(rows,indent=2)+'\n')
