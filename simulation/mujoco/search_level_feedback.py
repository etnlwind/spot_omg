"""Evaluate meaningful J1 feedback with the smoother cruise foot trajectory."""
import ctypes,json,subprocess,tempfile,copy
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController
ROOT=Path(__file__).resolve().parents[2]

def run(case):
    gain,library,period,height=case
    p,m=physics();r=RobotController(Simulation(p,m));r.select_profile('imu')
    r.profiles['imu']=copy.deepcopy(r.profiles['cruise']);r.profiles['imu']['params'][0]=period;r.profiles['imu']['params'][3]=height
    r.balance.policy=SimpleNamespace(_library=ctypes.CDLL(library))
    a=[];xyz=[];j1=[];cmd=[];ticks=[];peak=0;bad=False;first=None;body=m.body('robot').id;chassis=m.body('cad_base').id
    for i in range(1200):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<1100 and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==1100:r.command('@S 1200',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 250<=i<1100:
            if first is None:first=r.plant.data.qpos[:2].copy()
            last=r.plant.data.qpos[:2].copy();a.append([row['roll_deg'],row['pitch_deg']]);xyz.append(r.plant.data.xipos[chassis].copy());j1.append(r.balance.correction[::3].tolist());cmd.append(np.degrees(r.plant.desired)[::3]);ticks.append(r.plant.servo_ticks[::3])
        r.drain()
    a=np.array(a);xyz=np.array(xyz);acc=np.diff(xyz,n=2,axis=0)/.02**2
    return dict(gain=gain,period=period,height=height,rms_tilt=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),peak=peak,
                rate_rms=float(np.sqrt(np.mean(np.sum((np.diff(a,axis=0)/.02)**2,axis=1)))),
                vertical_accel_rms=float(np.sqrt(np.mean(acc[:,2]**2))),bounce_mm=float(np.std(xyz[:,2])*1000),
                speed=float(np.linalg.norm(last-first)/((len(a)-1)*.02)),j1_peak=float(np.max(np.abs(j1))),
                j1_command_span=np.ptp(cmd,axis=0).tolist(),ticks=[len(set(np.array(ticks)[:,i])) for i in range(4)],
                safety=r.safety,contact=bad,stopped=r.motion is None and r.transition is None)

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(ROOT/'simulation/mujoco/diagnostics/imu-balance-v18.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-level-feedback-'));paths={}
    for gain in (-30,0,15,30,57):
        d=folder/str(gain);d.mkdir();s=base.replace('.5f*input->roll*side*(.9f+.1f*support[i])',f'{gain}.f*input->roll*side');(d/'balance_control.h').write_text(s);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True);paths[gain]=str(lib)
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(g,p,period,height) for period,height in ((.8,.007),(1.,.007)) for g,p in paths.items()]):
            rows.append(row);print(json.dumps(row),flush=True);Path(__file__).with_name('level_feedback_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
