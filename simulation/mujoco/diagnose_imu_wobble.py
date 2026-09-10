"""Controlled ablations; temporary C libraries leave deployed policy untouched."""
import ctypes,json,subprocess,tempfile
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController
ROOT=Path(__file__).resolve().parents[2]


def build_variants():
    inc=ROOT/'firmware/stm32-learning/Inc';base=(ROOT/'simulation/mujoco/diagnostics/imu-balance-v1.h').read_text()
    variants={'current':base,'no_rate':base.replace('*.025f','*0.f'),
              'no_j1':base.replace('5.f*controlled.roll','0.f*controlled.roll'),
              'no_placement':base.replace('.03f*controlled.pitch','0.f*controlled.pitch'),
              'no_weight':base.replace('(.65f+.35f*support[i])','1.f'),
              'restrained':base.replace('*.025f','*0.f').replace('5.f*controlled.roll*side*(2.f*support[i]-1.f)','2.f*controlled.roll*side*support[i]').replace('.03f*controlled.pitch','.01f*controlled.pitch').replace('(.65f+.35f*support[i])','(.95f+.05f*support[i])')}
    folder=Path(tempfile.mkdtemp(prefix='spot-imu-ablation-'));paths={}
    for name,source in variants.items():
        d=folder/name;d.mkdir();(d/'balance_control.h').write_text(source);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True)
        paths[name]=str(lib)
    return paths


def run(case):
    name,library,mode=case;p,m=physics();r=RobotController(Simulation(p,m));r.select_profile('lift' if name=='lift' else 'imu')
    if library:r.balance.policy=SimpleNamespace(_library=ctypes.CDLL(library))
    tilts=[];rates=[];peak=0.;bad=False;j1=[]
    for i in range(1100):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<1000 and i%10==0:
            linear,yaw=(1000,0) if mode=='full' else (1000,500) if mode=='arc' else (250,0)
            r.command(f'@D {i} {linear} {yaw}',t)
        if i==1000:r.command('@S 1100',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad |= any(not c.endswith('_foot') for c in row['contacts'])
        if 200<=i<1000:
            tilts.append([row['roll_deg'],row['pitch_deg']]);j1.append(r.balance.correction[::3].tolist())
        r.drain()
    a=np.array(tilts);rate=np.diff(a,axis=0)/.02
    return dict(variant=name,mode=mode,rms_tilt=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),
                rms_rate=float(np.sqrt(np.mean(np.sum(rate*rate,axis=1)))),peak=peak,
                j1_peak=float(np.max(np.abs(j1))),safety=r.safety,contact=bad,
                stopped=r.motion is None and r.transition is None)

if __name__=='__main__':
    paths=build_variants();paths['lift']=None;rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(n,l,m) for m in ('full','arc','slow') for n,l in paths.items()]):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('imu_wobble_ablation.json').write_text(json.dumps(rows,indent=2)+'\n')
