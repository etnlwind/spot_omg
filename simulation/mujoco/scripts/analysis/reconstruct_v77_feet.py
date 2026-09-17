"""Offline CAD reconstruction, not a measurement of real ground clearance."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import argparse,csv,json
import numpy as np
from servo.joint_trace import parse,analyze,signed_target
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
from simulation.mujoco.runtime.standing_pose import SoleKinematics

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('capture',type=Path)
    args=ap.parse_args()
    text=(args.capture/'jointtrace.txt').read_text()
    meta,mapping,commands,_=parse(text)
    _,samples=analyze(text)
    plant=Simulation(load_parameters(parse_args([])))
    kin=SoleKinematics(plant.model,plant.stand_target)
    kin.set_angles(plant.stand_target)
    origin=np.array([kin.foot(i) for i in range(4)])
    rows=[]
    imu=list(csv.DictReader((args.capture/'aligned/imu.csv').open()))
    for c in commands:
        t=c['end']
        target=np.array([(signed_target(c['target'][j])-mapping[j]['center'])*mapping[j]['direction']*360/4096 for j in range(12)])
        target[[0,3]]*=-1
        actual=[]
        for j in range(12):
            sj=[r for r in samples if r['joint']==j]
            ts=[r['time_ms'] for r in sj]
            value=np.interp(t,ts,[r['actual_deg'] for r in sj]) if ts and ts[0]<=t<=ts[-1] else np.nan
            actual.append(-value if j in (0,3) else value)
        kin.set_angles(target)
        feet=np.array([kin.foot(i) for i in range(4)])-origin
        row=dict(time_ms=t)
        for i,name in enumerate(('FL','FR','RL','RR')):
            row[name+'_target_x_mm']=feet[i,0]*1000
            row[name+'_target_z_mm']=feet[i,2]*1000
        if np.isfinite(actual).all():
            kin.set_angles(actual)
            feet=np.array([kin.foot(i) for i in range(4)])-origin
            for i,name in enumerate(('FL','FR','RL','RR')):
                row[name+'_actual_x_mm']=feet[i,0]*1000
                row[name+'_actual_z_mm']=feet[i,2]*1000
            sample=min(imu,key=lambda r:abs(float(r['time_ms'])-t))
            row['roll_deg']=float(sample['roll_deg']);row['pitch_deg']=float(sample['pitch_deg'])
            # Sign sensitivity only: physical IMU-to-CAD mounting calibration
            # and absolute body height have not been measured in this capture.
            gaps=[]
            for rs in (-1,1):
                for ps in (-1,1):
                    roll,pitch=np.radians([rs*row['roll_deg'],ps*row['pitch_deg']])
                    cr,sr,cp,sp=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch)
                    vertical=np.array([-sp,cp*sr,cp*cr])
                    z=[]
                    for i,f in enumerate(kin.feet):
                        world=kin.vertices[i]@kin.data.geom_xmat[f].reshape(3,3).T+kin.data.geom_xpos[f]
                        z.append(float(np.min(world@vertical)))
                    # First diagonal only: FL/RR assumed supports. Their
                    # unequal predicted heights bracket the reference plane.
                    gaps.extend([(z[2]-z[0])*1000,(z[2]-z[3])*1000])
                    if rs==1 and ps==1:
                        row['FR_minus_RL_z_documented_axis_mm']=(z[1]-z[2])*1000
            row['RL_first_swing_tilt_sensitivity_min_mm']=min(gaps)
            row['RL_first_swing_tilt_sensitivity_max_mm']=max(gaps)
        rows.append(row)
    out=args.capture/'cad-reconstruction';out.mkdir(exist_ok=True)
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with (out/'feet.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    result={'note':'Body-level CAD coordinates relative to S. Actual angles interpolated from sequential ~120ms feedback; not measured ground clearance.',
            'first_swing_max_target_pair_z_difference_mm':max(abs(r['FR_target_z_mm']-r['RL_target_z_mm']) for r in rows if r['time_ms']<1000),
            'samples':[min(rows,key=lambda r:abs(r['time_ms']-t)) for t in (150,250,380,500,650,800)]}
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
