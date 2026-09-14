"""MuJoCo-only reaction to a small lateral COM/foot-position command.

All truth-state quantities below are diagnostic outputs, not controller input.
"""
import ctypes
import json
from pathlib import Path
import subprocess
import tempfile

import mujoco
import numpy as np

from diagnose_arc_body_axes import PrivateKinematics, make_plant, LEGS, ROOT
from cad_physics import foot_clearance


OUT=ROOT/'artifacts/audits/arc-com-reaction-diagnosis'


class ShiftKinematics:
    def __init__(self):
        self.base=PrivateKinematics()
        self.temp=tempfile.TemporaryDirectory(prefix='arc-com-reaction-')
        source=Path(self.temp.name)/'com.c'
        source.write_text('''#include "arc_turn.h"
int shift_targets(const float nominal[12],const float shift[2],const float lift[4],float out[12]){
 for(int i=0;i<4;i++){
  float q[3]={nominal[3*i],nominal[3*i+1],nominal[3*i+2]},p[3];arc_foot(i,q,p,0);
  p[0]-=shift[0];p[1]-=shift[1];p[2]+=lift[i];
  if(!arc_ik(i,p,q))return 0;for(int j=0;j<3;j++)out[3*i+j]=q[j];
 }return 1;
}
''')
        library=Path(self.temp.name)/'com.dylib'
        subprocess.run(['cc','-shared','-fPIC','-O2','-ffp-contract=off','-I',
                        str(ROOT/'firmware/stm32-learning/Inc'),str(source),'-o',str(library)],check=True)
        self.lib=ctypes.CDLL(str(library));fp=ctypes.POINTER(ctypes.c_float)
        self.lib.shift_targets.argtypes=(fp,fp,fp,fp)
        feet=self.base.feet(self.base.neutral)
        self.t=(feet[0,:2]-feet[3,:2]);self.t/=np.linalg.norm(self.t) # RR -> FL
        self.n=np.array([-self.t[1],self.t[0]])

    def targets(self,shift,lift):
        out=(ctypes.c_float*12)()
        assert self.lib.shift_targets((ctypes.c_float*12)(*self.base.neutral),
            (ctypes.c_float*2)(*shift),(ctypes.c_float*4)(*lift),out)
        return np.array(out)


def run(kin,amplitude_mm,ramp_s):
    plant,sensor,filt=make_plant(kin.base.neutral)
    model,data=plant.model,plant.data
    body=model.body('robot').id;floor=model.geom('floor').id
    feet=[model.geom(leg.lower()+'_foot').id for leg in LEGS]
    rows=[];start=3.44
    for frame in range(225):
        now=frame*.02
        lift=.02*min(1,max(0,(now-3)/.4))
        u=min(1,max(0,(now-start)/ramp_s))
        fraction=u*u*u*(10+u*(-15+6*u))
        shift=amplitude_mm*.001*fraction*kin.n
        target=kin.targets(shift,[0,lift,lift,0])
        plant.step(targets_deg=target,balance=False)
        R=data.xmat[body].reshape(3,3)
        roll=np.arctan2(R[2,1],R[2,2]);pitch=np.arcsin(np.clip(-R[2,0],-1,1))
        velocity=np.zeros(6);mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
        forces=np.zeros(4)
        for index,contact in enumerate(data.contact):
            if floor in (contact.geom1,contact.geom2):
                other=contact.geom2 if contact.geom1==floor else contact.geom1
                if other in feet:
                    force=np.zeros(6);mujoco.mj_contactForce(model,data,index,force)
                    forces[feet.index(other)]+=max(0.,force[0])
        midpoint=.5*(data.geom_xpos[feet[0]]+data.geom_xpos[feet[3]])
        # Ground-height reference, with measured support-foot XY. It is a
        # rolling-inclusive diagnostic proxy, not exact contact CoP.
        midpoint[2]=0
        com=data.subtree_com[body].copy();relative=R.T@(com-midpoint)
        reading=sensor.read(float(data.time));filt.update(reading)
        rows.append(dict(time_s=now,relative_time_s=now-start,roll_deg=float(np.degrees(roll)),
            pitch_deg=float(np.degrees(pitch)),freeaxis_angle_deg=float(np.degrees(np.dot([roll,pitch],kin.t))),
            freeaxis_rate_deg_s=float(np.degrees(np.dot(velocity[:2],kin.t))),
            com_world_m=com.tolist(),support_midpoint_world_m=midpoint.tolist(),
            com_normal_world_mm=float(np.dot(com[:2]-midpoint[:2],kin.n)*1000),
            com_normal_body_mm=float(np.dot(relative[:2],kin.n)*1000),body_com_height_m=float(relative[2]),
            commanded_body_shift_mm=float(amplitude_mm*fraction),contacts=int(sum(forces>.2)),
            forces_n=forces.tolist(),clearance_mm=[1000*foot_clearance(model,data,g) for g in feet],
            imu_roll_deg=reading['roll_tenths']/10 if reading else None,
            imu_pitch_deg=reading['pitch_tenths']/10 if reading else None))
    return rows


def compare(rows,baseline):
    fields=('roll_deg','pitch_deg','freeaxis_angle_deg','freeaxis_rate_deg_s',
            'com_normal_world_mm','com_normal_body_mm')
    result={}
    for name,begin,end in [('20_200ms',.019,.201),('200_600ms',.199,.601),('20_80ms',.019,.081),('80_200ms',.079,.201)]:
        paired=[(a,b) for a,b in zip(rows,baseline) if begin<=a['relative_time_s']<=end]
        result[name]={field:float(np.mean([a[field]-b[field] for a,b in paired])) for field in fields}
        result[name]['contacts_trial']=[a['contacts'] for a,b in paired]
        result[name]['contacts_baseline']=[b['contacts'] for a,b in paired]
    delta=np.array([a['com_normal_body_mm']-b['com_normal_body_mm'] for a,b in zip(rows,baseline)])*.001
    acceleration=np.gradient(np.gradient(delta,.02),.02)
    h=np.array([r['body_com_height_m'] for r in rows])
    gravity_torque=-4.418*9.81*delta
    acceleration_torque=4.418*h*acceleration
    result['diagnostic_acceleration']={
        'note':'Finite differences of measured body-frame COM/support-midpoint displacement; not an exact inverse dynamics decomposition.',
        'records': [dict(relative_time_s=r['relative_time_s'],delta_m=float(d),delta_acceleration_m_s2=float(a),
                         gravity_term_nm=float(g),translation_acceleration_term_nm=float(t))
                    for r,d,a,g,t in zip(rows,delta,acceleration,gravity_torque,acceleration_torque)
                    if -.04<=r['relative_time_s']<=.65]}
    return result


def main():
    OUT.mkdir(parents=True,exist_ok=True);kin=ShiftKinematics()
    baseline=run(kin,0,.1)
    (OUT/'baseline.json').write_text(json.dumps(baseline,indent=2)+'\n')
    summary=dict(estimated_physics=True,hardware_commands=False,
                 tangent_rr_to_fl=kin.t.tolist(),normal_n=kin.n.tolist(),shift_start_s=3.44,cases={})
    for amplitude,ramp in [(2,.02),(2,.1),(2,.2),(-2,.1)]:
        name=f'shift{amplitude:+d}mm-ramp{round(ramp*1000)}ms'
        rows=run(kin,amplitude,ramp)
        (OUT/(name+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
        result=compare(rows,baseline);summary['cases'][name]=result
        (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(name,{k:v for k,v in result.items() if k!='diagnostic_acceleration'},flush=True)


if __name__=='__main__':main()
