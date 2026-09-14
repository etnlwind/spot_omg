"""CAD torso-centered pivot IK. Private kinematics only; no plant pose/contact input."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
from simulation.mujoco.runtime.gait_profiles import foot_targets, smooth
from simulation.mujoco.runtime.support_shift import SupportShift

class PivotTurn:
    def __init__(self,model):
        self.kin=SupportShift(model);self.key=None;self.diagnostic={}

    def set_entry(self,angles):
        self.entry_angles=np.asarray(angles).copy()
        self.kin.set_angles(angles)
        self.entry_feet=np.array([self.kin.foot(i) for i in range(4)])

    def goals(self,params,phase,amplitude,yaw,config):
        sweep=float(config.get('sweep_rad',.24));lift=float(config.get('lift_m',.024))
        if not np.isfinite([phase,amplitude,yaw,sweep,lift]).all() or not .05<=sweep<=.6 or not .01<=lift<=.05 or not 0<=amplitude<=1 or abs(yaw)>1:
            raise ValueError('Invalid torso-centered pivot')
        key=tuple(params)
        if self.key!=key:
            self.neutral=self.entry_angles.copy() if config.get('entry_arc',False) and hasattr(self,'entry_angles') else foot_targets(params,0,0)
            self.kin.set_angles(self.neutral)
            self.reference=np.array([self.kin.foot(i) for i in range(4)])
            if not config.get('entry_arc',False):self.reference[:,2]=np.mean(self.reference[:,2])
            self.center=self.kin.data.xipos[self.kin.model.body('cad_base').id].copy()
            self.key=key
        duty=params[1];points=self.reference.copy()
        offsets=np.asarray(config.get('offsets',[0,.5,.5,0]),dtype=float)
        if offsets.shape!=(4,) or not np.isfinite(offsets).all() or np.any(offsets<0) or np.any(offsets>=1):raise ValueError('Invalid pivot leg phases')
        for i,offset in enumerate(offsets):
            q=(phase+offset)%1;z=0.
            if q<duty:x=.5-q/duty
            else:
                u=(q-duty)/(1-duty);k=(1-duty)/duty
                x=-.5+(1+k)*smooth(u)-k*u;z=64*u**3*(1-u)**3
            a=-amplitude*yaw*sweep*x;c,s=np.cos(a),np.sin(a)
            radius=self.reference[i,:2]-self.center[:2]
            points[i,:2]=self.center[:2]+np.array([[c,-s],[s,c]])@radius
            points[i,2]+=amplitude*abs(yaw)*lift*z
        return points

    def plan(self,params,phase,amplitude,yaw,config,nominal=None):
        if config.get('mode')=='center_correction':
            if nominal is None:raise ValueError('Pivot correction needs a nominal trajectory')
            shift=np.asarray(config.get('center_shift_m',[0.,0.]),dtype=float)
            sweep=float(config.get('sweep_rad',.24))
            if shift.shape!=(2,) or not np.isfinite(shift).all() or np.max(abs(shift))>.08 or not .05<=sweep<=.6:
                raise ValueError('Invalid pivot center correction')
            self.kin.set_angles(nominal);points=np.array([self.kin.foot(i) for i in range(4)])
            self.center=self.kin.data.xipos[self.kin.model.body('cad_base').id].copy()
            if config.get('retain_entry_foot_xy',False) and hasattr(self,'entry_feet'):
                self.kin.set_angles(foot_targets(params,0,0))
                neutral_feet=np.array([self.kin.foot(i) for i in range(4)])
                points[:,:2]+=self.entry_feet[:,:2]-neutral_feet[:,:2]
            for i,offset in enumerate((0,.5,.5,0)):
                q=(phase+offset)%1;duty=params[1]
                if q<duty:x=.5-q/duty
                else:
                    u=(q-duty)/(1-duty);x=-.5+(1+(1-duty)/duty)*smooth(u)-(1-duty)/duty*u
                angle=-amplitude*yaw*sweep*x;c,s=np.cos(angle),np.sin(angle)
                points[i,:2]+=(np.eye(2)-np.array([[c,-s],[s,c]]))@shift
            seed=nominal
        else:
            points=self.goals(params,phase,amplitude,yaw,config);seed=self.neutral
        result,residual=self.kin.solve(points,seed,iterations=12,stance=np.array([(phase+o)%1<params[1] for o in config.get('offsets',[0,.5,.5,0])]))
        self.diagnostic={'center_body_m':self.center.tolist(),'residual_m':residual,'source':'CAD commanded kinematics; no contact oracle'}
        if residual>.001:raise ValueError('Torso-centered pivot unreachable')
        return result
