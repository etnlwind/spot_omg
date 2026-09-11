"""Sensor-started Cartesian footsteps for position servos, not torque WBC.

Inputs: CAD geometry, phase, requested velocity, delayed encoders and IMU.
Flat-ground stance is a scheduled hypothesis, never a MuJoCo contact oracle.
Joint angles are outputs of the constrained foot-position IK.
"""
import numpy as np
import mujoco
from gait_profiles import foot_targets
from support_shift import SupportShift, OFFSETS, smooth, transfer_gate


def rotation(roll=0.,pitch=0.,yaw=0.):
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


class FootstepTracker:
    def __init__(self,model):
        self.kin=SupportShift(model)
        self.reset()

    def reset(self):
        self.anchors=None;self.base=None;self.previous_base=None
        self.body_command=None;self.velocity=np.zeros(3)
        self.encoders=None;self.healthy=False;self.yaw_origin=None;self.yaw=0.
        self.stance=np.ones(4,dtype=bool);self.starts=np.zeros((4,3));self.ends=np.zeros((4,3))
        self.reference=None;self.params=None;self.diagnostic={};self.last_target=None

    def observe(self,encoders,attitude,reading=None,reset_contacts=False):
        self.healthy=encoders is not None and attitude is not None and not attitude.failures
        if not self.healthy:return
        self.encoders=np.asarray(encoders).copy()
        angles=np.radians(np.asarray(attitude.filtered)/10.)
        self.attitude=angles
        self.angular_rate=np.radians(np.asarray(getattr(attitude,'rate',[0.,0.]))/10.)
        yaw=np.radians((reading or {}).get('yaw_tenths',0)/10.)
        if self.yaw_origin is None:self.yaw_origin=yaw
        self.yaw=float(np.arctan2(np.sin(yaw-self.yaw_origin),np.cos(yaw-self.yaw_origin)))
        self.orientation=rotation(*angles,self.yaw)
        k=self.kin;k.set_angles(self.encoders)
        quat=np.zeros(4);mujoco.mju_mat2Quat(quat,self.orientation.reshape(-1))
        k.data.qpos[3:7]=quat
        mujoco.mj_kinematics(k.model,k.data);mujoco.mj_comPos(k.model,k.data)
        relative=np.array([k.foot(i) for i in range(4)])
        com=np.average(k.data.xipos,axis=0,weights=k.model.body_mass)
        if self.anchors is None or reset_contacts:
            self.base=np.array([0.,0.,-np.mean(relative[:,2])])
            self.anchors=relative+self.base;self.anchors[:,2]=0.
            self.body_command=self.base.copy();self.base_height=self.base[2]
            self.previous_base=None;self.stance[:]=True
        candidates=self.anchors[self.stance]-relative[self.stance]
        estimated=np.mean(candidates,axis=0) if len(candidates) else self.base+.02*self.velocity
        raw=np.zeros(3) if self.previous_base is None else (estimated-self.previous_base)/.02
        # Reject implausible odometry impulses, rather than feeding contact
        # misclassification directly into a new landing location.
        rejected=bool(np.max(abs(raw))>.5)
        if not rejected:self.velocity+=.15*(raw-self.velocity)
        self.previous_base=estimated.copy();self.base=estimated
        self.feet=relative+self.base;self.com_relative=com
        self.diagnostic.update(odometry_outlier=rejected,sensor_source='delayed-imu-quantized-encoders',
                               contact_source='scheduled-flat-ground-hypothesis')

    def plan(self,params,phase,amplitude,linear,yaw,config):
        k=self.kin
        if self.params!=tuple(params):
            self.params=tuple(params)
            neutral=foot_targets(params,0,0).reshape(-1)
            k.set_angles(neutral);self.reference=np.array([k.foot(i) for i in range(4)])
            self.reference[:,2]=np.mean(self.reference[:,2])
            self.last_target=neutral.copy();self.neutral_j1=neutral[::3].copy()
        if not self.healthy or self.anchors is None:
            self.diagnostic['sensor_hold']=True
            return self.last_target.copy()
        period,duty,stride,lift=params[:4]
        local=(phase+OFFSETS)%1;stance=local<duty
        heading=rotation(yaw=self.yaw)
        requested_speed=float(linear*amplitude*stride/period)
        velocity=heading@np.array([requested_speed,0.,0.])
        self.body_command[:2]+=.02*velocity[:2]
        for i in range(4):
            if self.stance[i] and not stance[i]:
                # Actual estimated foot position is latched ONCE at liftoff.
                self.starts[i]=self.feet[i]
                # Body transfer occurs during four-foot overlap. During a
                # diagonal swing its COM reference follows the support line;
                # extrapolating a transient transfer velocity through the
                # entire swing would send the next foothold far past reach.
                future=self.base.copy()
                offset=self.reference[i].copy();offset[2]=0.
                offset[0]+=.5*stride*amplitude*np.clip(linear+(yaw if i%2==0 else -yaw),-1,1)
                self.ends[i]=future+heading@offset;self.ends[i,2]=0.
            elif not self.stance[i] and stance[i]:
                self.anchors[i]=self.feet[i];self.anchors[i,2]=0.
        self.stance=stance
        targets=self.anchors.copy()
        for i in np.flatnonzero(~stance):
            u=(local[i]-duty)/(1-duty)
            targets[i]=self.starts[i]+smooth(u)*(self.ends[i]-self.starts[i])
            targets[i,2]+=lift*amplitude*min(1,abs(linear)+abs(yaw))*64*u**3*(1-u)**3
        # All four legs use the SAME world-vertical arch. The rear path cannot
        # be lowered afterward by a phase-indexed roll compensation table.
        body=self.body_command.copy();shift=np.zeros(3)
        # Use the entire available four-foot overlap for transfer. The original
        # 0.2s phase-table experiment is preserved as a separate profile.
        window=max(.2/period,duty-.5)
        com_offset=self.com_relative
        forward=heading[:2,0];side=heading[:2,1]
        for pair,offset in [((0,3),0.),((1,2),.5)]:
            gate=transfer_gate((phase+offset)%1,duty,window)
            support=[i for i in range(4) if i not in pair]
            a,b=self.anchors[support,:2];line=b-a
            normal=np.array([-line[1],line[0]]);norm=np.linalg.norm(normal)
            if norm<1e-8:continue
            normal/=norm
            com=body[:2]+com_offset[:2]
            axis=line/np.linalg.norm(line)
            height=max(.05,float(self.base[2]+com_offset[2]))
            # Approximate moment balance about the otherwise unactuated
            # diagonal support line. Zero COM distance removes gravity torque
            # but cannot brake existing rotation. Offset the reference to
            # request a restoring gravity moment using delayed angle/rate.
            moment_offset=(height*height/9.81)*(config.get('attitude_kp',20.)*float(axis@self.attitude)+
                                                config.get('attitude_kd',4.)*float(axis@self.angular_rate))
            distance=float(np.dot(normal,com-a)-moment_offset)
            correction=-normal*distance
            dy=float(np.clip(correction@side,-.01,.01))
            # Keep COM on the support line using fore/aft progress as well as
            # the bounded lateral shift; do not pretend two point contacts can
            # independently impose all six floating-body accelerations.
            nx=float(normal@forward)
            dx=float((-distance-dy*(normal@side))/nx) if abs(nx)>.05 else 0.
            shift[:2]+=gate*(dx*forward+dy*side)
            shift[2]-=gate*min(.01,float(config.get('lower_m',.006)))
        body+=shift
        body[2]=self.base_height+shift[2]
        local_targets=(targets-body)@heading
        locked=self.neutral_j1 if config.get('lock_j1',False) else None
        result,residual=k.solve(local_targets,self.encoders,iterations=16,stance=stance,locked_j1=locked)
        self.last_target=result.copy()
        self.diagnostic.update(policy='sensor-started-cartesian-feet',phase=float(phase),sensor_hold=False,
            requested_speed_m_s=requested_speed,estimated_speed_m_s=float(self.velocity@heading[:,0]),
            commanded_stride_m=float(stride),body_shift_m=shift.tolist(),body_reference_m=body.tolist(),
            planned_residual_m=float(residual),unreachable=bool(residual>.001),
            j1_locked=bool(locked is not None),j1_fixed_target_deg=locked.tolist() if locked is not None else None,
            estimated_feet_m=self.feet.tolist(),target_feet_m=targets.tolist(),
            scheduled_stance=stance.tolist(),body_roll_pitch_target_deg=[0.,0.])
        return result
