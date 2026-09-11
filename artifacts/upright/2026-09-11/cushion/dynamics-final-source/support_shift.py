"""CAD/cushion-aware coordinated feet with bounded anticipatory body transfer.

Only nominal geometry, gait phase, and delayed sensor packets enter control.
Simulator truth is reserved for the independent evaluator.
"""
import math
import numpy as np
import mujoco
from gait_profiles import foot_targets

LEGS=('fl','fr','rl','rr')
OFFSETS=np.array([0.,.5,.5,0.])

def smooth(x):
    x=np.clip(x,0,1);return x*x*x*(10+x*(-15+6*x))


def swing_path(q,duty,push_bias=0.):
    if not math.isfinite(push_bias) or not 0<=push_bias<=12:
        raise ValueError('push_bias must be finite and between 0 and 12')
    if q<duty:
        u=q/duty
        # Redistribute stance travel into its second half without changing
        # stride endpoints or position/velocity/acceleration at either boundary.
        # J2/J3 are still solved from the Cartesian foot target.
        return .5-u+push_bias*u**3*(1-u)**3,0.
    u=(q-duty)/(1-duty);k=(1-duty)/duty
    return -.5+(1+k)*smooth(u)-k*u,64*u**3*(1-u)**3


def transfer_gate(q,duty,window):
    if q<window:return 1-smooth(q/window)
    if q<duty-window:return 0.
    if q<duty:return smooth((q-duty+window)/window)
    return 1.


class SupportShift:
    def __init__(self,model):
        self.model=model;self.data=mujoco.MjData(model)
        self.q=np.array([model.jnt_qposadr[model.joint(f'{l}_j{j}').id] for l in LEGS for j in (1,2,3)])
        self.v=np.array([model.jnt_dofadr[model.joint(f'{l}_j{j}').id] for l in LEGS for j in (1,2,3)])
        self.feet=[model.geom(l+'_foot').id for l in LEGS]
        self.vertices=[]
        for f in self.feet:
            mesh=model.geom_dataid[f]
            if model.geom_type[f]!=mujoco.mjtGeom.mjGEOM_MESH:raise ValueError('Support shift requires measured cushion mesh')
            a=model.mesh_vertadr[mesh];n=model.mesh_vertnum[mesh]
            self.vertices.append(model.mesh_vert[a:a+n].copy())
        self.reference=None;self.key=None;self.correction=np.zeros(12);self.integral=np.zeros(2)
        self.diagnostic={};self.last_phase=0.;self.height_error_filtered=None;self.height_correction=0.
        self.load_correction=np.zeros(12)
        self.support_offset_y=0.
        self.dynamics_controller=None

    def copy_front_swing(self,points,config,displacements=False):
        """Same-phase diagonal front-to-rear X/Z path, relative to each J1.

        Do this AFTER attitude compensation: copying before it was ineffective,
        because roll compensation subsequently lowered the opposite rear foot.
        Stance feet and lateral contact tasks remain unchanged. The smooth
        swing window preserves liftoff/touchdown position and velocity.
        """
        points=points.copy()
        if config.get('rear_swing_from_front',False):
            for rear,front in ((2,1),(3,0)):
                weight=self.swing_weights[rear]
                for axis in (0,2):
                    same_path=points[front,axis]
                    if not displacements:same_path+=self.shoulders[rear,axis]-self.shoulders[front,axis]
                    points[rear,axis]+=weight*(same_path-points[rear,axis])
        return points

    def reset(self):
        self.correction[:]=0;self.integral[:]=0;self.height_error_filtered=None;self.height_correction=0.
        self.load_correction[:]=0
        self.support_offset_y=0.
        if self.dynamics_controller is not None:self.dynamics_controller.reset()

    def load_preload(self,encoders,params,config):
        """Experimental static load estimate mapped to position-servo preload.

        Uses private CAD data and scheduled support, never simulator contact
        forces. Equal vertical load sharing (or COM projection onto the support
        line) is an approximation, not a feasible six-dimensional wrench solution. Servo stiffness is an estimate.
        """
        gain=float(config.get('load_preload_gain',0.))
        stiffness=float(config.get('estimated_servo_kp',35.))
        if not np.isfinite([gain,stiffness]).all() or not 0<=gain<=1 or stiffness<=0:
            raise ValueError('Invalid static preload gain or servo stiffness')
        if gain==0:
            self.load_correction[:]=0
            return self.load_correction.copy()
        self.set_angles(encoders)
        m,d=self.model,self.data
        d.qvel[:]=0;d.qacc[:]=0
        mujoco.mj_comVel(m,d)
        bias=np.zeros(m.nv);mujoco.mj_rne(m,d,0,bias)
        support=((self.last_phase+OFFSETS)%1)<params[1]
        external=np.zeros(m.nv)
        total_force=-m.opt.gravity*float(m.body_mass.sum())
        support_ids=np.flatnonzero(support)
        shares=np.zeros(4);shares[support]=1/max(1,int(support.sum()))
        if config.get('load_share')=='com_projection' and len(support_ids)==2:
            a,b=[self.foot(i)[:2] for i in support_ids]
            center=np.average(d.xipos,axis=0,weights=m.body_mass)[:2]
            along=b-a
            fraction=float(np.clip(np.dot(center-a,along)/max(1e-9,np.dot(along,along)),.1,.9))
            shares[support_ids]=[1-fraction,fraction]
        for i in np.flatnonzero(support):
            f=self.feet[i]
            world=self.vertices[i]@d.geom_xmat[f].reshape(3,3).T+d.geom_xpos[f]
            # Actual lowest mesh vertex, not the hybrid XY-center/Z-bottom IK
            # point. The same point supplies all three Jacobian rows.
            point=world[np.argmin(world[:,2])]
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[f]))
            external+=jp.T@(total_force*shares[i])
        torque=(bias-external)[self.v]
        requested=np.degrees(gain*torque/stiffness)
        if config.get('lock_j1',False):requested[::3]=0.
        bounded=np.clip(requested,-4.,4.)
        self.load_correction+=np.clip(bounded-self.load_correction,-.25,.25)
        self.diagnostic.update(load_torque_estimate_nm=torque.tolist(),
            load_preload_deg=self.load_correction.tolist(),
            load_preload_clipped=bool(np.any(abs(requested)>4.)),
            load_shares=shares.tolist(),
            load_root_wrench_residual=(bias-external)[:6].tolist(),
            load_source='static-cad-gravity-and-scheduled-support')
        return self.load_correction.copy()

    def set_angles(self,angles):
        d=self.data;d.qpos[:]=self.model.qpos0;d.qpos[:3]=0;d.qpos[3:7]=[1,0,0,0]
        d.qpos[self.q]=np.radians(angles)
        mujoco.mj_kinematics(self.model,d);mujoco.mj_comPos(self.model,d)

    def foot(self,i,jacobian=False):
        m,d=self.model,self.data;f=self.feet[i]
        world=self.vertices[i]@d.geom_xmat[f].reshape(3,3).T+d.geom_xpos[f]
        bottom=world[np.argmin(world[:,2])]
        point=d.geom_xpos[f].copy();point[2]=bottom[2]
        if not jacobian:return point
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
        mujoco.mj_jacGeom(m,d,jp,jr,f)
        bp=np.zeros_like(jp);br=np.zeros_like(jp)
        mujoco.mj_jac(m,d,bp,br,bottom,int(m.geom_bodyid[f]));jp[2]=bp[2]
        return point,jp[:,self.v[i*3:i*3+3]]

    def solve(self,targets,seed,iterations=8,stance=None,locked_j1=None):
        # Once the bounded body pose has been selected, leg Jacobian blocks are
        # disjoint. Solve stance equalities first; swing tasks cannot spend their
        # degrees of freedom or relax a stance constraint. This is kinematic IK,
        # not a floating-base inverse-dynamics or contact-force optimizer.
        stance=np.ones(4,dtype=bool) if stance is None else np.asarray(stance,dtype=bool)
        order=[*np.flatnonzero(stance),*np.flatnonzero(~stance)]
        angles=np.asarray(seed).copy();max_error=0.
        if locked_j1 is not None:
            locked_j1=np.asarray(locked_j1,dtype=float)
            if locked_j1.shape!=(4,) or not np.isfinite(locked_j1).all() or np.max(abs(locked_j1))>30:
                raise ValueError('Expected four finite J1 angles within joint limits')
            angles[::3]=locked_j1
        for _ in range(iterations):
            self.set_angles(angles);errors=[];increments=np.zeros(12)
            for i in order:
                point,jac=self.foot(i,True);error=targets[i]-point;errors.append(np.linalg.norm(error))
                active=jac if locked_j1 is None else jac[:,1:]
                dq=np.linalg.solve(active.T@active+np.eye(active.shape[1])*1e-7,active.T@error)
                start=i*3+(0 if locked_j1 is None else 1)
                increments[start:i*3+3]=np.clip(np.degrees(dq),-4,4)
            max_error=max(errors)
            if max_error<.0001:break
            angles+=increments;angles=np.clip(angles,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
        self.set_angles(angles)
        max_error=max(np.linalg.norm(targets[i]-self.foot(i)) for i in range(4))
        return angles,float(max_error)

    def plan(self,params,phase,amplitude,linear,yaw,config):
        sensor_phase=phase
        lead=float(config.get('kinematic_lead_s',0.))
        if not math.isfinite(lead) or not 0<=lead<=.08:raise ValueError('Invalid trajectory lead')
        phase=(phase+lead/params[0])%1
        key=tuple(params)
        if self.key!=key:
            neutral=foot_targets(params,0,0).reshape(-1)
            self.set_angles(neutral);self.reference=np.array([self.foot(i) for i in range(4)])
            self.reference[:,2]=np.mean(self.reference[:,2])
            self.shoulders=np.array([self.data.xanchor[self.model.joint(l+'_j1').id].copy() for l in LEGS])
            self.center=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
            self.neutral_j1=neutral[::3].copy()
            self.body_height_target=float(self.data.xipos[self.model.body('cad_base').id,2]-np.mean(self.reference[:,2]))
            self.key=key
        points=self.reference.copy();duty=params[1];activity=min(1,abs(linear)+abs(yaw))
        for i,q in enumerate((phase+OFFSETS)%1):
            x,z=swing_path(q,duty,float(config.get('push_bias',0.)));command=np.clip(linear+(yaw if i%2==0 else -yaw),-1,1)
            points[i,0]+=amplitude*command*params[2]*x
            points[i,2]+=amplitude*activity*params[3]*z*float(config.get('lift_scale',[1.,1.,1.,1.])[i])
        window=.2/params[0];shift=np.zeros(3)
        # Prepare before liftoff, retain through swing, release after touchdown.
        for pair,offset in [((0,3),0.),((1,2),.5)]:
            gate=transfer_gate((phase+offset)%1,duty,window)
            supports=[i for i in range(4) if i not in pair]
            a,b=points[supports,:2];fraction=(self.center[0]-a[0])/(b[0]-a[0]) if abs(b[0]-a[0])>1e-6 else .5
            y=(a+np.clip(fraction,0,1)*(b-a))[1]-self.center[1]
            shift[1]+=gate*np.clip(y,-config.get('lateral_m',.01),config.get('lateral_m',.01))
            if config.get('support_foreaft_shift',False):
                # With J1 held, use bounded sagittal body transfer toward the
                # next support line. This is a reference shift, not a torso pin.
                fraction_x=(self.center[1]-a[1])/(b[1]-a[1]) if abs(b[1]-a[1])>1e-6 else .5
                x=(a+np.clip(fraction_x,0,1)*(b-a))[0]-self.center[0]
                shift[0]+=gate*np.clip(x,-.01,.01)
            if not config.get('constant_body_height',False):
                shift[2]-=gate*config.get('lower_m',.006)
        pulse=float(config.get('forward_pulse_m',0.))
        if not math.isfinite(pulse) or not 0<=pulse<=.01:
            raise ValueError('forward_pulse_m must be finite and at most 10mm')
        # Common body-progress modulation: every stance foot gets the same
        # fore/aft displacement, avoiding incompatible velocities in overlap.
        peak_phase=float(config.get('forward_pulse_peak_phase',.7))
        if not math.isfinite(peak_phase):raise ValueError('Pulse phase must be finite')
        if config.get('forward_pulse_shape')=='overlap':
            # Zero displacement and derivatives at touchdown/liftoff keeps
            # the original stance endpoints, while all supporting feet agree.
            overlap=duty-.5
            u=(phase%.5)/overlap if overlap>0 else 2.
            if 0<=u<=1:shift[0]-=pulse*64*u**3*(1-u)**3
        else:
            shift[0]+=pulse*np.sin(4*np.pi*(phase-peak_phase))
        shift*=amplitude*activity
        shift[:]=np.clip(shift,-.01,.01)
        points-=shift
        coefficients=np.asarray(config.get('feedforward_coefficients',np.zeros((2,7))),dtype=float)
        angle=2*np.pi*((phase+config.get('lead_s',.1)/params[0])%1)
        basis=np.array([1.,np.sin(angle),np.cos(angle),np.sin(2*angle),np.cos(2*angle),np.sin(3*angle),np.cos(3*angle)])
        ff=np.radians(np.clip(coefficients@basis,-8,8))*amplitude*activity
        self.feedforward_rotation=np.r_[ff,0.]
        self.swing_weights=np.zeros(4)
        for i,q in enumerate((phase+OFFSETS)%1):
            if q>=duty:
                u=(q-duty)/(1-duty);self.swing_weights[i]=smooth(u/.15)*smooth((1-u)/.15)
        sign=(1-2*self.swing_weights) if config.get('world_swing',False) else np.ones(4)
        points+=sign[:,None]*np.cross(self.feedforward_rotation,points-self.center)
        points=self.copy_front_swing(points,config)
        seed=foot_targets(params,phase*params[0],amplitude,linear=linear,yaw=yaw)
        stance=((phase+OFFSETS)%1)<duty
        locked=self.neutral_j1 if config.get("lock_j1",False) else None
        angles,residual=self.solve(points,seed,stance=stance,locked_j1=locked)
        stance_error=max((np.linalg.norm(points[i]-self.foot(i)) for i in np.flatnonzero(stance)),default=0.)
        self.last_phase=sensor_phase
        self.diagnostic=dict(policy='cad-support-shift',j1_locked=locked is not None,body_height_target_m=self.body_height_target,constant_body_height=bool(config.get('constant_body_height',False)),body_shift_m=shift.tolist(),planned_residual_m=residual,
                             commanded_stride_m=params[2],period_s=params[0],phase=phase,unreachable=residual>.001,
                             stance_residual_m=float(stance_error),scheduled_stance=stance.tolist())
        return angles

    def height_feedback(self,encoders,angles,params,config):
        gain=float(config.get('height_feedback_gain',0.))
        if gain<=0 and not config.get('separate_support_swing',False):return 0.
        self.set_angles(encoders)
        cr,sr=np.cos(angles[0]/2),np.sin(angles[0]/2)
        cp,sp=np.cos(angles[1]/2),np.sin(angles[1]/2)
        self.data.qpos[3:7]=[cp*cr,cp*sr,sp*cr,-sp*sr]
        mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
        stance=((self.last_phase+OFFSETS)%1)<params[1]
        bottoms=np.array([self.foot(i)[2] for i in np.flatnonzero(stance)])
        # Flat-ground support hypothesis based on gait phase, delayed encoders
        # and IMU only. Reject inconsistent support heights; no contact oracle.
        valid=len(bottoms)>=2 and np.ptp(bottoms)<.02
        if valid:
            estimate=float(self.data.xipos[self.model.body('cad_base').id,2]-np.median(bottoms))
            error=float(np.clip(estimate-self.body_height_target,-.025,.025))
            old=error if self.height_error_filtered is None else self.height_error_filtered
            self.height_error_filtered=old+.25*(error-old)
            velocity=float(np.clip((self.height_error_filtered-old)/.02,-.2,.2))
            desired=float(np.clip(gain*self.height_error_filtered+config.get('height_feedback_damping_s',.04)*velocity,-.01,.01))
            self.height_correction+=float(np.clip(desired-self.height_correction,-.001,.001))
            self.diagnostic.update(estimated_body_height_m=estimate,height_error_m=error)
        else:
            self.height_correction*=.9
        self.diagnostic.update(height_estimate_valid=bool(valid),height_correction_m=self.height_correction,
            height_source='delayed-imu-encoders-scheduled-support')
        if gain<=0:
            self.height_correction=0.
            self.diagnostic['height_correction_m']=0.
        return self.height_correction

    def feedback(self,nominal,encoders,attitude,params,config,enabled):
        if not enabled or encoders is None or attitude is None or getattr(attitude,'failures',0):
            self.reset();return np.asarray(nominal)
        if config.get("dynamics_wbc"):
            if self.dynamics_controller is None:
                from dynamics_wbc import DynamicsWBC
                self.dynamics_controller=DynamicsWBC(self)
            return self.dynamics_controller.apply(nominal,encoders,attitude,params,config)
        # Residual orientation correction in Cartesian foot space, not angle offsets.
        angles=np.radians(np.asarray(attitude.filtered)/10.)
        if config.get('separate_support_swing',False) and config.get('predict_swing_attitude',False):
            # The additional firmware low-pass filter lags by roughly three
            # control frames. Use the latest delivered (still delayed) packet
            # and bounded rate prediction, never the simulator orientation.
            latest=np.radians(np.asarray(getattr(attitude,'previous',attitude.filtered))/10.)
            velocity=np.radians(np.asarray(getattr(attitude,'rate',[0.,0.]))/10.)
            angles=np.clip(latest+.04*velocity,-.15,.15)
        self.integral=np.clip(self.integral+.02*angles*config.get('ki',.02),-.015,.015)
        rate=np.radians(np.asarray(getattr(attitude,'rate',[0.,0.]))/10.)
        rotation=np.r_[config.get('feedback_gain',.15)*angles+
                       config.get('feedback_damping_s',0.)*rate+self.integral,0.]
        rotation=np.clip(rotation,-.04,.04)
        height_delta=self.height_feedback(encoders,angles,params,config)
        if config.get('split_residual_tasks',False) and config.get('support_line_feedback',False):
            self.set_angles(encoders)
            cr,sr=np.cos(angles[0]/2),np.sin(angles[0]/2)
            cp,sp=np.cos(angles[1]/2),np.sin(angles[1]/2)
            self.data.qpos[3:7]=[cp*cr,cp*sr,sp*cr,-sp*sr]
            mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
            ids=np.flatnonzero(((self.last_phase+OFFSETS)%1)<params[1])
            if len(ids)==2:
                a,b=np.array([self.foot(i)[:2] for i in ids])
                com=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
                fraction=(com[0]-a[0])/(b[0]-a[0]) if abs(b[0]-a[0])>1e-6 else .5
                offset=float(np.clip((a+np.clip(fraction,0,1)*(b-a))[1]-com[1],-.01,.01))
                transfer_gain=float(config.get('support_line_gain',.2))
                transfer_step=float(config.get('support_line_step_m',.0005))
                if not np.isfinite([transfer_gain,transfer_step]).all() or not 0<transfer_gain<=1 or not 0<transfer_step<=.002:
                    raise ValueError('Invalid support transfer filter')
                self.support_offset_y+=float(np.clip(transfer_gain*(offset-self.support_offset_y),-transfer_step,transfer_step))
        if config.get('separate_support_swing',False):
            return self.separate_feedback(nominal,encoders,angles,rotation,height_delta,params,config)
        self.set_angles(nominal);points=np.array([self.foot(i) for i in range(4)])
        center=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
        rotations=np.tile(rotation,(4,1))
        if config.get('world_swing',False):
            # Swing feet track the ground frame under ACTUAL observed body tilt.
            # Subtract the pre-applied prediction to avoid counting it twice.
            swing=-np.r_[angles,0.]+self.feedforward_rotation
            rotations=(1-self.swing_weights[:,None])*rotations+self.swing_weights[:,None]*swing
        desired=points+np.cross(rotations,points-center)
        desired[:,2]+=height_delta
        if config.get('split_residual_tasks',False):
            fraction=float(config.get('swing_residual_gain',.15))
            if not np.isfinite(fraction) or not 0<=fraction<=1:raise ValueError('Invalid swing residual gain')
            swing=-fraction*np.r_[angles,0.]
            rotations=(1-self.swing_weights[:,None])*rotation+self.swing_weights[:,None]*swing
            desired=points+np.cross(rotations,points-center)
            desired[:,2]+=height_delta*(1-self.swing_weights)
            if config.get('support_line_feedback',False):desired[:,1]-=self.support_offset_y*(1-self.swing_weights)
        # Encoder packets detect excessive lag; do not amplify an unreachable task.
        lag=float(max(abs(np.asarray(encoders)-nominal)))
        if lag>15:self.diagnostic['feedback_limited']=True;self.integral*=.98;desired=points+.25*(desired-points)
        # Nominal targets already contain the blended copy. Copy only the new
        # sensor correction, otherwise the transition blend is applied twice.
        desired=points+self.copy_front_swing(desired-points,config,displacements=True)
        locked=self.neutral_j1 if config.get("lock_j1",False) else None
        corrected,residual=self.solve(desired,nominal,iterations=4,locked_j1=locked)
        limit=min(10.,float(config.get('feedback_limit_deg',4.)))
        desired_delta=np.clip(corrected-nominal,-limit,limit)
        slew=min(.5,float(config.get('feedback_step_deg',.25)))
        self.correction+=np.clip(desired_delta-self.correction,-slew,slew)
        if locked is not None:self.correction[::3]=0.
        self.diagnostic.update(feedback_residual_m=residual,encoder_lag_deg=lag,
            j1_correction_deg=self.correction[::3].tolist(),sensor_source='delayed-bno055-and-quantized-encoders')
        preload=self.load_preload(encoders,params,config)
        if config.get('split_residual_tasks',False):preload*=np.repeat(1-self.swing_weights,3)
        return np.clip(nominal+self.correction+preload,np.tile([-30,-45,0],4),np.tile([30,100,150],4))

    def separate_feedback(self,nominal,encoders,angles,rotation,height_delta,params,config):
        from support_task_experiments import separate_feedback
        return separate_feedback(self,nominal,encoders,angles,rotation,height_delta,params,config)
