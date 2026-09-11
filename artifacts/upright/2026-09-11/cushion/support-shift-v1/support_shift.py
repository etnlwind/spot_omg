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


def swing_path(q,duty):
    if q<duty:return .5-q/duty,0.
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
        self.diagnostic={};self.last_phase=0.

    def reset(self):
        self.correction[:]=0;self.integral[:]=0

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

    def solve(self,targets,seed,iterations=8,stance=None):
        # Once the bounded body pose has been selected, leg Jacobian blocks are
        # disjoint. Solve stance equalities first; swing tasks cannot spend their
        # degrees of freedom or relax a stance constraint. This is kinematic IK,
        # not a floating-base inverse-dynamics or contact-force optimizer.
        stance=np.ones(4,dtype=bool) if stance is None else np.asarray(stance,dtype=bool)
        order=[*np.flatnonzero(stance),*np.flatnonzero(~stance)]
        angles=np.asarray(seed).copy();max_error=0.
        for _ in range(iterations):
            self.set_angles(angles);errors=[];increments=np.zeros(12)
            for i in order:
                point,jac=self.foot(i,True);error=targets[i]-point;errors.append(np.linalg.norm(error))
                dq=np.linalg.solve(jac.T@jac+np.eye(3)*1e-7,jac.T@error)
                increments[i*3:i*3+3]=np.clip(np.degrees(dq),-4,4)
            max_error=max(errors)
            if max_error<.0001:break
            angles+=increments;angles=np.clip(angles,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
        self.set_angles(angles)
        max_error=max(np.linalg.norm(targets[i]-self.foot(i)) for i in range(4))
        return angles,float(max_error)

    def plan(self,params,phase,amplitude,linear,yaw,config):
        key=tuple(params)
        if self.key!=key:
            neutral=foot_targets(params,0,0).reshape(-1)
            self.set_angles(neutral);self.reference=np.array([self.foot(i) for i in range(4)])
            self.reference[:,2]=np.mean(self.reference[:,2])
            self.center=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
            self.key=key
        points=self.reference.copy();duty=params[1];activity=min(1,abs(linear)+abs(yaw))
        for i,q in enumerate((phase+OFFSETS)%1):
            x,z=swing_path(q,duty);command=np.clip(linear+(yaw if i%2==0 else -yaw),-1,1)
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
            shift[2]-=gate*config.get('lower_m',.006)
        shift*=amplitude*activity
        shift[1:]=np.clip(shift[1:],-.01,.01)
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
        seed=foot_targets(params,phase*params[0],amplitude,linear=linear,yaw=yaw)
        stance=((phase+OFFSETS)%1)<duty
        angles,residual=self.solve(points,seed,stance=stance)
        stance_error=max((np.linalg.norm(points[i]-self.foot(i)) for i in np.flatnonzero(stance)),default=0.)
        self.last_phase=phase
        self.diagnostic=dict(policy='cad-support-shift',body_shift_m=shift.tolist(),planned_residual_m=residual,
                             commanded_stride_m=params[2],period_s=params[0],phase=phase,unreachable=residual>.001,
                             stance_residual_m=float(stance_error),scheduled_stance=stance.tolist())
        return angles

    def feedback(self,nominal,encoders,attitude,params,config,enabled):
        if not enabled or encoders is None or attitude is None or getattr(attitude,'failures',0):
            self.reset();return np.asarray(nominal)
        # Residual orientation correction in Cartesian foot space, not angle offsets.
        angles=np.radians(np.asarray(attitude.filtered)/10.)
        self.integral=np.clip(self.integral+.02*angles*config.get('ki',.02),-.015,.015)
        rotation=np.r_[config.get('feedback_gain',.15)*angles+self.integral,0.]
        rotation=np.clip(rotation,-.04,.04)
        self.set_angles(nominal);points=np.array([self.foot(i) for i in range(4)])
        center=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
        rotations=np.tile(rotation,(4,1))
        if config.get('world_swing',False):
            # Swing feet track the ground frame under ACTUAL observed body tilt.
            # Subtract the pre-applied prediction to avoid counting it twice.
            swing=-np.r_[angles,0.]+self.feedforward_rotation
            rotations=(1-self.swing_weights[:,None])*rotations+self.swing_weights[:,None]*swing
        desired=points+np.cross(rotations,points-center)
        # Encoder packets detect excessive lag; do not amplify an unreachable task.
        lag=float(max(abs(np.asarray(encoders)-nominal)))
        if lag>15:self.diagnostic['feedback_limited']=True;self.integral*=.98;desired=points+.25*(desired-points)
        corrected,residual=self.solve(desired,nominal,iterations=4)
        limit=min(10.,float(config.get('feedback_limit_deg',4.)))
        desired_delta=np.clip(corrected-nominal,-limit,limit)
        slew=min(.5,float(config.get('feedback_step_deg',.25)))
        self.correction+=np.clip(desired_delta-self.correction,-slew,slew)
        self.diagnostic.update(feedback_residual_m=residual,encoder_lag_deg=lag,
            j1_correction_deg=self.correction[::3].tolist(),sensor_source='delayed-bno055-and-quantized-encoders')
        return np.clip(nominal+self.correction,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
