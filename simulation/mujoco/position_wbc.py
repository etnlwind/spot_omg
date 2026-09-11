"""Position-servo adaptation of contact-first kinematic whole-body tasks.

Reference: Kim et al. 2019, eqs.16-24. Not their torque WBIC/MPC.
The controller owns isolated kinematic data. No simulation contacts, root pose,
velocities or actuator forces are read here. IMU and delayed encoder packets only.
"""
from collections import deque
import numpy as np
import mujoco


def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])


def pinv(a):
    return np.linalg.pinv(a,rcond=1e-5)


def priority_solve(contact, orientation, desired):
    n=orientation.shape[1]
    null=np.eye(n)-pinv(contact)@contact if len(contact) else np.eye(n)
    task=orientation@null
    delta=null@pinv(task)@desired
    return delta,null


class EncoderChannel:
    """Explicit 40ms/4096-count feedback model; values are not exact state."""
    def __init__(self,delay_frames=2):self.queue=deque();self.delay=delay_frames
    def read(self,angles):
        self.queue.append(np.round(np.asarray(angles)*4096/360)*360/4096)
        if len(self.queue)<=self.delay:return None
        return self.queue.popleft()


class PositionWBC:
    def __init__(self,model):
        self.model=model;self.data=mujoco.MjData(model)
        self.feet=[model.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
        self.q=np.array([model.jnt_qposadr[model.joint(l+'_j'+str(j)).id] for l in ('fl','fr','rl','rr') for j in (1,2,3)])
        self.v=np.array([model.jnt_dofadr[model.joint(l+'_j'+str(j)).id] for l in ('fl','fr','rl','rr') for j in (1,2,3)])
        self.delta=np.zeros(12);self.confidence=np.zeros(4);self.scale=1.
        self.diagnostic={}

    def apply(self,nominal,encoders,attitude,phase,duty,config,enabled):
        if not enabled or encoders is None:
            self.delta[:]=0;self.confidence[:]=0;self.scale=1.;return np.asarray(nominal)
        m,d=self.model,self.data
        d.qpos[:]=m.qpos0;d.qpos[:3]=0;d.qpos[3:7]=[1,0,0,0]
        d.qpos[self.q]=np.radians(encoders);mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
        attitude_rad=np.radians(np.asarray(attitude.filtered)/10.)
        rotation=np.array([attitude_rad[0],attitude_rad[1],0.])
        feet=np.array([d.geom_xpos[f] for f in self.feet])
        # Relative plane-height evidence. This is a heuristic contact estimator,
        # not a force sensor or a full InEKF. Soft pad compression is uncertain.
        level=feet+np.cross(rotation,feet)
        heights=level[:,2]-level[:,2].min()
        scheduled=np.array([((phase+o)%1)<duty for o in (0,.5,.5,0)])
        evidence=scheduled*(.55+.45*np.exp(-np.maximum(0,heights-.008)/.012))
        self.confidence+=.25*(evidence-self.confidence)
        contacts=[];jacobians=[]
        for f in self.feet:
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacGeom(m,d,jp,jr,f);jacobians.append(jp)
        support=np.flatnonzero(self.confidence>.35)
        for i in support:contacts.append(jacobians[i])
        if len(support)<2:
            desired_delta=np.zeros(12);residual=0.
        else:
            jc=np.vstack(contacts)
            orient=np.zeros((6,m.nv));orient[:,:6]=np.eye(6)
            gain=float(config.get('gain',.5))
            center=np.average(d.xipos,axis=0,weights=m.body_mass)
            # Keep estimated COM fixed: the CAD free-joint origin is not COM.
            demand=np.r_[np.cross(gain*rotation,center),-gain*rotation]
            full,null=priority_solve(jc,orient,demand)
            # Lower-priority swing foot task in the null-space of contact+attitude.
            null2=null@(np.eye(m.nv)-pinv(orient@null)@(orient@null))
            swing=[jacobians[i] for i in range(4) if i not in support]
            if swing:
                js=np.vstack(swing);full+=null2@pinv(js@null2)@(-js@full)
            desired_delta=np.degrees(full[self.v])
            residual=float(np.linalg.norm(jc@full))
        limit=float(config.get('joint_limit_deg',4.))
        saturation=float(np.max(abs(desired_delta)))/limit
        if saturation>1:desired_delta/=saturation
        self.delta+=np.clip(desired_delta-self.delta,-.25,.25)
        desired_scale=float(np.clip(1-(max(abs(attitude_rad))*180/np.pi-5)/8,.55,1))
        if saturation>1:desired_scale=min(desired_scale,.8)
        self.scale+=float(np.clip(desired_scale-self.scale,-.015,.003))
        result=np.asarray(nominal)+self.delta
        result=np.clip(result,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
        self.diagnostic=dict(policy='contact-first-position-wbc',support_confidence=self.confidence.tolist(),
            constraint_residual_m=residual,max_joint_correction_deg=float(max(abs(self.delta))),
            max_j1_correction_deg=float(max(abs(self.delta[::3]))),stride_scale=self.scale,
            encoder_delay_ms=40,contact_estimator='scheduled-plus-relative-kinematic-height')
        return result
