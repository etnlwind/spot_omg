"""Experimental full task solver; current dynamic trials did not pass quality gates.

Kept separate from the conservative residual controller for reproducible work.
"""
import numpy as np
import mujoco
from support_shift import OFFSETS

def separate_feedback(self,nominal,encoders,angles,rotation,height_delta,params,config):
    """Fixed-J1 sagittal tasks: body support vs ground-frame swing height.

    Lateral foot position is not an independently attainable task with J1
    held. Solve body-X and world-Z directly, rather than an inconsistent
    three-dimensional rotation of a two-DOF leg. No plant truth is used.
    """
    if config.get('support_line_feedback',False):
        self.set_angles(encoders)
        sensed=np.array([self.foot(i) for i in range(4)])
        com=np.average(self.data.xipos,axis=0,weights=self.model.body_mass)
        stance=((self.last_phase+OFFSETS)%1)<params[1]
        ids=np.flatnonzero(stance)
        desired_y=0.
        if len(ids)==2:
            a,b=sensed[ids,:2]
            fraction=(com[0]-a[0])/(b[0]-a[0]) if abs(b[0]-a[0])>1e-6 else .5
            desired_y=float(np.clip((a+np.clip(fraction,0,1)*(b-a))[1]-com[1],-.01,.01))
        self.support_offset_y+=float(np.clip(.2*(desired_y-self.support_offset_y),-.0005,.0005))
    else:self.support_offset_y=0.
    self.set_angles(nominal)
    points=np.array([self.foot(i) for i in range(4)])
    torso=self.data.xipos[self.model.body('cad_base').id].copy()
    cr,sr=np.cos(angles[0]/2),np.sin(angles[0]/2)
    cp,sp=np.cos(angles[1]/2),np.sin(angles[1]/2)
    quat=np.array([cp*cr,cp*sr,sp*cr,-sp*sr])
    matrix=np.empty(9);mujoco.mju_quat2Mat(matrix,quat);matrix=matrix.reshape(3,3)
    support_points=points+np.cross(np.tile(rotation,(4,1)),points-torso)
    support_points[:,2]+=height_delta
    support_points[:,1]-=self.support_offset_y
    support_z=support_points@matrix[2]
    estimate=self.diagnostic.get('estimated_body_height_m',self.body_height_target)
    height_error=float(np.clip(self.body_height_target-estimate,-.01,.01))
    swing_z=points[:,2]-torso[2]+matrix[2]@torso+height_error
    weights=self.swing_weights.copy()
    swing_gain=float(config.get('swing_world_gain',1.))
    if not np.isfinite(swing_gain) or not 0<=swing_gain<=1:
        raise ValueError('swing_world_gain must be between zero and one')
    target_z=(1-weights)*support_z+weights*((1-swing_gain)*(points@matrix[2])+swing_gain*swing_z)
    # Preserve the gait's body-frame horizontal travel. Only swing height
    # is a ground-frame task; rotating XY as well changes stride and lateral
    # placement while the torso tilts.
    target_xy=(1-weights[:,None])*support_points[:,:2]+weights[:,None]*points[:,:2]
    q=np.asarray(nominal).copy();locked=self.neutral_j1.copy() if config.get('lock_j1',False) else None
    for _ in range(8):
        self.set_angles(q);self.data.qpos[3:7]=quat
        mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
        delta=np.zeros(12);errors=[]
        for i in range(4):
            f=self.feet[i];point,jac=self.foot(i,True)
            jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacGeom(self.model,self.data,jp,jr,f)
            body_x=matrix[:,0]@self.data.geom_xpos[f]
            jac_x=matrix[:,0]@jp[:,self.v[i*3:i*3+3]]
            if locked is not None:
                task_jac=np.array([jac_x[1:],jac[2,1:]])
                error=np.array([points[i,0]-body_x,target_z[i]-point[2]])
            else:
                body_y=matrix[:,1]@self.data.geom_xpos[f]
                jac_y=matrix[:,1]@jp[:,self.v[i*3:i*3+3]]
                task_jac=np.array([jac_x,jac_y,jac[2]])
                error=np.array([target_xy[i,0]-body_x,target_xy[i,1]-body_y,target_z[i]-point[2]])
            errors.append(np.linalg.norm(error))
            dq=np.linalg.solve(task_jac.T@task_jac+np.eye(task_jac.shape[1])*1e-7,task_jac.T@error)
            delta[i*3+(1 if locked is not None else 0):i*3+3]=np.clip(np.degrees(dq),-4,4)
        if max(errors)<.0001:break
        q=np.clip(q+delta,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
        if locked is not None:q[::3]=locked
    limit=float(config.get('separated_limit_deg',6.))
    slew=float(config.get('separated_step_deg',.5))
    if not np.isfinite([limit,slew]).all() or not 0<limit<=10 or not 0<slew<=1.5:
        raise ValueError('Invalid separated correction limits')
    desired_delta=np.clip(q-nominal,-limit,limit)
    self.correction+=np.clip(desired_delta-self.correction,-slew,slew)
    if locked is not None:self.correction[::3]=0
    # Static support preload fades out on swing legs; it is not an
    # additional swing-position task.
    preload=self.load_preload(encoders,params,config)
    preload*=np.repeat(1-weights,3)
    output=np.clip(nominal+self.correction+preload,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
    self.set_angles(output);self.data.qpos[3:7]=quat
    mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
    applied_errors=[]
    for i,f in enumerate(self.feet):
        body_x=matrix[:,0]@self.data.geom_xpos[f]
        if locked is not None:
            error=[points[i,0]-body_x,target_z[i]-self.foot(i)[2]]
        else:
            body_y=matrix[:,1]@self.data.geom_xpos[f]
            error=[target_xy[i,0]-body_x,target_xy[i,1]-body_y,target_z[i]-self.foot(i)[2]]
        applied_errors.append(float(np.linalg.norm(error)))
    self.diagnostic.update(separate_support_swing=True,feedback_residual_m=float(max(errors)),
        support_offset_y_m=self.support_offset_y,
        applied_task_residual_m=max(applied_errors),applied_leg_task_residual_m=applied_errors,
        swing_task_weight=weights.tolist(),swing_height_error_m=height_error,
        j1_correction_deg=self.correction[::3].tolist(),sensor_source='delayed-bno055-and-quantized-encoders')
    return output
