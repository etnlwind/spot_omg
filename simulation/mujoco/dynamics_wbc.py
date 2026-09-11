"""Constrained floating-base inverse dynamics for the position-servo simulator.

M qdd + h = S.T tau + J.T f. Dynamics, unilateral/friction forces,
motor torque-speed bounds and joint limits are HARD constraints. Contact,
body and swing tracking are weighted soft tasks; residuals are reported.
Only a CAD model and delivered IMU/encoder packets enter this controller.
This is instantaneous inverse dynamics, not horizon MPC or torque WBIC.
"""
import time
import mujoco
import numpy as np
import osqp
from scipy import sparse

DT = .02
OFFSETS = np.array([0., .5, .5, 0.])
LOW = np.radians(np.tile([-30., -45., 0.], 4))
HIGH = np.radians(np.tile([30., 100., 150.], 4))


def constrained_dynamics(M, bias, J, tasks, joint_dofs, torque_limit,
                         acceleration_low, acceleration_high, friction=.5, force_reference=None):
    """Solve acceleration + point-force QP; never silently relax physics.

    J has three world Cartesian rows per point. Inscribed friction pyramid
    |fx|+|fy| <= mu*fz lies inside the circular Coulomb cone.
    """
    nv = len(bias); nf = J.shape[0]; n = nv + nf
    H = np.eye(n)*1e-5; g = np.zeros(n)
    for A, target, weight in tasks:
        w = np.broadcast_to(np.asarray(weight), len(target))
        H[:nv, :nv] += A.T @ (w[:, None]*A)
        g[:nv] -= A.T @ (w*target)
    H[nv:, nv:] += np.eye(nf)*1e-4
    if force_reference is not None:
        H[nv:,nv:]+=np.eye(nf)*.01
        g[nv:]-=.01*np.asarray(force_reference).reshape(-1)
    dynamics = np.c_[M, -J.T]
    base = np.array([i for i in range(nv) if i not in joint_dofs])
    rows = [dynamics[base], dynamics[joint_dofs], np.c_[np.eye(nv), np.zeros((nv,nf))]]
    lower = [-bias[base], -torque_limit-bias[joint_dofs], acceleration_low]
    upper = [-bias[base], torque_limit-bias[joint_dofs], acceleration_high]
    for i in range(nf//3):
        cone = np.zeros((5,n)); k = nv+3*i
        cone[0,k+2] = 1
        for r,(sx,sy) in enumerate(((1,1),(1,-1),(-1,1),(-1,-1)),1):
            cone[r,k:k+3] = [sx,sy,-friction]
        rows.append(cone); lower.append(np.array([0,-np.inf,-np.inf,-np.inf,-np.inf]))
        upper.append(np.array([150,0,0,0,0]))
    A = np.vstack(rows); lo=np.concatenate(lower); hi=np.concatenate(upper)
    if np.any(lo>hi):
        return None, dict(status='infeasible-joint-envelope')
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(np.triu(H)), q=g, A=sparse.csc_matrix(A),
                 l=lo, u=hi, verbose=False, eps_abs=2e-5, eps_rel=2e-5,
                 max_iter=2000, polishing=True)
    result=solver.solve(raise_error=False)
    if result.x is None or result.info.status_val not in (1,2):
        return None,dict(status=result.info.status)
    x=result.x; violation=float(max(np.max(lo-A@x),np.max(A@x-hi),0))
    if not np.isfinite(x).all() or violation>2e-3:
        return None,dict(status='constraint-residual-rejected',violation=violation)
    qdd=x[:nv]; force=x[nv:]; tau=(dynamics@x+bias)[joint_dofs]
    return (qdd,force,tau),dict(status=result.info.status,
        constraint_violation=violation,iterations=int(result.info.iter),
        dynamics_residual=float(np.max(abs((dynamics@x+bias)[base]))))


class DynamicsWBC:
    def __init__(self, geometry):
        self.geometry=geometry;self.model=geometry.model
        self.data=mujoco.MjData(self.model);self.future=mujoco.MjData(self.model)
        self.q=geometry.q;self.v=geometry.v;self.feet=geometry.feet
        self.base=self.model.body('cad_base').id
        self.reset()

    def reset(self):
        self.previous_q=None;self.qvelocity=np.zeros(12)
        self.previous_nominal=None;self.nominal_velocity=np.zeros(12)
        self.base_velocity=np.zeros(3);self.previous_command=None
        self.failed_frames=0;self.diagnostic={};self.adapter_delta=np.zeros(12);self.load_delta=np.zeros(12)
        self.preview=None;self.preview_report={};self.preview_tick=0;self.lateral_position=0.

    def kinematics(self,data,local_points):
        m=self.model;points=[];jac=[]
        for i,f in enumerate(self.feet):
            point=data.geom_xpos[f]+data.geom_xmat[f].reshape(3,3)@local_points[i]
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jac(m,data,jp,jr,point,int(m.geom_bodyid[f]))
            points.append(point);jac.append(jp)
        return np.array(points),np.array(jac)

    def apply(self,nominal,encoders,attitude,params,config):
        start=time.perf_counter();g=self.geometry;m=self.model;d=self.data;cfg=config['dynamics_wbc']
        observed=np.radians(encoders);nominal_rad=np.radians(nominal)
        if observed.shape!=(12,) or not np.isfinite(observed).all():
            self.reset();self.diagnostic={'feasible':False,'reason':'invalid-encoder'}
            g.diagnostic['dynamics_wbc']=self.diagnostic
            return np.asarray(nominal)
        settings={
            'estimated_voltage_v':(10.5,6.,12.6),'rated_factor':(1.,.1,3.),
            'friction':(.5,.05,2.),'contact_patch_depth_m':(0.,0.,.003),
            'pose_preview_s':(.1,.02,.2),'target_step_deg':(1.,.1,9.),
            'encoder_prediction_s':(.04,0.,.06),'imu_prediction_s':(.02,0.,.06),
            'servo_kp':(35.,1.,200.),'servo_kd':(.8,0.,10.),
            'attitude_kp':(80.,0.,200.),'attitude_kd':(12.,0.,40.)}
        for key,(default,low,high) in settings.items():
            value=float(cfg.get(key,default))
            if not np.isfinite(value) or not low<=value<=high:
                raise ValueError('Invalid dynamics WBC setting: '+key)
        if cfg.get('position_adapter','torque-inversion') not in ('torque-inversion','trajectory','bounded_trajectory'):
            raise ValueError('Unknown dynamics WBC position adapter')
        if self.previous_q is not None:
            raw=np.clip((observed-self.previous_q)/DT,-8,8)
            self.qvelocity += .35*(raw-self.qvelocity)
        self.previous_q=observed.copy()
        old_nomvel=self.nominal_velocity.copy()
        if self.previous_nominal is not None:
            raw=np.clip((nominal_rad-self.previous_nominal)/DT,-8,8)
            self.nominal_velocity += .5*(raw-self.nominal_velocity)
        self.previous_nominal=nominal_rad.copy()
        nominal_acc=np.clip((self.nominal_velocity-old_nomvel)/DT,-100,100)
        # Latest delivered packet avoids the extra firmware smoothing delay.
        angle=np.radians(np.asarray(getattr(attitude,'previous',attitude.filtered))/10.)
        omega=np.r_[np.radians(np.asarray(getattr(attitude,'rate',[0.,0.]))/10.),0.]
        if not np.isfinite(angle).all() or not np.isfinite(omega).all():
            self.reset();self.diagnostic={'feasible':False,'reason':'invalid-imu'}
            g.diagnostic['dynamics_wbc']=self.diagnostic
            return np.asarray(nominal)
        angle=np.clip(angle+float(cfg.get('imu_prediction_s',.02))*omega[:2],-.3,.3)
        cr,sr=np.cos(angle[0]/2),np.sin(angle[0]/2);cp,sp=np.cos(angle[1]/2),np.sin(angle[1]/2)
        d.qpos[:]=m.qpos0;d.qpos[:3]=0;d.qpos[3:7]=[cp*cr,cp*sr,sp*cr,-sp*sr]
        d.qpos[self.q]=np.clip(observed+float(cfg.get('encoder_prediction_s',.04))*self.qvelocity,LOW,HIGH)
        d.qvel[:]=0;d.qvel[self.v]=self.qvelocity
        mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
        R=d.xmat[self.base].reshape(3,3)
        # MuJoCo free-joint angular velocity is in the body frame.
        d.qvel[3:6]=R.T@omega
        scheduled=((g.last_phase+OFFSETS)%1)<params[1]
        support=np.flatnonzero(scheduled)
        local=[]
        for i,f in enumerate(self.feet):
            rotated=g.vertices[i]@d.geom_xmat[f].reshape(3,3).T
            local.append(g.vertices[i][np.argmin(rotated[:,2])])
        points,jac=self.kinematics(d,local)
        # Flat terrain, scheduled support, delayed encoders: no truth contacts.
        ground=float(np.median(points[support,2]))
        height=float(d.xipos[self.base,2]-ground)
        velocities=np.array([jac[i]@d.qvel for i in support])
        self.base_velocity += .25*(-np.mean(velocities,axis=0)-self.base_velocity)
        d.qvel[:3]=self.base_velocity
        mujoco.mj_forward(m,d)
        M=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,d,M)
        bias=d.qfrc_bias.copy()-d.qfrc_passive.copy()
        # Jdot*qdot at fixed material points, not differences of contact vertices.
        future=self.future;future.qpos[:]=d.qpos;future.qvel[:]=d.qvel
        eps=1e-4;mujoco.mj_integratePos(m,future.qpos,d.qvel,eps)
        mujoco.mj_kinematics(m,future);mujoco.mj_comPos(m,future)
        _,nextjac=self.kinematics(future,local)
        jdot=np.array([(nextjac[i]-jac[i])@d.qvel/eps for i in range(4)])
        # Nominal material-point world targets when the torso is level. The
        # cushion bottom has the same world clearance for each diagonal foot.
        g.set_angles(nominal)
        ref=[];refjac=[]
        for i,f in enumerate(self.feet):
            p=g.data.geom_xpos[f]+g.data.geom_xmat[f].reshape(3,3)@local[i]
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jac(m,g.data,jp,jr,p,int(m.geom_bodyid[f]))
            ref.append(p);refjac.append(jp[:,self.v])
        ref=np.array(ref);refjac=np.array(refjac)
        # Near touchdown/liftoff the nominal path and delivered kinematics can
        # support a short load transfer, without pausing the diagonal clock.
        # In particular amplitude=0 startup has FOUR feet, not two contacts.
        nominal_bottom=np.array([g.foot(i)[2] for i in range(4)])
        eligible=(nominal_bottom-g.reference[:,2]<.0015)&(points[:,2]-ground<.005)
        scheduled=scheduled|eligible
        support=np.flatnonzero(scheduled)
        desired_velocity=np.array([j@self.nominal_velocity for j in refjac])
        forward_velocity=-float(np.mean(desired_velocity[support,0]))
        root_z=g.body_height_target-(g.data.xipos[self.base,2])
        desired=ref.copy();desired[:,2]+=ground+root_z
        preview_acc=None;preview_forces=None
        self.lateral_position+=self.base_velocity[1]*DT
        if cfg.get('centroidal_preview',False):
            from centroidal_preview import solve_preview
            if self.preview_tick%4==0:
                com=np.average(d.xipos,axis=0,weights=m.body_mass)
                inertia=np.zeros((3,3))
                for b in range(1,m.nbody):
                    rot=d.ximat[b].reshape(3,3);r=d.xipos[b]-com
                    inertia+=rot@np.diag(m.body_inertia[b])@rot.T+m.body_mass[b]*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
                state=np.r_[0.,self.lateral_position,height-g.body_height_target,angle,0.,self.base_velocity,omega]
                self.preview,self.preview_report=solve_preview(state,float(m.body_mass.sum()),inertia,points,com,
                    g.last_phase,params,forward_velocity,float(cfg.get('friction',.5)))
            self.preview_tick+=1
            if self.preview is not None:
                states,forces=self.preview
                preview_forces=forces[0]
                preview_acc=(states[1,6:]-states[0,6:])/.08
        tasks=[];nv=m.nv
        body_jp=np.zeros((3,nv));body_jr=np.zeros_like(body_jp)
        mujoco.mj_jacBodyCom(m,d,body_jp,body_jr,self.base)
        body_acc=np.array([8*(forward_velocity-self.base_velocity[0]),
                           -8*self.base_velocity[1],
                           100*(g.body_height_target-height)-18*self.base_velocity[2]])
        angular_acc=-float(cfg.get('attitude_kp',80))*np.r_[angle,0.]-float(cfg.get('attitude_kd',12))*omega
        if preview_acc is not None:
            body_acc=preview_acc[:3];angular_acc=preview_acc[3:]
        tasks.append((body_jp,np.clip(body_acc,-3,3),30.))
        tasks.append((body_jr,np.clip(angular_acc,-25,25),3.))
        for i in range(4):
            velocity=jac[i]@d.qvel
            if scheduled[i]:
                accel=-20*velocity-jdot[i]
                tasks.append((jac[i],np.clip(accel,-10,10),300.))
            else:
                velref=desired_velocity[i]+np.array([forward_velocity,0,0])
                accel=refjac[i]@nominal_acc+160*(desired[i]-points[i])+22*(velref-velocity)-jdot[i]
                tasks.append((jac[i],np.clip(accel,-20,20),60.))
        # Mild posture task resolves unused DOFs without fixing J1.
        joint_task=np.zeros((12,nv));joint_task[:,self.v]=np.eye(12)
        joint_acc=nominal_acc+30*(nominal_rad-d.qpos[self.q])+6*(self.nominal_velocity-self.qvelocity)
        tasks.append((joint_task,np.clip(joint_acc,-100,100),.005))
        # Conservative 3S operating voltage estimate and symmetric motoring
        # envelope; rated limits are used unless explicitly testing peak torque.
        voltage=float(cfg.get('estimated_voltage_v',10.5));scale=voltage/12
        stall=np.tile([2.941995,4.903325,2.941995],4)
        speed=np.tile([4.717106,7.873666,4.717106],4)
        rated=np.tile([.980665,1.569064,.980665],4)
        limits=np.minimum(rated*float(cfg.get('rated_factor',1.)),stall*scale*np.maximum(0,1-abs(self.qvelocity)/(speed*scale)))
        lo=np.full(nv,-100.);hi=np.full(nv,100.)
        horizon=.1
        lo[self.v]=np.maximum(lo[self.v],2*(LOW-d.qpos[self.q]-horizon*self.qvelocity)/horizon**2)
        hi[self.v]=np.minimum(hi[self.v],2*(HIGH-d.qpos[self.q]-horizon*self.qvelocity)/horizon**2)
        force_jac=[];force_points=[];force_legs=[]
        depth=float(cfg.get('contact_patch_depth_m',0.))
        for i in support:
            f=self.feet[i]
            world=g.vertices[i]@d.geom_xmat[f].reshape(3,3).T+d.geom_xpos[f]
            candidates=np.flatnonzero(world[:,2]<=world[:,2].min()+depth)
            chosen=np.unique([candidates[np.argmin(world[candidates,0])],candidates[np.argmax(world[candidates,0])],
                              candidates[np.argmin(world[candidates,1])],candidates[np.argmax(world[candidates,1])]])
            for k in chosen:
                jp=np.zeros((3,nv));jr=np.zeros_like(jp)
                mujoco.mj_jac(m,d,jp,jr,world[k],int(m.geom_bodyid[f]))
                force_jac.append(jp);force_points.append(world[k].copy());force_legs.append(int(i))
        J=np.vstack(force_jac)
        force_reference=None
        if preview_forces is not None:
            force_reference=np.array([preview_forces[i]/force_legs.count(i) for i in force_legs])
        solution,report=constrained_dynamics(M,bias,J,tasks,self.v,limits,lo,hi,float(cfg.get('friction',.5)),force_reference)
        if solution is None:
            self.failed_frames+=1
            # Explicit failure: hold last finite target, never reinterpret an
            # infeasible solve as a passed gait or reduce its stride.
            output=np.asarray(nominal) if self.previous_command is None else self.previous_command.copy()
            self.diagnostic=dict(qp=report,failed_frames=self.failed_frames,feasible=False)
        else:
            self.failed_frames=0;qdd,force,tau=solution
            # Invert the existing position-servo PD estimate. Real firmware has
            # no torque command, so this is an estimated torque-to-position map.
            kp=float(cfg.get('servo_kp',35));kd=float(cfg.get('servo_kd',.8))
            adapter_residual=0.
            target=d.qpos[self.q]+(tau+kd*self.qvelocity)/kp
            if cfg.get('position_adapter') in ('trajectory','bounded_trajectory'):
                # A position servo cannot reproduce a free torque loop at 50 Hz.
                # Keep its path reference, using the solved body acceleration
                # for a bounded pose preview and solved torque as feedforward.
                # Report the resulting torque discrepancy; this is NOT exact
                # inverse-dynamics torque execution.
                horizon=float(cfg.get('pose_preview_s',.1))
                root_delta=.5*horizon*horizon*(body_jp@qdd)
                root_delta[0]=0.
                root_delta=np.clip(root_delta,-.01,.01)
                turn_delta=.5*horizon*horizon*(body_jr@qdd)
                turn_delta=np.clip(turn_delta,-.08,.08)
                g.set_angles(nominal)
                nominal_points=np.array([g.foot(i) for i in range(4)])
                center=g.data.xipos[self.base].copy()
                desired_points=nominal_points.copy()
                for i in range(4):
                    if scheduled[i]:
                        desired_points[i]-=root_delta+np.cross(turn_delta,nominal_points[i]-center)
                    else:
                        desired_world=nominal_points[i].copy()
                        desired_world[2]+=ground+root_z
                        desired_points[i]=R.T@desired_world
                solved_angles,residual=g.solve(desired_points,nominal,iterations=8,stance=scheduled)
                feedforward_tau=tau
                if cfg.get('position_adapter')=='bounded_trajectory':
                    feedforward_tau=bias[self.v]-J[:,self.v].T@force
                    # Acceleration feedback belongs to the position loop. Do
                    # not add that same PD torque again as a feedforward load.
                    correction=np.clip(solved_angles-nominal,-4.,4.)
                    feedforward=np.clip(np.degrees(feedforward_tau/kp),-4.,4.)
                    # The load preload must not trail into the swing. Keeping
                    # it in the pose integrator delayed unloading by ~0.32 s.
                    self.adapter_delta+=np.clip(correction-self.adapter_delta,-.25,.25)
                    self.load_delta+=np.clip(feedforward-self.load_delta,-.25,.25)
                    load_window=np.repeat(1-g.swing_weights,3)
                    target=nominal_rad+np.radians(self.adapter_delta+self.load_delta*load_window)
                else:
                    target=np.radians(solved_angles)+(tau+kd*self.nominal_velocity)/kp
                adapter_residual=residual
            output=np.degrees(np.clip(target,LOW,HIGH))
            clipped=bool(np.any(abs(target-np.clip(target,LOW,HIGH))>1e-8))
            self.diagnostic=dict(qp=report,feasible=True,failed_frames=0,
                estimated_height_m=height,estimated_base_velocity=self.base_velocity.tolist(),
                estimated_rp_rad=angle.tolist(),support=support.tolist(),
                contact_force_n=force.reshape(-1,3).tolist(),torque_nm=tau.tolist(),
                torque_limits_nm=limits.tolist(),qdd=qdd.tolist(),servo_map_clipped=clipped,
                adapter=cfg.get('position_adapter','torque-inversion'),adapter_ik_residual_m=adapter_residual,
                adapter_torque_difference_nm=(kp*(target-d.qpos[self.q])-kd*self.qvelocity-tau).tolist(),
                body_linear_acc_residual=(body_jp@qdd-body_acc).tolist(),
                body_angular_acc_residual=(body_jr@qdd-angular_acc).tolist(),
                contact_acc_residual=(jac[support].reshape(-1,nv)@qdd+jdot[support].reshape(-1)).tolist(),
                force_point_legs=force_legs,force_points_m=np.asarray(force_points).tolist(),
                source='delayed-imu-encoders-private-cad-point-contact-QP')
        # Bumpless engagement and phase transfer, before the plant's own
        # velocity/acceleration limits; record when the position map is limited.
        previous=np.asarray(nominal) if self.previous_command is None else self.previous_command
        delta=np.clip(output-previous,-float(cfg.get('target_step_deg',1.)),float(cfg.get('target_step_deg',1.)))
        limited=previous+delta
        self.diagnostic['centroidal_preview']=self.preview_report
        self.diagnostic['command_slew_limited']=bool(np.any(abs(limited-output)>1e-6))
        output=limited
        self.previous_command=output.copy()
        self.diagnostic['elapsed_ms']=(time.perf_counter()-start)*1000
        g.diagnostic.update(dynamics_wbc=self.diagnostic)
        return output
