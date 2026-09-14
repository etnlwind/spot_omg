"""Isolated CAD stance experiment; no deployed firmware or profile mutation."""
import numpy as np
import mujoco
from support_shift import SupportShift, LEGS
from gait_profiles import foot_targets, smooth
import virtual_robot
from cad_physics import foot_clearance
from support_sweep import fore_aft_path

def configure(robot, config=None, *, placement=None):
    config=config or {}
    if 'touchdown_x_m' in config or 'liftoff_x_m' in config:
        if not all(key in config for key in ('touchdown_x_m','liftoff_x_m','cartesian_stride_m','period_s','duty')):
            raise ValueError('Explicit fore/aft endpoints need both endpoints, stride, period and duty')
        fore_aft_path(0.,config['duty'],config['period_s'],config['touchdown_x_m'],config['liftoff_x_m'])
        if not np.isclose(config['touchdown_x_m']-config['liftoff_x_m'],config['cartesian_stride_m'],atol=1e-9,rtol=0):
            raise ValueError('Fore/aft endpoint distance must equal declared stride')
    kin=SupportShift(robot.plant.model)
    old=np.tile([0.,45.,90.],4);kin.set_angles(old)
    original=np.array([kin.foot(i) for i in range(4)])
    shoulders=np.array([kin.data.xanchor[kin.model.joint(l+'_j2').id].copy() for l in LEGS])
    goals=original.copy();goals[:,0]=shoulders[:,0];goals[:,2]=original[:,2].mean()-float(config.get('body_raise_m',0))
    if 'stance_half_width_m' in config:
        half_width=float(config['stance_half_width_m'])
        if not .045<=half_width<=.12:raise ValueError('Invalid experimental stance half width')
        goals[:,1]=np.sign(goals[:,1])*half_width
    neutral,residual=kin.solve(goals,old,iterations=30)
    if residual>.0001:raise RuntimeError(f'Neutral unreachable: {residual}')
    # J2 anchor moves slightly with J1; converge on its solved position.
    for _ in range(3):
        kin.set_angles(neutral)
        shoulders=np.array([kin.data.xanchor[kin.model.joint(l+'_j2').id].copy() for l in LEGS])
        goals[:,0]=shoulders[:,0]
        neutral,residual=kin.solve(goals,neutral,iterations=30)
    kin.set_angles(neutral);actual=np.array([kin.foot(i) for i in range(4)])
    centroidal=None
    if config.get('centroidal_lateral'):
        from centroidal_lateral_reference import CentroidalLateralReference
        center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
        centroidal=CentroidalLateralReference(goals,center,config['period_s'],config['duty'],config['cartesian_stride_m'])
    info=dict(shoulders_m=shoulders.tolist(),neutral_deg=neutral.reshape(4,3).tolist(),foot_goals_m=goals.tolist(),old_feet_m=original.tolist(),neutral_residual_m=residual,max_path_residual_m=0.,max_command_step_deg=0.)
    plant=robot.plant;plant.desired=np.radians(neutral);plant.data.qpos[plant.q]=plant.desired
    mujoco.mj_forward(plant.model,plant.data)
    plant.data.qpos[2]+=.001-min(foot_clearance(plant.model,plant.data,g) for g in kin.feet)
    mujoco.mj_forward(plant.model,plant.data)
    plant.filtered=plant.desired.copy();plant.delay=__import__('collections').deque([plant.desired.copy() for _ in plant.delay]);plant.target_velocity[:]=0
    robot.target=neutral.copy();robot.command_target=neutral.copy()
    old_step=virtual_robot.shared_drive_step
    previous=neutral.copy()
    lateral_shift=0.
    custom_phase=0.
    from ground_foothold_transfer import FootholdTransfer
    footholds=FootholdTransfer()
    from capture_foot_placement import CaptureFootPlacement
    capture=CaptureFootPlacement()
    def step(r):
        nonlocal previous,lateral_shift,custom_phase
        if r is not robot:return old_step(r)
        evaluated_phase=custom_phase if config.get("period_s") else r.phase
        nominal=old_step(r)
        if r.safety!='ok':return nominal
        # The shared kernel may end a motion (for example a tracking stop).
        # Do not overwrite its held target/transition with another gait frame.
        if r.motion is None or r.transition is not None:return r.target.copy()
        params=r.active_profile_params()
        base=foot_targets(params,0,0)
        kin.set_angles(base);reference=np.array([kin.foot(i) for i in range(4)])
        kin.set_angles(nominal);points=np.array([kin.foot(i) for i in range(4)])
        targets=goals+(points-reference)
        # Explicit CAD cushion-bottom trajectory: 32mm at full forward.
        # No arbitrary joint-angle boost; solve coordinated J2/J3 via Jacobian.
        amplitude=smooth(min(1.,r.elapsed/float(config.get('startup_s',1))))*min(1.,abs(r.linear))
        duty=float(config.get('duty',params[1]))
        if config.get('period_s'):
            custom_phase=(custom_phase+.02/float(config['period_s']))%1
            r.phase=custom_phase
            info['period_s']=config['period_s'];info['duty']=duty
        if config.get('cartesian_stride_m'):
            for leg,offset in enumerate((0.,.5,.5,0.)):
                ph=(evaluated_phase+offset)%1
                lam=float(config.get('stance_hyperbolic',0))
                if ph<duty:
                    x=-.5*np.sinh(lam*(2*ph/duty-1))/np.sinh(lam) if lam>0 else .5-ph/duty
                else:
                    u=(ph-duty)/(1-duty);k=(1-duty)/duty
                    if lam>0:k*=lam/np.tanh(lam)
                    x=-.5+(1+k)*smooth(u)-k*u
                targets[leg,0]=goals[leg,0]+config['cartesian_stride_m']*amplitude*x
                if 'touchdown_x_m' in config:
                    path_x,_,_=fore_aft_path(ph,duty,config['period_s'],config['touchdown_x_m'],config['liftoff_x_m'])
                    targets[leg,0]=goals[leg,0]+amplitude*path_x

        for leg,offset in enumerate((0.,.5,.5,0.)):
            q=(evaluated_phase+offset)%1.
            lift=0.
            if q>=duty:
                u=(q-duty)/(1-duty)
                rise=float(config.get('lift_rise_fraction',.4));fall=float(config.get('lift_fall_start',.7))
                held=smooth(u/rise) if u<rise else smooth((1-u)/(1-fall)) if u>fall else 1.
                arch=64*u**3*(1-u)**3
                lift=arch+(held-arch)*smooth(min(1.,max(0.,r.linear/.5)))
                if config.get('lift_shape')=='arch':lift=arch
                if config.get('lift_shape')=='sin2':lift=np.sin(np.pi*u)**2
                if config.get('lift_shape')=='quartic':lift=16*u*u*(1-u)*(1-u)
            targets[leg,2]=goals[leg,2]+.032*amplitude*lift
        if placement is not None:
            targets=placement.update(targets,goals,r,evaluated_phase,duty,
                                     float(config.get('period_s',params[0])))
            info['placement']=placement.diagnostic.copy()
        if config.get('support_transfer_gain',0):
            kin.set_angles(previous)
            center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
            # Blend the two diagonal support-line targets across double support.
            shifts=[]
            for pair in ((0,3),(1,2)):
                a,b=targets[list(pair),:2]
                line_y=a[1]+(center[0]-a[0])*(b[1]-a[1])/(b[0]-a[0])
                shifts.append(center[1]-line_y)
            window=max(.02,duty-.5)
            blend=smooth(evaluated_phase/window) if evaluated_phase<.5 else 1-smooth((evaluated_phase-.5)/window)
            desired=blend*shifts[0]+(1-blend)*shifts[1]
            desired=np.clip(config['support_transfer_gain']*desired,-.01,.01)
            lateral_shift+=np.clip(desired-lateral_shift,-.0008,.0008)
            targets[:,1]+=amplitude*lateral_shift
            info['max_lateral_transfer_m']=max(info.get('max_lateral_transfer_m',0),abs(lateral_shift))
        if config.get('foothold_transfer'):
            kin.set_angles(previous)
            center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
            targets=footholds.update(targets,goals,center,evaluated_phase,duty,amplitude,config)
            if config.get('body_shift_after_placement'):targets[:,1]+=amplitude*lateral_shift
            info['foothold_transfer']=footholds.diagnostic.copy()
        if config.get('capture_gain'):
            kin.set_angles(previous)
            center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
            height=float(center[2]-goals[:,2].mean())
            targets[:,1]+=capture.update(evaluated_phase,duty,r.attitude_filter,height,
                {'gain':config['capture_gain'],'position_gain':config.get('capture_position_gain',1),'rate_gain':config.get('capture_rate_gain',.5),'limit_m':.01})
            info['capture']=capture.diagnostic.copy()
        if config.get('roll_feedforward'):
            kin.set_angles(previous)
            center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
            ph=evaluated_phase+float(config.get('feedforward_lead_s',0))/float(config.get('period_s',params[0]))
            coeff=np.asarray(config['roll_feedforward'],dtype=float)
            basis=np.array([f(2*np.pi*h*ph) for h in range(1,len(coeff),2) for f in (np.sin,np.cos)])
            angle=float(coeff@basis)
            angle*=amplitude*smooth(max(0.,(r.elapsed-float(config.get('feedforward_start_s',0)))/float(config.get('feedforward_ramp_s',1))))
            if abs(angle)>.15:raise ValueError('Feedforward rotation exceeds limit')
            cr,sr=np.cos(angle),np.sin(angle)
            rotation=np.array([[1.,0,0],[0,cr,-sr],[0,sr,cr]])
            targets=(targets-center)@rotation.T+center
            if config.get('pitch_feedforward'):
                pc=np.asarray(config['pitch_feedforward']);pa=float(pc@np.array([1,np.sin(4*np.pi*ph),np.cos(4*np.pi*ph),np.sin(8*np.pi*ph),np.cos(8*np.pi*ph)]))
                pa*=amplitude*smooth(max(0.,(r.elapsed-float(config.get('feedforward_start_s',0)))/float(config.get('feedforward_ramp_s',1))))
                if abs(pa)>.1:raise ValueError('Pitch feedforward exceeds bound')
                cp,sp=np.cos(pa),np.sin(pa);pr=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
                targets=(targets-center)@pr.T+center
            info['roll_feedforward_rad']=angle
        if centroidal is not None:
            body_y=amplitude*centroidal.at(evaluated_phase+float(config.get('centroidal_lead_s',0))/config['period_s'])
            targets[:,1]-=body_y
            info['centroidal_lateral']={**centroidal.diagnostic,'body_y_m':body_y,
                'within_10mm_body_reference':centroidal.diagnostic['planned_within_10mm']}
        info['peak_lift_target_mm']=32.
        result,error=kin.solve(targets,previous,iterations=20)
        info['max_path_residual_m']=max(info['max_path_residual_m'],error)
        info['max_command_step_deg']=max(info['max_command_step_deg'],float(np.max(abs(result-previous))))
        if error>.001:raise RuntimeError(f'Gait target unreachable: {error}')
        r.ground_evaluated_phase=evaluated_phase
        r.ground_duty=duty
        previous=result.copy();return result
    virtual_robot.shared_drive_step=step
    # No separate 1-second shift to the old rearward neutral before first step.
    original_begin=robot.begin
    def begin(motion,now):
        nonlocal previous,lateral_shift,custom_phase
        original_begin(motion,now)
        previous=neutral.copy();lateral_shift=0.;custom_phase=0.
        footholds.reset();capture.reset()
        if placement is not None:
            (placement.reset_motion if hasattr(placement,'reset_motion') else placement.reset)()
        robot.ground_evaluated_phase=0.
        info['roll_feedforward_rad']=0.
        robot.transition=None;robot.target=neutral.copy()
    robot.begin=begin
    original_finish_stop=robot.finish_stop
    def finish_stop(reason):
        # Begin from the actual last command, including residual corrections.
        # No simulator qpos/qvel or servo feedback is overwritten here.
        corrected=robot.command_target.copy()
        original_finish_stop(reason)
        if robot.transition is not None:
            _,_,elapsed,duration,completion=robot.transition
            robot.target=corrected.copy();robot.command_target=corrected.copy()
            robot.transition=(corrected,neutral.copy(),elapsed,duration,completion)
            info['roll_feedforward_rad']=0.
    robot.finish_stop=finish_stop
    original_reply=robot.reply
    def reply(message='OK'):
        # Shared tick recognizes only legacy45/90 as standing after a generic
        # stop completion. Tag this preview's converged neutral consistently,
        # while retaining the unchanged protocol completion response.
        if (message.startswith('$SPOTDRIVE stopped reason=') and robot.motion is None
                and robot.transition is None and np.allclose(robot.target,neutral,atol=1e-6)):
            robot.pose='stand'
        original_reply(message)
    robot.reply=reply
    robot.ground_nominal_info=info
    return info
