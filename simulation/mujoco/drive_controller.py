"""50 Hz deployed motion kernel, using the same C source as STM32."""
import ctypes
import numpy as np

from gait_profiles import load_profiles
NAMES=('legacy',*load_profiles())

def _arc_options(parameters):
    """Validate before a C call can read a configuration with the wrong arity."""
    if parameters.get('arc_swing_shape_trial') is not None:
        shape=np.asarray(parameters['arc_swing_shape_trial'],dtype=float)
        if shape.shape!=(4,) or not np.isfinite(shape).all() or np.any(np.abs(shape)>.01):
            raise ValueError('Arc swing shaping requires four finite offsets within +/-10mm')
    if parameters.get('arc_dynamic_trial') is not None:
        dynamic=np.asarray(parameters['arc_dynamic_trial'],dtype=float)
        if dynamic.shape!=(4,) or not np.isfinite(dynamic).all():
            raise ValueError('Dynamic arc balance requires four finite parameters')
        if any(parameters.get(name) for name in ('arc_com_trial','arc_shift_trial','arc_tripod_shift','arc_posture_trial','arc_tripod_trial')):
            raise ValueError('Dynamic arc balance cannot combine independent body-shift controllers')
        wave=np.asarray(parameters.get('arc_wave_trial',[0.]*6),dtype=float)
        if wave.shape!=(6,) or not np.isfinite(wave).all():
            raise ValueError('Offline body wave requires six finite parameters')
        # Conservative all-phase envelope, leaving 4mm for dynamic residual.
        extent=np.abs(wave[:2])+np.array([np.hypot(*wave[2:4]),np.hypot(*wave[4:6])])
        if np.linalg.norm(extent)>.006:
            raise ValueError('Offline body wave plus dynamic residual exceeds the 10mm budget')
    options=parameters.get('arc_trial')
    tripod=bool(parameters.get('arc_tripod_trial'))
    if options is None:
        if tripod:raise ValueError('Tripod arc trial requires six trajectory parameters')
        return None
    try:
        values=np.asarray(options,dtype=float)
    except (TypeError,ValueError) as exc:
        raise ValueError('Arc trial requires four or six finite parameters') from exc
    if values.ndim!=1 or len(values) not in (4,6) or not np.isfinite(values).all():
        raise ValueError('Arc trial requires four or six finite parameters')
    if tripod and len(values)!=6:
        raise ValueError('Tripod arc trial requires six trajectory parameters')
    bounds=((.02,.06),(.5,.85 if tripod else .65),(0,.12),(0,1),(.5,2),(.05,.4))
    if any(not low<=value<=high for value,(low,high) in zip(values,bounds)):
        raise ValueError('Arc trajectory parameter is outside its geometric range')
    return values.tolist()

def step(robot):
    options=_arc_options(robot.plant.p) if robot.profile=='arcturn' else None
    support=robot.profile=='arcsupport'
    fn=robot.plant.policy._library.spot_drive_step_stateful if support else robot.plant.policy._library.spot_drive_step_timed
    fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_float,ctypes.c_float,fp)
    extra=[]
    if support:
        fn.argtypes=(fp,*fn.argtypes)
        if not hasattr(robot,'arc_support_state'):robot.arc_support_state=(ctypes.c_float*12)()
        extra=[robot.arc_support_state]
    fn.restype=ctypes.c_int
    v=(ctypes.c_float*11)(robot.phase,robot.linear,robot.yaw,robot.elapsed,*robot.heading.state,robot.turn_assist)
    out=(ctypes.c_float*12)()
    now=float(robot.plant.data.time)
    if robot.elapsed==0:robot.tracking.reset(now)
    rate=robot.tracking.step(now,.02,bool(robot.stopping_reason)) if robot.tracking_enabled else 1.
    if robot.tracking_enabled and robot.tracking.diagnostic.get('fault'):
        robot.motion=None;robot.transition=None;robot.stopping_reason=None
        robot.request=(0.,0.);robot.linear=robot.yaw=0.
        robot.safety='tracking';robot.pose='custom'
        robot.reply('$SPOTDRIVE stopped reason=tracking feedback fault')
        return robot.command_target.copy()
    sample=robot.imu_reading
    valid=sample is not None and sample['age_ms']<=100
    if robot.plant.p.get('arc_observer_trial') is not None:
        from types import SimpleNamespace
        if not hasattr(robot,'arc_observer'):
            from arc_observer import ArcObserver
            robot.arc_observer=ArcObserver()
        robot.arc_observer.update(sample,now)
        estimate=robot.arc_observer.read(now,lookahead_s=float(robot.plant.p['arc_observer_trial']))
        robot.arc_sensor_available=estimate is not None
        if estimate:
            robot.arc_sensor=SimpleNamespace(filtered=np.array(estimate['angle_deg'])*10,rate=np.array(estimate['rate_deg_s'])*10)
    if not fn(v,*extra,NAMES.index(robot.profile),*robot.request,sample['yaw_tenths']/10 if valid else 0,
              valid,robot.heading.enabled and robot.safety=='ok',bool(robot.stopping_reason),.02,rate,out):
        raise ValueError('Invalid shared drive target')
    if robot.profile in ('arcturn','arcsupport'):
        # Snapshot the frame that produced the target, before publishing the
        # next phase. All downstream CAD control uses this one time basis.
        fraction=abs(v[2])/max(abs(v[1])+abs(v[2]),1.e-9)
        lead_period=1.44-.24*fraction
        period_fn=robot.plant.policy._library.spot_locomotion_period
        period_fn.argtypes=(ctypes.c_int,ctypes.c_float,ctypes.c_float)
        period_fn.restype=ctypes.c_float
        period=float(period_fn(NAMES.index(robot.profile),v[1],v[2]))
        config=options if options is not None else [.02,.5,.04,0]
        if len(config)>4:
            period=float(config[4])
            lead_period=period
            v[0]=(float(robot.phase)+.02*rate/period)%1
        robot.arc_frame=dict(phase=float(robot.phase),period_s=period,
            lead_period_s=lead_period,
            target_phase=(float(robot.phase)+config[2]/lead_period)%1.,
            duty=float(config[1]),scale=robot.plant.policy.smootherstep(min(1,v[3])))
        robot.arc_frame['motion_scale']=robot.arc_frame['scale']*min(1.,abs(v[1])+abs(v[2]))
        robot.arc_frame['swing_scale']=robot.arc_frame['scale']*max(min(1.,abs(v[1])+abs(v[2])),
            robot.plant.policy.smootherstep(min(1.,abs(v[2])/.5)))
        robot.arc_frame['offsets']=[0,.5,.75,.25] if robot.plant.p.get('arc_tripod_trial') else [0,.5,.5,0]
    if options is not None and robot.profile=='arcturn':
        trial=robot.plant.policy._library.spot_arc_trajectory if len(options)>4 else robot.plant.policy._library.spot_arc_configured
        if robot.plant.p.get('arc_tripod_trial'):trial=robot.plant.policy._library.spot_arc_tripod
        trial.argtypes=(*([ctypes.c_float]*4),fp,fp);trial.restype=ctypes.c_int
        if not trial(robot.phase,robot.plant.policy.smootherstep(min(1,v[3])),v[1],v[2],(ctypes.c_float*len(options))(*options),out):
            raise ValueError('Arc clearance trial is infeasible')
    if robot.plant.p.get('arc_lift_first_trial') is not None and robot.profile=='arcturn':
        fraction=float(robot.plant.p['arc_lift_first_trial'])
        if not np.isfinite(fraction) or not .05<=fraction<=.3:
            raise ValueError('Lift-first fraction must be between .05 and .3')
        if options is None or robot.plant.p.get('arc_tripod_trial'):
            raise ValueError('Lift-first requires a diagonal arc trajectory')
        fnlift=robot.plant.policy._library.spot_arc_lift_first
        fnlift.argtypes=(fp,*([ctypes.c_float]*7),fp);fnlift.restype=ctypes.c_int
        lifted=(ctypes.c_float*12)()
        if not fnlift(out,robot.arc_frame['target_phase'],robot.arc_frame['scale'],v[1],v[2],
                      options[1],options[5] if len(options)>4 else .2,fraction,lifted):
            raise ValueError('Lift-first Cartesian target infeasible')
        out=lifted
    if robot.plant.p.get('arc_posture_trial') and robot.profile=='arcturn':
        posture=robot.plant.policy._library.spot_arc_posture
        posture.argtypes=(fp,fp,ctypes.c_float,fp);posture.restype=ctypes.c_int
        posed=(ctypes.c_float*12)()
        if not posture(out,(ctypes.c_float*3)(*robot.plant.p['arc_posture_trial']),robot.arc_frame['scale'],posed):
            raise ValueError('Arc posture target infeasible')
        out=posed
    if robot.plant.p.get('arc_swing_shape_trial') is not None and robot.profile=='arcturn':
        if not hasattr(robot,'arc_swing_shape'):
            from arc_swing_shape import ArcSwingShape
            robot.arc_swing_shape=ArcSwingShape()
        swing_frame={**robot.arc_frame,'motion_scale':robot.arc_frame['swing_scale']}
        shaped=robot.arc_swing_shape.apply(out,swing_frame,robot.plant.p['arc_swing_shape_trial'])
        out=(ctypes.c_float*12)(*shaped)
    if robot.plant.p.get('arc_wave_trial') and robot.profile=='arcturn':
        fnwave=robot.plant.policy._library.spot_arc_wave
        fnwave.argtypes=(fp,ctypes.c_float,ctypes.c_float,fp,fp);fnwave.restype=ctypes.c_int
        shifted=(ctypes.c_float*12)()
        if not fnwave(out,robot.arc_frame['target_phase'],robot.arc_frame['motion_scale'],(ctypes.c_float*6)(*robot.plant.p['arc_wave_trial']),shifted):
            raise ValueError('Arc offline body trajectory infeasible')
        out=shifted
    if robot.plant.p.get('arc_tripod_shift') and robot.profile=='arcturn':
        if not hasattr(robot,'arc_tripod_state') or robot.elapsed==0:robot.arc_tripod_state=(ctypes.c_float*2)()
        frame=robot.arc_frame;shifted=(ctypes.c_float*12)()
        tripod=robot.plant.policy._library.spot_arc_tripod_support
        tripod.argtypes=(fp,fp,ctypes.c_float,ctypes.c_float,ctypes.c_float,fp,fp);tripod.restype=ctypes.c_int
        if not tripod(robot.arc_tripod_state,out,frame['target_phase'],frame['period_s'],frame['duty'],(ctypes.c_float*2)(*robot.plant.p['arc_tripod_shift']),shifted):
            raise ValueError('Tripod support margin infeasible')
        out=shifted
    if robot.plant.p.get('arc_shift_trial') and robot.profile=='arcturn':
        if not hasattr(robot,'arc_shift_state') or robot.elapsed==0:robot.arc_shift_state=(ctypes.c_float*2)()
        width,gain,lead=robot.plant.p['arc_shift_trial'];frame=robot.arc_frame
        shift=robot.plant.policy._library.spot_arc_shift
        shift.argtypes=(fp,ctypes.c_float,ctypes.c_float,ctypes.c_float,fp,fp);shift.restype=ctypes.c_int
        shifted=(ctypes.c_float*12)()
        if not shift(out,frame['target_phase']+lead/frame['period_s'],width/frame['period_s'],gain,robot.arc_shift_state,shifted):
            raise ValueError('Arc support translation infeasible')
        out=shifted
    if robot.plant.p.get('arc_com_trial') and robot.profile=='arcturn':
        if not hasattr(robot,'arc_com_state') or robot.elapsed==0:robot.arc_com_state=(ctypes.c_float*2)()
        config=robot.plant.p['arc_com_trial'];frame=robot.arc_frame
        sensed=getattr(robot,'arc_sensor',robot.attitude_filter)
        imu=np.radians(np.array([*sensed.filtered,*sensed.rate])/10)
        feedback_sign=float(robot.plant.p.get('arc_com_sign',1.))
        if feedback_sign not in (-1.,1.):raise ValueError('Invalid experimental COM feedback sign')
        imu*=feedback_sign
        if (not valid or robot.attitude_filter.failures or not robot.balance.enabled
                or robot.safety!='ok' or not getattr(robot,'arc_sensor_available',True)):
            imu[:]=0
        com=robot.plant.policy._library.spot_arc_com
        com.argtypes=(fp,ctypes.c_float,ctypes.c_float,fp,fp,fp,fp);com.restype=ctypes.c_int
        shifted=(ctypes.c_float*12)()
        if not com(out,frame['target_phase'],frame['period_s'],(ctypes.c_float*4)(*imu),(ctypes.c_float*4)(*config),robot.arc_com_state,shifted):
            raise ValueError('Arc CoM residual infeasible')
        out=shifted
    if robot.plant.p.get('arc_dynamic_trial') is not None and robot.profile=='arcturn':
        if not hasattr(robot,'arc_dynamic'):
            from arc_dynamic_balance import ArcDynamicBalance
            robot.arc_dynamic=ArcDynamicBalance()
        available=(valid and not robot.attitude_filter.failures and robot.balance.enabled
            and robot.safety=='ok' and getattr(robot,'arc_sensor_available',True)
            and robot.arc_frame['motion_scale']>.001)
        out=robot.arc_dynamic.apply(out,robot.arc_frame,getattr(robot,'arc_sensor',robot.attitude_filter),
            available,robot.plant.p['arc_dynamic_trial'])
    robot.phase,robot.linear,robot.yaw,robot.elapsed=v[:4]
    robot.heading.state[:]=v[4:10]
    robot.turn_assist=v[10]
    return np.array(out)
