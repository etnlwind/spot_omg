"""Apply the firmware Cartesian swing adapter to simulator nominal targets."""
import ctypes as ct
import numpy as np

def apply(robot, target):
    mm=getattr(robot,'foot_lift_mm',[0,0,0,0])
    widths=getattr(robot,"foot_width_mm",[0]*4)
    nav=getattr(robot,'navigation_frame',{})
    if not any(mm) and not any(widths) and not nav.get('ipsilateral_side'):return target
    if len(mm)!=4 or any(not 0<=v<=2147483647 for v in mm):raise ValueError('Invalid foot lift')
    phase=robot.nominal_phase
    kind=robot.motion[0]
    offsets=[0,.5,.5,0]
    if kind in ('drive','profile') and robot.profile!='legacy':
        duty=robot.active_profile_params()[1]
        family=robot.profiles[robot.profile]['family']
        offsets={'trot':[0,.5,.5,0],'crawl':[0,.5,.75,.25],'amble':[0,.5,.75,.25],
                 'pace':[0,.5,0,.5],'bound':[0,0,.5,.5]}.get(family,offsets)
        if robot.profile in ('arcturn','arcsupport'):
            fraction=abs(robot.yaw)/max(abs(robot.linear)+abs(robot.yaw),1e-9)
            phase+=.04/(1.44-.24*fraction);duty=.5
    else:
        duty={'trot':.5,'trot2':.5,'trot3':.65,'trot4':.6,'trot4back':.6,
              'turn':.6,'trot5':.5979488437760033,'crab':.8,'drive':.6,'profile':.6}[kind]
        if kind=='crab':offsets=[.8,.3,.05,.55]
    scale=robot.gait_start_scale()
    if kind in ('drive','profile'):scale*=min(1,(abs(robot.linear)+abs(robot.yaw))/.15)
    nav=getattr(robot,'navigation_frame',{})
    if robot.profile in ('attitudepd_v7','attitudepd_v8','attitudepd_v9','attitudepd_v10') and nav.get('sideways'):
        duty=.8;offsets=[.8,.3,.05,.55]
        if nav['lateral']<0:offsets=[.3,.8,.55,.05]
        if nav.get('paired_side'):
            duty=nav['duty'];offset=nav['pair_offset'];offsets=[offset,offset-.5,offset-.5,offset]
        scale=robot.plant.policy.smootherstep(min(1,robot.elapsed/(1.5 if nav.get('paired_side') else 1)))*min(1,(abs(nav['lateral'])+abs(robot.yaw))/.15)
    values=(ct.c_float*12)(*target)
    fn=robot.plant.policy._library.spot_foot_lift
    fn.argtypes=[ct.POINTER(ct.c_uint32),ct.c_float,ct.c_float,ct.c_float,ct.POINTER(ct.c_float),ct.POINTER(ct.c_float)]
    fn.restype=ct.c_int
    if nav.get('left_instep'):valid=True
    elif nav.get('paired_side'):
        extra=robot.plant.policy._library.spot_navigation_v9_extra_lift if nav.get('ipsilateral_side') else robot.plant.policy._library.spot_navigation_pair_extra_lift
        extra.argtypes=[ct.POINTER(ct.c_uint32),ct.c_float,ct.c_float,ct.POINTER(ct.c_float)];extra.restype=ct.c_int
        valid=extra((ct.c_uint32*4)(*mm),phase,scale,values)
    else:valid=fn((ct.c_uint32*4)(*mm),phase,duty,scale,(ct.c_float*4)(*offsets),values)
    if not valid:
        raise ValueError('Additional foot lift unreachable; command held')
    width_fn=robot.plant.policy._library.spot_foot_width_cad if robot.profile in ('attitudepd_v7','attitudepd_v8','attitudepd_v9','attitudepd_v10') else robot.plant.policy._library.spot_foot_width
    width_fn.argtypes=[ct.POINTER(ct.c_int32),ct.c_float,ct.POINTER(ct.c_float)]
    width_fn.restype=ct.c_int
    if not width_fn((ct.c_int32*4)(*widths),scale,values):raise ValueError('Foot width unreachable; command held')
    if nav.get('left_instep'):
        fn=robot.plant.policy._library.spot_navigation_v10_targets
        fn.argtypes=[ct.c_float,ct.c_float,ct.c_float,ct.POINTER(ct.c_uint32),ct.POINTER(ct.c_float)];fn.restype=ct.c_int
        cfg=robot.plant.p.get('v10_experiment')
        if cfg is not None:
            if len(cfg)!=6 or not all(np.isfinite(cfg)) or not (0<cfg[2]<=.2 and 0<cfg[4]<=.16 and 0<=cfg[3]<=1 and 0<=cfg[5]<=.3):
                raise ValueError('Invalid V10 experiment parameters')
            fn=robot.plant.policy._library.spot_navigation_v10_configured
            fn.argtypes=[ct.c_float,ct.c_float,ct.c_float,ct.POINTER(ct.c_uint32),ct.POINTER(ct.c_float),ct.POINTER(ct.c_float)];fn.restype=ct.c_int
            valid=fn(phase,scale,nav['lateral'],(ct.c_uint32*4)(*mm),(ct.c_float*6)(*cfg),values)
        else:valid=fn(phase,scale,nav['lateral'],(ct.c_uint32*4)(*mm),values)
        if not valid:raise ValueError('V10 left instep target unreachable')
    elif nav.get('ipsilateral_side'):
        distance=float(robot.plant.p.get('v9_transfer_m',.06))
        transfer=robot.plant.policy._library.spot_navigation_v9_transfer
        transfer.argtypes=[ct.c_float,ct.c_float,ct.c_float,ct.POINTER(ct.c_float)];transfer.restype=ct.c_int
        if not transfer(phase,scale,distance,values):raise ValueError('V9 support transfer unreachable')
        nav['transfer_m']=distance
    return np.array(values)
