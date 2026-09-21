"""Apply the firmware Cartesian swing adapter to simulator nominal targets."""
import ctypes as ct
import numpy as np

def apply(robot, target):
    mm=getattr(robot,'foot_lift_mm',[0,0,0,0])
    if not any(mm):return target
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
    values=(ct.c_float*12)(*target)
    fn=robot.plant.policy._library.spot_foot_lift
    fn.argtypes=[ct.POINTER(ct.c_uint32),ct.c_float,ct.c_float,ct.c_float,ct.POINTER(ct.c_float),ct.POINTER(ct.c_float)]
    fn.restype=ct.c_int
    if not fn((ct.c_uint32*4)(*mm),phase,duty,scale,(ct.c_float*4)(*offsets),values):
        raise ValueError('Additional foot lift unreachable; command held')
    return np.array(values)
