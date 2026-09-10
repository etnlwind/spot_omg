"""50 Hz deployed motion kernel, shared byte-for-byte with STM32."""
import ctypes
import numpy as np

from gait_profiles import load_profiles
NAMES=('legacy',*load_profiles())

def step(robot):
    fn=robot.plant.policy._library.spot_drive_step
    fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_int,ctypes.c_int,fp)
    fn.restype=ctypes.c_int
    v=(ctypes.c_float*11)(robot.phase,robot.linear,robot.yaw,robot.elapsed,*robot.heading.state,robot.turn_assist)
    out=(ctypes.c_float*12)()
    sample=robot.imu_reading
    valid=sample is not None and sample['age_ms']<=100
    if not fn(v,NAMES.index(robot.profile),*robot.request,sample['yaw_tenths']/10 if valid else 0,
              valid,robot.heading.enabled and robot.safety=='ok',bool(robot.stopping_reason),out):
        raise ValueError('Invalid shared drive target')
    robot.phase,robot.linear,robot.yaw,robot.elapsed=v[:4]
    robot.heading.state[:]=v[4:10]
    robot.turn_assist=v[10]
    return np.array(out)
