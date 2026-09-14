"""Shared CAD attitude residual controller; only delayed IMU is observed."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import ctypes
import numpy as np

class ArcAttitude:
    def __init__(self,policy):
        self.fn=policy._library.spot_arc_attitude
        f=ctypes.c_float;fp=ctypes.POINTER(f)
        self.fn.argtypes=(fp,fp,f,f,f,fp,ctypes.c_int,f,fp,fp)
        self.fn.restype=ctypes.c_int
        self.state=(f*4)()
    def apply(self,target,frame,attitude,available,config):
        f=ctypes.c_float;out=(f*12)()
        imu=np.radians(np.array([*attitude.filtered,*attitude.rate])/10)
        if not self.fn(self.state,(f*12)(*target),frame['target_phase'],frame['period_s'],frame['duty'],
            (f*4)(*imu),available,.02,(f*6)(*config),out):
            raise ValueError('CAD attitude residual infeasible')
        return np.array(out)
