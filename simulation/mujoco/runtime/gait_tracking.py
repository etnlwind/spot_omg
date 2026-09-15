"""Same MCU supervisor, with one timestamped/quantized encoder per 20ms frame."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import ctypes
import numpy as np

class GaitTracking:
    def __init__(self,policy):
        self.lib=policy._library;self.index=0;self.rate=1.;self.diagnostic={}
        lib=self.lib;ptr=ctypes.c_void_p;u=ctypes.c_uint32;f=ctypes.c_float
        lib.spot_tracking_size.restype=ctypes.c_uint
        lib.spot_tracking_reset.argtypes=(ptr,u)
        lib.spot_tracking_sample.argtypes=(ptr,ctypes.c_uint,f,u)
        lib.spot_tracking_step.argtypes=(ptr,u,f,ctypes.c_int,ctypes.POINTER(f));lib.spot_tracking_step.restype=f
        lib.spot_tracking_step_responsive.argtypes=lib.spot_tracking_step.argtypes
        lib.spot_tracking_step_responsive.restype=f
        self.state=ctypes.create_string_buffer(lib.spot_tracking_size());self.reset(0)
    def reset(self,now):
        self.lib.spot_tracking_reset(self.state,round(now*1000));self.index=0
    def sample(self,now,target,actual,drop=False):
        i=self.index;self.index=(i+1)%12
        if drop:return
        # Target goes through the deployed encoder; feedback has one tick resolution.
        fp=ctypes.POINTER(ctypes.c_float);fn=self.lib.spot_servo_encode
        fn.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp);fn.restype=ctypes.c_int
        ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
        if not fn((ctypes.c_float*12)(*target),ticks,decoded):raise ValueError('Invalid tracking target')
        measured=round(float(actual[i])*4096/360)*360/4096
        self.lib.spot_tracking_sample(self.state,i,measured-decoded[i],round(now*1000))
    def step(self,now,dt,stopping,responsive=False):
        diag=(ctypes.c_float*4)()
        fn=self.lib.spot_tracking_step_responsive if responsive else self.lib.spot_tracking_step
        self.rate=float(fn(self.state,round(now*1000),dt,stopping,diag))
        self.diagnostic=dict(rate=self.rate,peak_error_deg=diag[0],oldest_ms=diag[1],fault=int(diag[2]),blocked_ms=diag[3],contact='unobserved')
        return self.rate
