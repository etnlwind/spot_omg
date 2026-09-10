"""Shared hardware/simulator Stow: stop driving before inter-leg contact.

Derived from full CAD triangles by stow_clearance.py (18 mm design margin).
At completion, release motor torque so gravity may gently seat the legs.
This is not a physical-servo calibration or hardware approval.
"""
LANDING = [0., 40., 130.] * 4
FOLDED = [0., -254.63, 4.42] * 2 + [0., -80.75, 4.42] * 2
FOLD_SECONDS = 12.
SETTLE_SECONDS = 3.
RELEASE_TORQUE_AT_STOW = True
CAPABILITY = 'stow'

# Production trajectory/encoder binding. The same C functions run on STM32.
import ctypes
from servo import SharedGaitPolicy
_SHARED = None

def _library():
    global _SHARED
    if _SHARED is None:
        _SHARED=SharedGaitPolicy()._library
        fp=ctypes.POINTER(ctypes.c_float);ip=ctypes.POINTER(ctypes.c_int32)
        _SHARED.spot_stow_frame.argtypes=[fp,ctypes.c_int,ctypes.c_uint32,ip,fp]
        _SHARED.spot_stow_frame.restype=ctypes.c_int
        _SHARED.spot_stow_encode.argtypes=[fp,ip,fp]
        _SHARED.spot_stow_encode.restype=ctypes.c_int
    return _SHARED

def frame(start, folded, elapsed_s):
    ticks=(ctypes.c_int32*12)();values=(ctypes.c_float*12)()
    if not _library().spot_stow_frame((ctypes.c_float*12)(*start),folded,round(elapsed_s*1000),ticks,values):
        raise ValueError('Stow target outside physical actuator envelope')
    return list(values),list(ticks)

def encode(values):
    ticks=(ctypes.c_int32*12)();quantized=(ctypes.c_float*12)()
    if not _library().spot_stow_encode((ctypes.c_float*12)(*values),ticks,quantized):
        raise ValueError('Stow target outside physical actuator envelope')
    return list(quantized),list(ticks)


def attitude_ok(reading):
    import math
    fn=_library().spot_stow_attitude
    fn.argtypes=[ctypes.c_int]*3;fn.restype=ctypes.c_int
    return bool(fn(reading is not None,0 if reading is None else reading["roll_tenths"],0 if reading is None else reading["pitch_tenths"]))


def pose_frame(start,end,elapsed_s=0):
    fn=_library().spot_pose_frame
    fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=[fp,fp,ctypes.c_uint32,fp];fn.restype=ctypes.c_int
    values=(ctypes.c_float*12)()
    duration=fn((ctypes.c_float*12)(*start),(ctypes.c_float*12)(*end),round(elapsed_s*1000),values)
    if duration<0:raise ValueError('Pose outside physical encoder range')
    return list(values),duration/1000
