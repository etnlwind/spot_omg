"""Fixed CAD gravity estimate shared with C; no simulator state or forces."""
import ctypes
import numpy as np


class ArcPreload:
    def __init__(self, policy):
        fp = ctypes.POINTER(ctypes.c_float)
        self.fn = policy._library.spot_arc_preload
        self.fn.argtypes = (fp, fp, *([ctypes.c_float] * 4), fp)
        self.fn.restype = ctypes.c_int
        self.state = (ctypes.c_float * 12)()

    def reset(self):
        self.state = (ctypes.c_float * 12)()

    def correction(self, target, phase, duty, gain):
        values = (ctypes.c_float * 12)(*target)
        result = (ctypes.c_float * 12)()
        if not self.fn(self.state, values, phase, duty, gain, 35., result):
            raise ValueError('Arc gravity preload target infeasible')
        return np.asarray(result).copy() - np.asarray(target)

class ArcTransfer:
    def __init__(self,policy):
        f=ctypes.c_float;fp=ctypes.POINTER(f)
        self.fn=policy._library.spot_arc_transfer
        self.fn.argtypes=(fp,fp,f,f,f,fp,f,fp);self.fn.restype=ctypes.c_int
        self.state=(f*12)()
    def reset(self):self.state=(ctypes.c_float*12)()
    def correction(self,target,frame,config):
        f=ctypes.c_float;out=(f*12)()
        if not self.fn(self.state,(f*12)(*target),frame['target_phase'],frame['period_s'],frame['duty'],(f*3)(*config),.02,out):
            raise ValueError('CAD support transfer infeasible')
        return np.array(out)-np.asarray(target)
