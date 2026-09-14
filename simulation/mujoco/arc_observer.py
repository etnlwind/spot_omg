"""Standalone host binding of the optional shared-C BNO gait observer.

Does not run, replace, or relax the firmware's independent attitude safety.
"""
import ctypes
import hashlib
import math
from pathlib import Path
import subprocess
import tempfile


class _State(ctypes.Structure):
    _fields_ = [('angle_deg',ctypes.c_float*2),('rate_deg_s',ctypes.c_float*2),
                ('previous_deg',ctypes.c_float*2),('sampled_s',ctypes.c_double),
                ('updated_s',ctypes.c_double),('initialized',ctypes.c_bool),
                ('clock_initialized',ctypes.c_bool),('available',ctypes.c_bool)]


_LIBRARY = None


def _library():
    global _LIBRARY
    if _LIBRARY is None:
        header = Path(__file__).resolve().parents[2]/'firmware/stm32-learning/Inc/arc_observer.h'
        source = '''#include "arc_observer.h"
void observer_reset(ArcObserver*s){arc_observer_reset(s);}
int observer_update(ArcObserver*s,int valid,float r,float p,double sampled,double now){return arc_observer_update(s,valid,r,p,sampled,now);}
int observer_read(const ArcObserver*s,double now,float lookahead,float*out){return arc_observer_read(s,now,lookahead,out);}
'''
        key = hashlib.sha256(header.read_bytes()+source.encode()).hexdigest()[:20]
        cache = Path(tempfile.gettempdir())/'spot-arc-observer'
        cache.mkdir(exist_ok=True)
        library = cache/(key+'.dylib')
        if not library.exists():
            # Unique build paths avoid processes seeing a partly written library.
            with tempfile.TemporaryDirectory(dir=cache) as directory:
                cfile = Path(directory)/'observer.c'
                cfile.write_text(source)
                output = Path(directory)/'observer.dylib'
                subprocess.run(['cc','-shared','-fPIC','-O2','-std=c11','-I',str(header.parent),
                                str(cfile),'-o',str(output)],check=True,capture_output=True)
                output.replace(library)
        lib = ctypes.CDLL(str(library))
        pointer = ctypes.POINTER(_State)
        lib.observer_reset.argtypes = (pointer,)
        lib.observer_update.argtypes = (pointer,ctypes.c_int,ctypes.c_float,ctypes.c_float,
                                       ctypes.c_double,ctypes.c_double)
        lib.observer_update.restype = ctypes.c_int
        lib.observer_read.argtypes = (pointer,ctypes.c_double,ctypes.c_float,ctypes.POINTER(ctypes.c_float))
        lib.observer_read.restype = ctypes.c_int
        _LIBRARY = lib
    return _LIBRARY


class ArcObserver:
    def __init__(self):
        self.lib = _library()
        self.state = _State()

    def reset(self):
        self.lib.observer_reset(ctypes.byref(self.state))

    def update(self, reading, now_s):
        """Accept BNO ``roll_tenths/pitch_tenths/sample_time_s`` or None."""
        valid = reading is not None
        try:
            roll = float(reading['roll_tenths'])/10 if valid else 0.
            pitch = float(reading['pitch_tenths'])/10 if valid else 0.
            sampled = float(reading['sample_time_s']) if valid else 0.
        except (KeyError,TypeError,ValueError):
            valid=False;roll=pitch=sampled=0.
        return bool(self.lib.observer_update(ctypes.byref(self.state),valid,roll,pitch,sampled,float(now_s)))

    def read(self, now_s, lookahead_s=0.):
        """Return a degree-valued estimate or None when unavailable/stale.

        Lookahead is explicit and bounded to 100ms; timestamp age is reported,
        not added to the prediction. The caller still runs its normal safety.
        """
        if not math.isfinite(lookahead_s) or not 0<=lookahead_s<=.1:
            raise ValueError('Observer lookahead must be in [0, .1] seconds')
        out = (ctypes.c_float*4)()
        if not self.lib.observer_read(ctypes.byref(self.state),float(now_s),lookahead_s,out):
            return None
        return dict(available=True, angle_deg=list(out[:2]),rate_deg_s=list(out[2:]),
                    sample_time_s=self.state.sampled_s,age_s=float(now_s)-self.state.sampled_s,
                    lookahead_s=lookahead_s)
