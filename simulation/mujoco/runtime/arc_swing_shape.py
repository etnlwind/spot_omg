"""Optional shared-C per-leg Cartesian swing arch, before body-wave shaping."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes
import hashlib
from pathlib import Path
from servo.host_build import build_shared, library_suffix
import tempfile

import numpy as np


_LIBRARY=None


def _library():
    global _LIBRARY
    if _LIBRARY is None:
        inc=REPO_ROOT/'firmware/stm32-learning/Inc'
        source='''#include "arc_swing_shape.h"
int shape_apply(const float values[12],float phase,float duty,float scale,const float offsets[4],const float delta[4],float out[12]){
 GaitPolicyLegTarget nominal[4],result[4];
 for(int i=0;i<4;i++)nominal[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
 if(!arc_swing_shape_apply(nominal,phase,duty,scale,offsets,delta,result))return 0;
 for(int i=0;i<4;i++){out[3*i]=result[i].j1_deg;out[3*i+1]=result[i].j2_deg;out[3*i+2]=result[i].j3_deg;}return 1;
}
void shape_feet(const float values[12],float out[12]){for(int i=0;i<4;i++)arc_foot(i,values+3*i,out+3*i,0);}
float shape_envelope(float phase,float duty){return arc_swing_shape_envelope(phase,duty);}
int shape_test_nominal(float phase,float out[12]){
 GaitPolicyLegTarget q[4];if(!arc_turn_targets_configured(phase,1,0,-1,.022,.5,0,0,q))return 0;
 for(int i=0;i<4;i++){out[3*i]=q[i].j1_deg;out[3*i+1]=q[i].j2_deg;out[3*i+2]=q[i].j3_deg;}return 1;
}
'''
        digest=hashlib.sha256(source.encode())
        for name in ('arc_swing_shape.h','arc_turn.h','arc_geometry.h','gait_policy.h'):
            digest.update((inc/name).read_bytes())
        cache=Path(tempfile.gettempdir())/'spot-arc-swing-shape';cache.mkdir(exist_ok=True)
        library=cache/(digest.hexdigest()[:20]+library_suffix())
        if not library.exists():
            with tempfile.TemporaryDirectory(dir=cache) as directory:
                cfile=Path(directory)/'shape.c';cfile.write_text(source)
                build_shared([cfile],inc,library,extra=['-ffp-contract=off'])
        lib=ctypes.CDLL(str(library));f=ctypes.c_float;fp=ctypes.POINTER(f)
        lib.shape_apply.argtypes=(fp,f,f,f,fp,fp,fp);lib.shape_apply.restype=ctypes.c_int
        lib.shape_feet.argtypes=(fp,fp)
        lib.shape_envelope.argtypes=(f,f);lib.shape_envelope.restype=f
        lib.shape_test_nominal.argtypes=(f,fp);lib.shape_test_nominal.restype=ctypes.c_int
        _LIBRARY=lib
    return _LIBRARY


class ArcSwingShape:
    def __init__(self):
        self.lib=_library()

    def apply(self,target,frame,delta4):
        target=np.asarray(target,dtype=float);delta=np.asarray(delta4,dtype=float)
        offsets=np.asarray(frame.get('offsets',[0,.5,.5,0]),dtype=float)
        phase=float(frame['target_phase']);duty=float(frame['duty'])
        scale=float(frame.get('motion_scale',frame.get('scale',1.)))
        if target.shape!=(12,) or delta.shape!=(4,) or offsets.shape!=(4,):
            raise ValueError('Swing shape requires twelve joint angles and four leg deltas/offsets')
        if not np.isfinite(target).all() or not np.isfinite(delta).all() or np.any(abs(delta)>.01) or not np.isfinite(offsets).all():
            raise ValueError('Swing shape inputs must be finite and deltas within ±10mm')
        if (not np.isfinite([phase,duty,scale]).all() or not .5<=duty<=.85 or not 0<=scale<=1 or
            np.any(offsets<0) or np.any(offsets>=1) or
            np.any(target<np.tile([-30.,-45.,0.],4)) or np.any(target>np.tile([30.,100.,150.],4))):
            raise ValueError('Swing shape phase, amplitude, or joint range is invalid')
        f=ctypes.c_float;out=(f*12)()
        if not self.lib.shape_apply((f*12)(*target),phase,duty,scale,(f*4)(*offsets),(f*4)(*delta),out):
            raise ValueError('Cartesian swing shape is invalid or outside CAD joint reach')
        result=np.array(out,dtype=float)
        # Preserve untouched caller values exactly, including float64 inputs.
        for leg in range(4):
            if scale==0 or delta[leg]==0 or (phase+offsets[leg])%1<duty:
                result[3*leg:3*leg+3]=target[3*leg:3*leg+3]
        return result
