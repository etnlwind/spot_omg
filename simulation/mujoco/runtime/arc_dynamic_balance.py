"""Isolated opt-in CAD/LQR binding; caller supplies delayed IMU only."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes
import hashlib
from pathlib import Path
import subprocess
import tempfile

import numpy as np


def library():
    source=(SIM_ROOT / 'runtime/arc_dynamic_host.c')
    include=REPO_ROOT/'firmware/stm32-learning/Inc'
    names=('arc_dynamic_balance.h','arc_turn.h','arc_geometry.h','gait_policy.h')
    digest=hashlib.sha256(source.read_bytes()+b''.join((include/name).read_bytes() for name in names)).hexdigest()[:24]
    cache=Path(tempfile.gettempdir())/'spot-arc-dynamic';cache.mkdir(exist_ok=True)
    output=cache/(digest+'.dylib')
    if not output.exists():
        with tempfile.TemporaryDirectory(dir=cache) as directory:
            temporary=Path(directory)/'dynamic.dylib'
            subprocess.run(['clang','-O2','-shared','-fPIC','-I'+str(include),str(source),'-o',str(temporary)],
                           check=True,capture_output=True)
            temporary.replace(output)
    lib=ctypes.CDLL(str(output));f=ctypes.c_float;fp=ctypes.POINTER(f)
    lib.arc_dynamic_xy.argtypes=(fp,fp,f,f,fp,ctypes.c_int,f,fp,fp,fp)
    lib.arc_dynamic_xy.restype=ctypes.c_int
    lib.arc_dynamic_xy_geometry.argtypes=(fp,fp,fp)
    return lib


class ArcDynamicBalance:
    def __init__(self):
        self.lib=library();self.reset()

    def reset(self):
        self.state=(ctypes.c_float*4)();self.diagnostic={}

    def apply(self,target,frame,attitude,available,config,dt=.02):
        """config=[gain_scale,maximum_speed_m_s,maximum_accel_m_s2,blend_s].

        Frame uses target_phase and period_s; attitude is the established
        filtered/rate tenths-of-degrees interface. State describes commands,
        not measured or reconstructed physical body translation.
        """
        f=ctypes.c_float
        values=np.asarray(target,dtype=float);parameters=np.asarray(config,dtype=float)
        if values.shape!=(12,) or parameters.shape!=(4,):
            raise ValueError('Dynamic balance needs twelve targets and four configuration values')
        imu=np.radians(np.asarray([*attitude.filtered,*attitude.rate],dtype=float)/10)
        if imu.shape!=(4,):raise ValueError('Dynamic balance needs roll/pitch and their rates')
        output=(f*12)();diagnostic=(f*6)()
        if not self.lib.arc_dynamic_xy(self.state,(f*12)(*values),frame['target_phase'],frame['period_s'],
            (f*4)(*imu),bool(available),dt,(f*4)(*parameters),output,diagnostic):
            raise ValueError('Dynamic balance state, target, or bounded acceleration is infeasible')
        self.diagnostic=dict(delta_m=list(self.state[:2]),velocity_m_s=list(self.state[2:]),
            requested_acceleration_m_s2=list(diagnostic[:2]),acceleration_m_s2=list(diagnostic[2:4]),
            pair0_weight=float(diagnostic[4]),constrained=bool(diagnostic[5]),
            source='fixed-CAD-model-delayed-IMU-command-integrators')
        return np.array(output)
