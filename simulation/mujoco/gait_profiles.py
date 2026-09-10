"""Shared embedded foot-space locomotion families.

Different phase schedules and speed-dependent stride/cadence are evaluated on
unchanged CAD inertia, contact, voltage and servo limits before promotion.
"""
import json
import math
from pathlib import Path
import numpy as np

PROFILE_FILE = Path(__file__).resolve().parents[2]/'config'/'locomotion_profiles.json'
PHASES = {
    'trot': (0., .5, .5, 0.),
    'crawl': (0., .5, .75, .25),
    'amble': (0., .5, .75, .25),
    'pace': (0., .5, 0., .5),
    'bound': (0., 0., .5, .5),
}


def smooth(x):
    x = min(1., max(0., float(x)))
    return x*x*x*(10+x*(-15+6*x))


@__import__('functools').lru_cache(maxsize=1)
def _shared():
    import ctypes
    from servo import SharedGaitPolicy
    policy=SharedGaitPolicy()
    fn=policy._library.spot_foot_targets
    fn.argtypes=(ctypes.POINTER(ctypes.c_float),ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.POINTER(ctypes.c_float))
    fn.restype=ctypes.c_int
    return policy,fn


def foot_targets(params, t, scale=1., family='trot', linear=1., yaw=0.):
    """Execute the same C foot-space IK and pivot trajectory as STM32."""
    import ctypes
    if len(params)!=7 or not all(math.isfinite(v) for v in (*params,t,scale,linear,yaw)) or params[0]<=0:
        raise ValueError('Invalid gait input')
    families={'trot':0,'crawl':1,'amble':1,'pace':2,'bound':3}
    if family not in families:raise ValueError('Unknown gait family')
    values=(ctypes.c_float*12)()
    if not _shared()[1]((ctypes.c_float*7)(*params),(t/params[0])%1,scale,families[family],linear,yaw,values):
        raise ValueError('Invalid or unreachable gait target')
    return np.array(values,dtype=float)


def load_profiles():
    return json.loads(PROFILE_FILE.read_text())['profiles']
