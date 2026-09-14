
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import ctypes
import pytest
from servo import SharedGaitPolicy
from simulation.mujoco.runtime.gait_tracking import GaitTracking

def feed(t,now,error):
    for i in range(12):t.lib.spot_tracking_sample(t.state,i,error,now)

def test_normal_feedback_keeps_full_rate_and_stop_never_waits():
    t=GaitTracking(SharedGaitPolicy())
    for now in range(0,1000,20):
        feed(t,now,2);assert t.step(now/1000,.02,False)==1
    feed(t,1000,14);assert t.step(1,.02,False)==0
    assert t.step(1.02,.02,True)==1

def test_partial_lag_slows_common_phase_and_recovers_smoothly():
    t=GaitTracking(SharedGaitPolicy())
    for now in range(0,1000,20):feed(t,now,10);t.step(now/1000,.02,False)
    assert .45<t.rate<.55
    previous=t.rate
    feed(t,1000,0);t.step(1,.02,False)
    assert 0<t.rate-previous<=.01001

def test_stale_missing_and_persistent_error_fault():
    t=GaitTracking(SharedGaitPolicy())
    assert t.step(.38,.02,False)==0
    t.step(.62,.02,False);assert t.diagnostic['fault']==1
    t.reset(1)
    for now in range(1000,1620,20):feed(t,now,14);t.step(now/1000,.02,False)
    assert t.diagnostic['fault']==2

def test_round_robin_age_and_timestamp_wrap():
    t=GaitTracking(SharedGaitPolicy());start=2**32-100;t.lib.spot_tracking_reset(t.state,start)
    for n in range(100):
        now=(start+n*20)%2**32
        t.lib.spot_tracking_sample(t.state,n%12,1,now)
        diag=(ctypes.c_float*4)()
        assert t.lib.spot_tracking_step(t.state,now,.02,0,diag)==1
        assert diag[1]<=240 and diag[2]==0

def test_frozen_progress_does_not_follow_new_direction_but_stop_can_slew():
    from simulation.mujoco.runtime.drive_controller import NAMES
    lib=SharedGaitPolicy()._library;fn=lib.spot_drive_step_timed
    fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.c_int,*([ctypes.c_float]*3),*([ctypes.c_int]*3),ctypes.c_float,ctypes.c_float,fp)
    fn.restype=ctypes.c_int
    state=(ctypes.c_float*11)();out=(ctypes.c_float*12)();idx=NAMES.index('arcturn')
    for _ in range(80):assert fn(state,idx,0,-1,0,1,1,0,.02,1,out)
    phase=state[0];yaw=state[2]
    assert fn(state,idx,1,1,0,1,1,0,.02,0,out)
    held=list(out)
    for _ in range(5):
        assert fn(state,idx,1,1,0,1,1,0,.02,0,out)
        assert list(out)==held and state[0]==phase and state[2]==yaw
    assert fn(state,idx,0,0,0,1,1,1,.02,1,out)
    assert abs(state[2])<abs(yaw)
