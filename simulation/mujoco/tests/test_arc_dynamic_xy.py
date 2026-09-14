"""Opt-in XY integration contracts; no claim of physical gait stability."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import ctypes
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.mujoco.runtime.arc_dynamic_balance import ArcDynamicBalance
from servo import SharedGaitPolicy


@pytest.fixture(scope='module')
def nominal():
    lib=SharedGaitPolicy()._library;f=ctypes.c_float;fp=ctypes.POINTER(f)
    fn=lib.spot_arc_configured;fn.argtypes=(f,f,f,f,fp,fp);fn.restype=ctypes.c_int
    out=(f*12)();assert fn(0,0,0,0,(f*4)(.02,.5,.04,0),out)
    return np.array(out)


def geometry(control,target):
    f=ctypes.c_float;center=(f*12)();contact=(f*12)()
    control.lib.arc_dynamic_xy_geometry((f*12)(*target),center,contact)
    return np.array(center).reshape(4,3),np.array(contact).reshape(4,3)


def sample(roll=0,pitch=0,roll_rate=0,pitch_rate=0):
    # Public adapter consumes tenths of degrees.
    return SimpleNamespace(filtered=np.degrees([roll,pitch])*10,
                           rate=np.degrees([roll_rate,pitch_rate])*10)


def frame(phase):return dict(target_phase=phase,period_s=1.2)


def test_zero_gain_preserves_nominal_exactly(nominal):
    control=ArcDynamicBalance()
    output=control.apply(nominal,frame(.2),sample(.03,-.02,.2,-.1),True,[0,.05,.5,.12])
    np.testing.assert_array_equal(output,nominal)
    np.testing.assert_array_equal(control.state,[0,0,0,0])


def test_acceleration_blends_before_single_xy_integration_and_keeps_geometry(nominal):
    control=ArcDynamicBalance();original,_=geometry(control,nominal);dt=.02
    for step in range(400):
        previous=np.array(control.state)
        output=control.apply(nominal,frame(step*dt/1.2),sample(.04*np.sin(step*.071),.03*np.cos(step*.053),
            .142*np.cos(step*.071),-.0795*np.sin(step*.053)),True,[1,.05,.5,.12],dt=dt)
        state=np.array(control.state);acceleration=np.array(control.diagnostic['acceleration_m_s2'])
        assert np.linalg.norm(state[:2])<.004
        assert np.linalg.norm(state[2:])<=.05+1e-7
        assert np.linalg.norm(acceleration)<=.5+1e-7
        np.testing.assert_allclose(state[:2],previous[:2]+dt*previous[2:]+.5*dt*dt*acceleration,atol=3e-9)
        np.testing.assert_allclose(state[2:],previous[2:]+dt*acceleration,atol=1e-8)
        actual,_=geometry(control,output)
        np.testing.assert_allclose(actual[:,:2]-original[:,:2],np.broadcast_to(-state[:2],(4,2)),atol=.00015)
        np.testing.assert_allclose(actual[:,2],original[:,2],atol=.00015)


def test_projection_has_expected_nonminimum_phase_acceleration_sign(nominal):
    control=ArcDynamicBalance();_,feet=geometry(control,nominal)
    t=feet[3,:2]-feet[0,:2];t/=np.linalg.norm(t);n=np.array([-t[1],t[0]])
    # Quarter phase is clear of switching; only FL/RR free-axis is active.
    control.apply(nominal,frame(.25),sample(*(t*.001)),True,[1,.05,.5,.12])
    requested=np.array(control.diagnostic['requested_acceleration_m_s2'])
    assert np.dot(requested,n)>0
    assert abs(np.dot(requested,t))<1e-6


@pytest.mark.parametrize('available,gain',[(False,1),(True,0)])
def test_missing_imu_or_zero_gain_returns_command_state_without_using_stale_angles(nominal,available,gain):
    control=ArcDynamicBalance()
    for _ in range(8):control.apply(nominal,frame(.25),sample(.02,.01),True,[1,.05,.5,.12])
    assert np.linalg.norm(control.state)>1e-6
    for _ in range(300):
        control.apply(nominal,frame(.25),sample(float('nan'),float('nan')),available,[gain,.05,.5,.12])
    assert np.linalg.norm(control.state)<1e-6
    control.reset();assert control.diagnostic=={}
    np.testing.assert_array_equal(control.state,[0,0,0,0])


def test_invalid_inputs_do_not_partially_advance_xy_state(nominal):
    control=ArcDynamicBalance();control.apply(nominal,frame(.25),sample(.002,.001),True,[1,.05,.5,.12])
    before=bytes(control.state)
    for config in ([1,.11,.5,.12],[1,.05,2,.12],[1,.05,.5,float('nan')]):
        with pytest.raises(ValueError):control.apply(nominal,frame(.25),sample(.01,.01),True,config)
        assert bytes(control.state)==before
    invalid=nominal.copy();invalid[-1]=200
    with pytest.raises(ValueError):control.apply(invalid,frame(.25),sample(),True,[1,.05,.5,.12])
    assert bytes(control.state)==before
