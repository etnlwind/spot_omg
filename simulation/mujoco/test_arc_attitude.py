"""Geometry and failure semantics for the opt-in CAD attitude controller.

These are host checks, not evidence that a physical gait is stable.
"""
import ctypes
from pathlib import Path
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
FP = ctypes.POINTER(ctypes.c_float)


def floats(values):
    return (ctypes.c_float * len(values))(*values)


@pytest.fixture(scope='module')
def lib(tmp_path_factory):
    output = tmp_path_factory.mktemp('arc-attitude') / 'attitude.dylib'
    subprocess.run(['clang', '-O2', '-shared', '-fPIC',
                    '-I'+str(ROOT/'firmware/stm32-learning/Inc'),
                    str(Path(__file__).with_name('arc_attitude_host.c')), '-o', str(output)], check=True)
    value = ctypes.CDLL(str(output))
    value.spot_arc_attitude.argtypes = (FP, FP, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                       FP, ctypes.c_int, ctypes.c_float, FP, FP)
    value.spot_arc_attitude.restype = ctypes.c_int
    value.spot_arc_attitude_foot.argtypes = (FP, FP)
    value.spot_arc_attitude_targets.argtypes = (ctypes.c_float, ctypes.c_float, FP)
    value.spot_arc_attitude_targets.restype = ctypes.c_int
    return value


def target(lib, phase, yaw=-1.):
    values = floats([0.]*12)
    assert lib.spot_arc_attitude_targets(phase, yaw, values)
    return values


def feet(lib, values):
    output = floats([0.]*12)
    lib.spot_arc_attitude_foot(values, output)
    return np.array(output).reshape(4, 3)


def apply(lib, state, nominal, phase, imu, available=True, config=None, dt=.02, period=1.2, duty=.5):
    config = [.25, .75, 0., .004, .03, .08] if config is None else config
    output = floats([123.]*12)
    ok = lib.spot_arc_attitude(state, nominal, phase, period, duty, floats(imu),
                               available, dt, floats(config), output)
    return ok, output


@pytest.mark.parametrize('phase', [.25, .75])
@pytest.mark.parametrize('axis', [0, 1])
@pytest.mark.parametrize('sign', [-1, 1])
def test_stance_and_swing_signs_from_cad_geometry(lib, phase, axis, sign):
    nominal = target(lib, phase)
    original = feet(lib, nominal)
    imu = [0., 0., 0., 0.]
    imu[axis] = sign*np.radians(2.)
    state = floats([0.]*4)
    for _ in range(12):
        ok, result = apply(lib, state, nominal, phase, imu)
        assert ok
    moved = feet(lib, result)
    np.testing.assert_allclose(moved[:, :2], original[:, :2], atol=.00015)
    # Left = +Y, front = +X. Positive roll needs left support foot up;
    # positive pitch needs front support foot down. Swing has opposite sign.
    origin = np.array([-.007048938, -.000101855, .279936363])
    basic = (original[:, 1]-origin[1]) if axis == 0 else -(original[:, 0]-origin[0])
    support = (phase+np.array([0., .5, .5, 0.])) % 1 < .5
    expected_sign = np.sign(basic)*sign*np.where(support, 1, -1)
    assert np.all((moved[:, 2]-original[:, 2])*expected_sign > .0001)
    np.testing.assert_allclose(moved[:, 2]-original[:, 2], state, atol=.00015)


def test_zero_attitude_does_not_change_nominal(lib):
    for phase in np.linspace(0, 1, 30, endpoint=False):
        nominal = target(lib, phase)
        ok, result = apply(lib, floats([0.]*4), nominal, phase, [0.]*4)
        assert ok
        assert list(result) == list(nominal)


def test_sensor_missing_decays_and_limits_are_in_metres_per_second(lib):
    phase = .25
    nominal = target(lib, phase)
    state = floats([0.]*4)
    for _ in range(30):
        before = np.array(state)
        ok, result = apply(lib, state, nominal, phase, [.12, -.12, 0., 0.])
        assert ok
        assert np.max(abs(np.array(state)-before)) <= .03*.02+1e-8
        assert np.max(abs(np.array(state))) <= .004+1e-8
    for _ in range(10):
        before = np.array(state)
        ok, result = apply(lib, state, nominal, phase, [float('nan')]*4, available=False)
        assert ok
        assert np.all(abs(np.array(state)) <= abs(before)+1e-8)
    np.testing.assert_allclose(state, 0, atol=1e-8)
    np.testing.assert_allclose(result, nominal, atol=1e-6)


def test_role_boundaries_and_variable_control_intervals_are_continuous(lib):
    for boundary in (.5, 1.):
        samples=[]
        for phase in (boundary-.0001, boundary, boundary+.0001):
            nominal=target(lib, phase)
            state=floats([0.]*4)
            ok, _=apply(lib, state, nominal, phase, [.04, -.04, 0., 0.])
            assert ok
            samples.append(np.array(state))
        assert np.max(np.abs(samples)) < 1e-7
    state=floats([0.]*4)
    phase=.49
    for dt in (.01, .04, .02, .06, .01):
        phase+=dt/1.2
        before=np.array(state)
        ok, _=apply(lib, state, target(lib,phase), phase, [.04, -.04, 0., 0.],dt=dt)
        assert ok
        assert np.max(abs(np.array(state)-before)) <= .03*dt+1e-8


def test_rejection_is_atomic_for_bad_sensor_config_or_targets(lib):
    nominal=target(lib,.25)
    invalid=np.array(nominal);invalid[11]=200.
    cases=[dict(imu=[float('nan'),0,0,0]),dict(period=0),dict(dt=.1),dict(duty=.4),
           dict(config=[.25,.75,0,.02,.03,.08]),dict(nominal=floats(invalid))]
    for changes in cases:
        state=floats([.001]*4);before=bytes(state)
        args=dict(nominal=nominal,phase=.25,imu=[.02,0,0,0]);args.update(changes)
        ok, result=apply(lib,state,**args)
        assert not ok
        assert bytes(state)==before
        assert list(result)==[123.]*12
