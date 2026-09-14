"""Opt-in arc runtime contracts; these host tests do not validate a gait."""
import ctypes
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from drive_controller import NAMES, step
from servo import SharedGaitPolicy

F=ctypes.c_float
FP=ctypes.POINTER(F)
def array(values):return (F*len(values))(*values)


@pytest.fixture(scope='module')
def policy():return SharedGaitPolicy()


def robot(policy,linear=0.,yaw=-1.,parameters=None):
    return SimpleNamespace(plant=SimpleNamespace(policy=policy,p=parameters or {},data=SimpleNamespace(time=5.)),
        phase=.27,linear=linear,yaw=yaw,elapsed=3.,heading=SimpleNamespace(state=np.zeros(6),enabled=False),
        balance=SimpleNamespace(enabled=True),turn_assist=0.,tracking=SimpleNamespace(reset=lambda now:None),
        tracking_enabled=False,stopping_reason=None,imu_reading=None,safety='ok',request=(linear,yaw),profile='arcturn')


@pytest.mark.parametrize('linear,yaw',[(0,-1),(0,1),(.1,.2),(.7,0),(-.6,-.3),(0,0)])
def test_default_configured_path_preserves_baseline_targets_and_phase(policy,linear,yaw):
    baseline=robot(policy,linear,yaw)
    configured=robot(policy,linear,yaw,{'arc_trial':[.02,.5,.04,0]})
    for _ in range(12):
        np.testing.assert_array_equal(step(baseline),step(configured))
        assert baseline.phase==configured.phase
        assert baseline.linear==configured.linear and baseline.yaw==configured.yaw


@pytest.mark.parametrize('parameters',[{}, {'arc_trial':[.021,.5,.04,0]}, {'arc_trial':[.021,.5,.04,0,1.2,.2]}])
@pytest.mark.parametrize('linear,yaw',[(0,-1),(0,.1),(.2,.1),(0,0)])
def test_metadata_period_matches_actual_phase_progression(policy,parameters,linear,yaw):
    value=robot(policy,linear,yaw,parameters);previous=value.phase
    step(value)
    frame=value.arc_frame
    assert abs(((value.phase-previous)%1)*frame['period_s']-.02)<1e-7
    expected=(previous+.04/frame['lead_period_s'])%1
    assert abs(frame['target_phase']-expected)<1e-8
    assert abs(frame['motion_scale']-frame['scale']*min(1,abs(value.linear)+abs(value.yaw)))<1e-7


@pytest.mark.parametrize('options,tripod',[
    ([.02,.5,.04],False),([.02,.5,.04,0,1.2],False),([.02,.5,.04,0],True),
    (None,True),([.02,.5,float('nan'),0],False),([.01,.5,.04,0],False),
    ([.02,.9,.04,0,1.2,.3],True),([.02,.5,.04,0,.1,.2],False)])
def test_bad_configuration_is_rejected_before_publishing_state(policy,options,tripod):
    value=robot(policy,parameters={'arc_trial':options,'arc_tripod_trial':tripod})
    with pytest.raises(ValueError):step(value)
    assert value.phase==.27 and value.elapsed==3.
    assert not hasattr(value,'arc_frame')


def test_zero_input_removes_offline_wave_at_stop(policy):
    parameters={'arc_wave_trial':[.003,-.002,.002,0,.001,0]}
    with_wave=robot(policy,0,0,parameters);without_wave=robot(policy,0,0)
    np.testing.assert_array_equal(step(with_wave),step(without_wave))
    assert with_wave.arc_frame['motion_scale']==0


@pytest.mark.parametrize('sensor_available,balance_enabled',[(False,True),(True,False)])
def test_com_feedback_does_not_use_stale_observer_or_disabled_balance(policy,sensor_available,balance_enabled):
    value=robot(policy,parameters={'arc_com_trial':[.12,0.,.2,0.]})
    value.imu_reading={'age_ms':0,'yaw_tenths':0}
    value.attitude_filter=SimpleNamespace(filtered=[0,0],rate=[0,0],failures=0)
    value.arc_sensor=SimpleNamespace(filtered=[30,-20],rate=[100,50])
    value.arc_sensor_available=sensor_available;value.balance.enabled=balance_enabled
    reference=robot(policy)
    np.testing.assert_array_equal(step(value),step(reference))
    np.testing.assert_array_equal(value.arc_com_state,[0,0])


def test_available_enabled_com_feedback_is_not_accidentally_disabled(policy):
    value=robot(policy,parameters={'arc_com_trial':[.12,0.,.2,0.]})
    value.imu_reading={'age_ms':0,'yaw_tenths':0}
    value.attitude_filter=SimpleNamespace(filtered=[0,0],rate=[0,0],failures=0)
    value.arc_sensor=SimpleNamespace(filtered=[30,-20],rate=[100,50])
    value.arc_sensor_available=True
    reference=robot(policy)
    assert np.max(abs(step(value)-step(reference)))>.001


@pytest.fixture(scope='module')
def foot_geometry(tmp_path_factory):
    directory=tmp_path_factory.mktemp('arc-wave-geometry')
    source=directory/'foot.c';source.write_text('#include "arc_turn.h"\nvoid feet(const float*q,float*out){for(int i=0;i<4;i++)arc_foot(i,q+3*i,out+3*i,0);}\n')
    output=directory/'foot.dylib';root=Path(__file__).resolve().parents[2]
    subprocess.run(['clang','-shared','-fPIC','-O2','-I'+str(root/'firmware/stm32-learning/Inc'),str(source),'-o',str(output)],check=True)
    lib=ctypes.CDLL(str(output));lib.feet.argtypes=(FP,FP)
    def feet(q):
        result=array([0.]*12);lib.feet(array(q),result)
        return np.array(result).reshape(4,3)
    return feet


def wave(policy,nominal,phase,scale,params):
    fn=policy._library.spot_arc_wave;fn.argtypes=(FP,F,F,FP,FP);fn.restype=ctypes.c_int
    output=array([123.]*12)
    ok=fn(array(nominal),phase,scale,array(params),output)
    return ok,np.array(output)


def test_full_activity_wave_matches_direct_kernel_without_new_amplitude_change(policy):
    params=[.001,-.002,.003,.001,.002,-.001]
    value=robot(policy,parameters={'arc_wave_trial':params});reference=robot(policy)
    nominal=step(reference)
    expected_ok,expected=wave(policy,nominal,reference.arc_frame['target_phase'],reference.arc_frame['scale'],params)
    assert expected_ok
    np.testing.assert_array_equal(step(value),expected)


def test_wave_uses_x_second_harmonic_and_y_first_harmonic(policy,foot_geometry):
    nominal=step(robot(policy,0,0));original=foot_geometry(nominal)
    params=[.001,-.002,.003,.001,.002,-.001]
    for phase in np.linspace(0,1,33):
        ok,result=wave(policy,nominal,phase,1,params);assert ok
        angle=2*np.pi*phase
        shift=np.array([params[0]+params[2]*np.cos(2*angle)+params[3]*np.sin(2*angle),
                        params[1]+params[4]*np.cos(angle)+params[5]*np.sin(angle)])
        change=foot_geometry(result)-original
        np.testing.assert_allclose(change[:,:2],np.broadcast_to(-shift,(4,2)),atol=.00015)
        np.testing.assert_allclose(change[:,2],0,atol=.00015)
        assert np.linalg.norm(shift)<=.01


@pytest.mark.parametrize('params',[[.009,.009,0,0,0,0],[.011,0,0,0,0,0],[float('nan'),0,0,0,0,0]])
def test_wave_rejects_nonfinite_or_ten_mm_excess_atomically(policy,params):
    nominal=step(robot(policy,0,0))
    ok,output=wave(policy,nominal,.25,1,params)
    assert not ok
    np.testing.assert_array_equal(output,[123.]*12)


def test_wave_ten_mm_boundary_is_accepted_without_clipping(policy,foot_geometry):
    nominal=step(robot(policy,0,0));original=foot_geometry(nominal)
    ok,result=wave(policy,nominal,0,1,[.006,.008,0,0,0,0]);assert ok
    np.testing.assert_allclose(foot_geometry(result)[:,:2]-original[:,:2],
                               np.broadcast_to([-.006,-.008],(4,2)),atol=.00015)


@pytest.mark.parametrize('parameters',[
    {'arc_dynamic_trial':[1,.05,.5]},
    {'arc_dynamic_trial':[1,.05,.5,.12],'arc_wave_trial':[.007,0,0,0,0,0]},
    {'arc_dynamic_trial':[1,.05,.5,.12],'arc_com_trial':[.12,0,.2,0]},
])
def test_dynamic_body_envelope_is_rejected_before_advancing_controller(policy,parameters):
    value=robot(policy,parameters=parameters)
    with pytest.raises(ValueError):step(value)
    assert value.phase==.27 and value.elapsed==3.
    assert not hasattr(value,'arc_frame')


def test_dynamic_does_not_move_body_with_zero_input_and_zero_state(policy):
    value=robot(policy,0,0,{'arc_dynamic_trial':[1,.05,.5,.12]})
    value.attitude_filter=SimpleNamespace(filtered=[30,-20],rate=[100,50],failures=0)
    value.imu_reading={'age_ms':0,'yaw_tenths':0}
    np.testing.assert_array_equal(step(value),step(robot(policy,0,0)))
    np.testing.assert_array_equal(value.arc_dynamic.state,[0,0,0,0])


def test_cartesian_swing_shaping_composes_with_wave_and_preserves_zero_input(policy):
    parameters={'arc_swing_shape_trial':[.003,.003,.001,-.003],
                'arc_wave_trial':[.001,0,.001,0,.001,0]}
    zero=robot(policy,0,0,parameters)
    np.testing.assert_array_equal(step(zero),step(robot(policy,0,0)))
    moving=robot(policy,parameters=parameters)
    assert np.isfinite(step(moving)).all()


@pytest.mark.parametrize('offsets',[[.003]*3,[.011,0,0,0],[float('nan'),0,0,0]])
def test_bad_swing_shape_rejects_before_frame_publication(policy,offsets):
    value=robot(policy,parameters={'arc_swing_shape_trial':offsets})
    with pytest.raises(ValueError):step(value)
    assert value.phase==.27 and not hasattr(value,'arc_frame')
