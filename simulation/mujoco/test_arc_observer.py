import ctypes
import math

import pytest

from arc_observer import ArcObserver


def sample(time, roll=0., pitch=0.):
    return dict(sample_time_s=time,roll_tenths=roll*10,pitch_tenths=pitch*10)


def test_fractional_filter_keeps_small_dc_angles_without_integer_deadband():
    observer = ArcObserver()
    assert observer.update(sample(0),0)
    for i in range(1,61):
        assert observer.update(sample(i*.02,.2,-.3),i*.02)
    out=observer.read(1.2)
    assert out['angle_deg']==pytest.approx([.2,-.3],abs=1e-6)
    assert out['rate_deg_s']==pytest.approx([0,0],abs=1e-5)


def test_time_based_alphas_match_20ms_and_repeated_sample_is_not_refiltered():
    observer=ArcObserver();observer.update(sample(0),0)
    observer.update(sample(.02,1),.02)
    assert observer.read(.02)['angle_deg'][0]==pytest.approx(.6,abs=1e-6)
    assert observer.read(.02)['rate_deg_s'][0]==pytest.approx(12.5,abs=1e-5)
    observer.update(sample(.02,1),.04)
    assert observer.read(.04)['angle_deg'][0]==pytest.approx(.6,abs=1e-6)
    assert observer.read(.04)['rate_deg_s'][0]==pytest.approx(12.5,abs=1e-5)


def test_prediction_is_explicit_and_rate_is_bounded():
    observer=ArcObserver();observer.update(sample(0),0)
    observer.update(sample(.02,10),.04)
    ordinary=observer.read(.04)
    predicted=observer.read(.04,.1)
    assert ordinary['rate_deg_s'][0]==pytest.approx(30.,abs=1e-4)
    assert ordinary['angle_deg'][0]==pytest.approx(6.,abs=1e-5)
    assert predicted['angle_deg'][0]==pytest.approx(9.,abs=1e-5)
    assert ordinary['age_s']==pytest.approx(.02)
    for invalid in (-.01,.10001,float('nan')):
        with pytest.raises(ValueError):observer.read(.04,invalid)


@pytest.mark.parametrize('bad', [None,dict(sample_time_s=.02,roll_tenths=float('nan'),pitch_tenths=0),
                                dict(sample_time_s=1,roll_tenths=0,pitch_tenths=0),{}])
def test_missing_or_bad_sample_clears_availability_decays_and_recovers(bad):
    observer=ArcObserver();observer.update(sample(0,1,-1),0)
    before=observer.state.angle_deg[0]
    assert not observer.update(bad,.02)
    assert observer.read(.02) is None
    assert 0<observer.state.angle_deg[0]<before
    # A fresh reading, rather than re-polling the same sample, restores output.
    assert observer.update(sample(.04,.5,-.5),.04)
    assert observer.read(.04) is not None


def test_stale_backwards_and_nonfinite_clock_never_return_healthy_estimate():
    observer=ArcObserver();observer.update(sample(0,1),0)
    assert observer.read(.101) is None
    assert not observer.update(sample(0,1),.101)
    assert observer.update(sample(.12,.3),.12)
    assert not observer.update(sample(.1,.3),.13)
    assert observer.read(.13) is None
    assert observer.update(sample(.14,.3),.14)
    assert not observer.update(sample(.16,.3),float('nan'))
    assert observer.read(.16) is None


def test_long_dropout_restart_has_no_invented_rate_and_failed_read_is_atomic():
    observer=ArcObserver();observer.update(sample(0,10),0)
    observer.update(None,.3)
    out=(ctypes.c_float*4)(123,123,123,123)
    assert not observer.lib.observer_read(ctypes.byref(observer.state),.3,0,out)
    assert list(out)==[123]*4
    observer.update(sample(.32,-.2,.2),.32)
    result=observer.read(.32)
    assert result['angle_deg']==pytest.approx([-.2,.2],abs=1e-6)
    assert result['rate_deg_s']==[0,0]
