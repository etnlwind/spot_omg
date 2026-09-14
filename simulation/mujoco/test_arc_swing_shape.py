import ctypes

import numpy as np
import pytest

from arc_swing_shape import ArcSwingShape


def nominal(adapter,phase):
    out=(ctypes.c_float*12)();assert adapter.lib.shape_test_nominal(phase,out)
    return np.array(out,dtype=float)


def feet(adapter,q):
    out=(ctypes.c_float*12)();adapter.lib.shape_feet((ctypes.c_float*12)(*q),out)
    return np.array(out).reshape(4,3)


def frame(phase,scale=1):
    return dict(target_phase=phase,duty=.5,motion_scale=scale,offsets=[0,.5,.5,0])


def test_zero_deltas_and_zero_motion_are_exact():
    adapter=ArcSwingShape();target=nominal(adapter,.75)+np.linspace(0,1e-9,12)
    np.testing.assert_array_equal(adapter.apply(target,frame(.75),[0]*4),target)
    np.testing.assert_array_equal(adapter.apply(target,frame(.75,0),[.01]*4),target)


@pytest.mark.parametrize('phase',[.0,.15,.25,.49,.5,.63,.75,.99])
def test_only_swing_z_changes_by_requested_c2_arch_and_stance_is_exact(phase):
    adapter=ArcSwingShape();target=nominal(adapter,phase);delta=np.array([.004,-.003,.007,.002])
    result=adapter.apply(target,frame(phase),delta);difference=feet(adapter,result)-feet(adapter,target)
    for leg,offset in enumerate((0,.5,.5,0)):
        local=(phase+offset)%1
        if local<.5:
            np.testing.assert_array_equal(result[leg*3:leg*3+3],target[leg*3:leg*3+3])
        expected=delta[leg]*adapter.lib.shape_envelope(local,.5)
        np.testing.assert_allclose(difference[leg],[0,0,expected],atol=1.1e-5)


def test_maximum_requested_delta_is_not_silently_reduced():
    adapter=ArcSwingShape();target=nominal(adapter,.75)
    result=adapter.apply(target,frame(.75),[.01,0,0,-.01]);dz=(feet(adapter,result)-feet(adapter,target))[:,2]
    assert dz[0]==pytest.approx(.01,abs=1.1e-5)
    assert dz[3]==pytest.approx(-.01,abs=1.1e-5)
    with pytest.raises(ValueError):adapter.apply(target,frame(.75),[.01001,0,0,0])


def test_c2_envelope_vanishes_with_zero_slope_and_curvature_at_boundaries():
    adapter=ArcSwingShape();eps=1e-4;f=adapter.lib.shape_envelope
    for boundary in (.5,1.):
        assert f(boundary,.5)==0
        first=(f(boundary+eps,.5)-f(boundary-eps,.5))/(2*eps)
        second=(f(boundary+eps,.5)-2*f(boundary,.5)+f(boundary-eps,.5))/eps**2
        assert abs(first)<1e-5
        assert abs(second)<.06


def test_invalid_inputs_reject_without_partial_c_output():
    adapter=ArcSwingShape();f=ctypes.c_float;target=nominal(adapter,.75)
    output=(f*12)(*([123]*12))
    assert not adapter.lib.shape_apply((f*12)(*target),.75,.5,1,(f*4)(0,.5,.5,0),(f*4)(.002,0,0,float('nan')),output)
    assert list(output)==[123]*12
    with pytest.raises(ValueError):adapter.apply(target,frame(.75),[float('nan')]*4)
