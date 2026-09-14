"""Experimental lift-first curve: boundary, CAD and unchanged stance checks."""
import ctypes
import numpy as np
import pytest
from servo import SharedGaitPolicy
from drive_controller import step
from test_arc_trial_contracts import robot

@pytest.mark.parametrize('fraction',[.05,.1,.2,.3])
def test_curve_matches_stance_velocity_and_is_c2(fraction):
    p=SharedGaitPolicy(); f=p._library.spot_arc_lift_first_x
    f.argtypes=(ctypes.c_float,)*3;f.restype=ctypes.c_float
    value=lambda u:f(u,.5,fraction)
    assert value(0)==pytest.approx(-.5)
    assert value(1)==pytest.approx(.5)
    assert value(.5)==pytest.approx(0,abs=1e-6)
    h=.002
    for t in [0,fraction,1-fraction,1]:
        if t==0: derivative=(value(h)-value(0))/h
        elif t==1:derivative=(value(1)-value(1-h))/h
        else:
            left=(value(t)-value(t-h))/h
            right=(value(t+h)-value(t))/h
            assert abs(left-right)<.02
            continue
        assert derivative==pytest.approx(-1,abs=.005)
    for t in np.linspace(0,1,101):
        assert value(t)==pytest.approx(-value(1-t),abs=2e-6)

@pytest.mark.parametrize('yaw',[-1,1])
def test_stance_and_vertical_task_retained(yaw):
    policy=SharedGaitPolicy();fp=ctypes.POINTER(ctypes.c_float)
    # Exported CAD foot function supplies the same lowest-cushion task as IK.
    fk=policy._library.spot_arc_contact_gravity
    fk.argtypes=(fp,)*5;fk.restype=None
    def heights(values):
        bias=(ctypes.c_float*12)();com=(ctypes.c_float*3)();feet=(ctypes.c_float*12)();jac=(ctypes.c_float*12)()
        fk((ctypes.c_float*12)(*values.ravel()),bias,com,feet,jac)
        return np.array(feet).reshape(4,3)[:,2]
    params={'arc_trial':[.022,.5,.04,0]}
    for phase in np.linspace(0,1,31,endpoint=False):
        a=robot(policy,phase,0,params);a.request=(0,yaw);a.yaw=yaw;a.elapsed=2
        b=robot(policy,phase,0,{**params,'arc_lift_first_trial':.1});b.request=a.request;b.yaw=yaw;b.elapsed=2
        before=step(a).reshape(4,3);after=step(b).reshape(4,3)
        for i in range(4):
            p=(a.arc_frame['target_phase']+([0,.5,.5,0][i]))%1
            if p<.5:np.testing.assert_array_equal(before[i],after[i])
            else:
                assert heights(after)[i]==pytest.approx(heights(before)[i],abs=.00011)

@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-.1,0,.31])
def test_invalid_fraction_does_not_modify_output(bad):
    policy=SharedGaitPolicy();fp=ctypes.POINTER(ctypes.c_float)
    f=policy._library.spot_arc_lift_first
    f.argtypes=(fp,*([ctypes.c_float]*7),fp);f.restype=ctypes.c_int
    values=(ctypes.c_float*12)(*([.75,50,92]*4));out=(ctypes.c_float*12)(*([123]*12))
    assert not f(values,.7,1,0,1,.5,.2,bad,out)
    assert list(out)==[123]*12

@pytest.mark.parametrize('yaw',[-1,1])
def test_stop_returns_to_neutral_without_residual_transfer(yaw):
    policy=SharedGaitPolicy()
    r=robot(policy,0,0,{'arc_trial':[.022,.5,.04,0],'arc_lift_first_trial':.1})
    r.request=(0,yaw);r.phase=r.elapsed=0
    for n in range(350):
        if n==200:r.request=(0,0);r.stopping_reason='test'
        values=step(r)
        assert np.isfinite(values).all()
    assert abs(r.linear)+abs(r.yaw)<1e-6
    reference=robot(policy,r.phase,0,{'arc_trial':[.022,.5,.04,0]})
    reference.elapsed=r.elapsed;reference.request=(0,0);reference.linear=reference.yaw=0
    np.testing.assert_allclose(values,step(reference),atol=.001)
