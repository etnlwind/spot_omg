import pytest
from heading_controller import HeadingController
from servo import SharedGaitPolicy


def sample(deg, age=20): return dict(yaw_tenths=round(deg*10), age_ms=age)


def locked(deg=0):
    h=HeadingController(SharedGaitPolicy())
    for _ in range(15): h.update(sample(deg),.8,0,0)
    assert h.diagnostic()['active']
    return h


@pytest.mark.parametrize('reference,current,sign', [(359,1,1),(1,359,-1),(30,25,-1),(30,35,1)])
def test_wrap_and_correction_direction(reference,current,sign):
    h=locked(reference)
    for _ in range(30): out=h.update(sample(current),.8,0,0)
    assert out*sign > 0
    assert abs(h.diagnostic()['error_deg']) < 6


@pytest.mark.parametrize('reason',['manual','reverse','stop','stale','offline','disabled','unsafe'])
def test_release_and_user_turn_clear_heading_reference(reason):
    h=locked();h.update(sample(5),.8,0,0)
    r=sample(5,101 if reason=='stale' else 20)
    if reason=='offline':r=None
    if reason=='disabled':h.enabled=False
    linear=-.8 if reason=='reverse' else 0 if reason=='stop' else .8
    out=h.update(r,linear,.001 if reason=='manual' else 0,0,reason!='unsafe')
    assert out==0 and not h.diagnostic()['active']
    h.enabled=True
    for _ in range(15):h.update(sample(90),.8,0,0)
    assert h.diagnostic()['reference_deg']==90


def test_saturation_slew_and_turn_settle():
    h=locked();old=0
    for _ in range(200):
        out=h.update(sample(60),.8,0,0)
        assert abs(out)<=.250001 and abs(out-old)<=.010001
        old=out
    h.update(sample(60),.8,.5,.5)
    for _ in range(20):h.update(sample(70),.8,0,.1)
    assert not h.diagnostic()['active']
    for _ in range(15):h.update(sample(75),.8,0,0)
    assert h.diagnostic()['reference_deg']==75


def test_sub_degree_error_is_not_ignored():
    h=locked()
    for _ in range(100):
        out=h.update(sample(.2),.8,0,0)
    assert out > .005


def test_reference_averages_wrapped_samples_during_settle():
    h=HeadingController(SharedGaitPolicy())
    for i in range(10):
        h.update(sample(359.9 if i%2 else .1),.8,0,0)
    reference=h.diagnostic()['reference_deg']
    assert abs((reference+180)%360-180) < .03


def test_constant_disturbance_rejected_without_windup():
    h=locked()
    yaw=0.
    for _ in range(3000):
        correction=h.update(sample(yaw),.8,0,0)
        # Simple independent yaw plant, clockwise actuator and CCW disturbance.
        yaw += (1.0-20*correction)*.02
    assert abs(yaw)<.15
    assert abs(correction-.05)<.01
