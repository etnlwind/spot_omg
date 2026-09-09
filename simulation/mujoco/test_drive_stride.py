"""Verify stride extension changes excursion, not lift/neutral/turn commands."""
import math
import numpy as np
import pytest
from servo import SharedGaitPolicy


def foot(target,leg):
    hip=math.radians(target[leg,2]);lower=hip-math.radians(target[leg,3])
    return np.array([math.sin(hip)+math.sin(lower),math.cos(hip)+math.cos(lower)])


def test_forward_excursion_grows_without_extra_lift():
    policy=SharedGaitPolicy()
    for amplitude in (0.,.3,1.):
        for linear in (.2,1.):
            for leg in ('FL','FR','RL','RR'):
                before=[];after=[]
                for phase in np.linspace(0,1,101):
                    a,sa=policy.drive_stride_targets(float(phase),amplitude,linear,0,1.)
                    b,sb=policy.drive_stride_targets(float(phase),amplitude,linear,0,1.6)
                    assert sa==sb
                    assert a[leg,1]==b[leg,1]
                    before.append(foot(a,leg));after.append(foot(b,leg))
                a=np.array(before);b=np.array(after)
                np.testing.assert_allclose(a[:,1],b[:,1],atol=1e-6,rtol=0)
                assert np.ptp(b[:,0])==pytest.approx(1.6*np.ptp(a[:,0]),abs=2e-6)
                assert (b[:,0].max()+b[:,0].min())==pytest.approx(a[:,0].max()+a[:,0].min(),abs=2e-6)


def test_backward_turn_and_neutral_are_preserved():
    p=SharedGaitPolicy()
    for phase in np.linspace(0,1,101):
        for linear,yaw in ((0,0),(0,.5),(0,-.5),(-1,0),(-.3,.5)):
            a,sa=p.drive_stride_targets(float(phase),1,linear,yaw,1.)
            b,sb=p.drive_stride_targets(float(phase),1,linear,yaw,1.6)
            assert a==b and sa==sb
        for stride in (1.,1.6):
            a,_=p.drive_stride_targets(float(phase),0,1,0,stride)
            assert abs(a['FL',2]-45)<.001 and abs(a['FL',3]-90)<.001


def test_selected_default_and_mixed_commands_stay_in_joint_limits():
    p=SharedGaitPolicy()
    for phase in np.linspace(0,1,201):
        for linear in (0.,.3,1.):
            for yaw in (-.5,0.,.5):
                a,sa=p.drive_targets(float(phase),1,linear,yaw)
                b,sb=p.drive_stride_targets(float(phase),1,linear,yaw,1.6)
                assert a==b and sa==sb
                for (leg,joint),angle in a.items():
                    lo,hi={1:(-30,30),2:(-45,100),3:(0,150)}[joint]
                    assert lo<=angle<=hi


@pytest.mark.parametrize('stride',[float('nan'),float('inf'),.9,2.1])
def test_invalid_stride_rejected(stride):
    with pytest.raises(ValueError):SharedGaitPolicy().drive_stride_targets(0,1,1,0,stride)
