"""The selectable shared-C profile must reproduce the saved experiment."""
import numpy as np
import pytest
from servo import SharedGaitPolicy
from drive_controller import step
from arc_preload import ArcTransfer
from test_arc_trial_contracts import robot

PARAMETERS=dict(arc_trial=[.022,.5,.04,0],arc_wave_trial=[.0017557040361956876,0,
    -.0009905324626316228,.0007808772327605116,.0012395485171061073,.0013113643286768628])


@pytest.mark.parametrize('linear,yaw',[(0,-1),(0,1),(.7,0),(.3,.6),(-.6,-.4)])
def test_shared_profile_reproduces_reference_cartesian_wave_and_stateful_load(linear,yaw):
    policy=SharedGaitPolicy()
    reference=robot(policy,0,0,PARAMETERS);reference.request=(linear,yaw)
    deployed=robot(policy,0,0);deployed.profile='arcsupport';deployed.request=(linear,yaw)
    reference.phase=deployed.phase=reference.elapsed=deployed.elapsed=0
    transfer=ArcTransfer(policy)
    for frame in range(350):
        if frame==250:
            reference.request=deployed.request=(0,0)
            reference.stopping_reason=deployed.stopping_reason='test'
        nominal=step(reference)
        gain=.3*min(1,abs(reference.linear)+abs(reference.yaw))
        expected=nominal+transfer.correction(nominal,reference.arc_frame,[.12,0,gain])
        actual=step(deployed)
        np.testing.assert_allclose(actual,expected,atol=.002,rtol=0)
        assert deployed.phase==reference.phase
        assert max(abs(np.array(deployed.arc_support_state)))<=4
