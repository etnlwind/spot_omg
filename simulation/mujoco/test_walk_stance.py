from pathlib import Path
import numpy as np
import pytest
from servo import SharedGaitPolicy
from cad_physics import KEYS
from compare_stance import shift_height


def test_shared_c_matches_simulated_height_for_all_directions():
    p=SharedGaitPolicy()
    for scale in (0.,.3,1.):
        for phase in np.linspace(0,1,81):
            for linear,yaw in ((1,0),(-1,0),(0,-.5),(0,.5),(.5,.5),(0,0)):
                old,support=p.drive_targets(float(phase),scale,linear,yaw)
                actual,got=p.drive_walk_targets(float(phase),scale,linear,yaw)
                expected=shift_height([old[k] for k in KEYS],40)
                np.testing.assert_allclose([actual[k] for k in KEYS],expected,atol=.0001,rtol=0)
                assert support==got
                if scale==0:
                    for leg in ('FL','FR','RL','RR'):
                        assert actual[leg,2]==pytest.approx(40,abs=.0001)
                        assert actual[leg,3]==pytest.approx(80,abs=.0001)


@pytest.mark.parametrize('phase,scale,linear,yaw',[(float('nan'),1,1,0),(0,1.1,1,0),(0,1,2,0),(0,1,1,float('inf'))])
def test_invalid_walk_requests_rejected(phase,scale,linear,yaw):
    with pytest.raises(ValueError):SharedGaitPolicy().drive_walk_targets(phase,scale,linear,yaw)


def test_firmware_preparation_reference_and_return_use_walk_pose():
    root=Path(__file__).resolve().parents[2]
    source=(root/'firmware/stm32-learning/Src/robot.c').read_text()
    config=(root/'firmware/stm32-learning/Src/robot_config.c').read_text()
    assert 'return robot_pose_targets(45, 90, targets);' in config
    assert 'transition_gait_pose(robot, false, continuous_drive)' in source
    assert 'gait_policy_drive_walk_targets(0, 0, 0, 0, leg_targets)' in source
    assert 'transition_gait_pose(robot, true, true)' in source
    assert 'robot->balance_enabled && !optimized' in source
