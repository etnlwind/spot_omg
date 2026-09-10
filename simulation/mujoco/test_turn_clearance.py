"""Regression for the low-input pivot/arc which previously dragged the feet."""
import pytest
from diagnose_turn_clearance import run


@pytest.mark.parametrize('yaw',[-221,221])
def test_low_speed_turn_lifts_each_foot_without_excessive_tilt(yaw):
    summary,_=run(177,yaw,14)
    assert summary['safety']=='ok'
    for leg in summary['legs'].values():
        assert leg['samples']==450  # A fault/stopped robot is not a passing gait.
        assert leg['swing_samples']>100
        assert leg['middle_swing_contact_fraction']<.1
        assert leg['peak_clearance_mm']>10
        assert leg['peak_tilt_deg']<5
