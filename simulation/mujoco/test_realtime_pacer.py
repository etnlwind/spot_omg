import pytest
from realtime_pacer import PhysicsPacer


def test_slow_viewer_does_not_slow_physics_clock():
    p=PhysicsPacer(0)
    steps=sum(p.due(i*.033) for i in range(301))
    assert abs(steps*.02-300*.033)<=.020001


def test_long_pause_is_bounded_and_no_duplicate_step():
    p=PhysicsPacer(0)
    assert p.due(0)==1
    assert p.due(0)==0
    assert p.due(.10)==4
    assert p.due(3)==1
    assert p.due(3)==0


def test_invalid_period():
    with pytest.raises(ValueError):PhysicsPacer(0,0)
