"""Turn governor boundaries and one physical left-to-forward regression."""
import pytest
import mujoco
from servo import SharedGaitPolicy
from cad_physics import build
from check_drive_transition import evaluate


def test_turn_limit_preserves_fine_control_and_stop():
    policy = SharedGaitPolicy()
    for requested in range(-1000, 1001):
        actual = policy.drive_yaw_limit(requested)
        assert actual == max(-500, min(500, requested))
        assert policy.drive_yaw_limit(-requested) == -actual
    for invalid in (1001, -1001, float('nan'), .5):
        with pytest.raises(ValueError): policy.drive_yaw_limit(invalid)


def test_left_to_forward_reduces_worst_nominal_phase_tilt():
    xml, p = build(write_scene=False)
    model = mujoco.MjModel.from_xml_string(xml)
    before, _ = evaluate(p, model, 4.9, legacy=True)
    after, rows = evaluate(p, model, 4.9)
    assert before['reached_transition'] and after['reached_transition']
    assert not before['fallen'] and not after['fallen']
    assert after['peak_transition_tilt_deg'] < before['peak_transition_tilt_deg']*.7
    assert rows[-1]['time_s'] > 8.8
