import numpy as np
from ground_foothold_transfer import FootholdTransfer


GOALS = np.array([[.2, .1, 0.], [.2, -.1, 0.], [-.2, .1, 0.], [-.2, -.1, 0.]])


def test_swing_preplacement_reduces_touchdown_line_error():
    planner = FootholdTransfer()
    cfg = dict(touchdown_x_offset_m=.04)
    planner.update(GOALS, GOALS, np.zeros(3), .51, .52, 1., cfg)
    planner.update(GOALS, GOALS, np.zeros(3), .521, .52, 1., cfg)
    event = planner.diagnostic['touchdown_events'][0]
    assert abs(event['support_line_distance_before_m']) > .01
    assert abs(event['support_line_distance_after_m']) < 1e-10


def test_stance_y_is_fixed_and_xz_are_unchanged():
    planner = FootholdTransfer()
    for phase in np.linspace(.51, 1.01, 200):
        result = planner.update(GOALS, GOALS, np.zeros(3), phase % 1., .52, 1., dict(touchdown_x_offset_m=.04))
    y = result[[0, 3], 1].copy()
    for phase in np.linspace(.02, .49, 25):
        changed = GOALS.copy(); changed[:, 0] += .01 * np.sin(phase)
        changed[:, 1] += .002; changed[:, 2] += .003
        result = planner.update(changed, GOALS, np.zeros(3), phase, .52, 1., dict(touchdown_x_offset_m=.04))
        np.testing.assert_array_equal(result[[0, 3], 1], y)
        np.testing.assert_array_equal(result[:, [0, 2]], changed[:, [0, 2]])


def test_pair_synchrony_bounds_and_boundary_continuity():
    planner = FootholdTransfer()
    samples=[]
    for phase in np.arange(.50, 1.025, .0005):
        result = planner.update(GOALS, GOALS, np.array([.08, 0., .2]), phase % 1., .52, 1., dict(touchdown_x_offset_m=.04, max_foothold_y_m=.01))
        offset=result[:, 1] - GOALS[:, 1]
        assert np.max(abs(offset)) <= .01000001
        assert abs(offset[0] - offset[3]) < 1e-12
        samples.append(result[:, 1])
    differences=np.diff(samples, axis=0)
    assert np.max(abs(differences)) < .00005
