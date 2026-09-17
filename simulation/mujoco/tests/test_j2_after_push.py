from types import SimpleNamespace

import numpy as np

from simulation.mujoco.scripts.analysis.capture_j2_85_candidate import high_j2, high_j2_after_entry


def test_entry_and_first_post_support_recovery():
    nominal=np.tile([-9.,55.,90.],4)
    # First FR/RL entry is identical to the preserved policy.
    gait=SimpleNamespace(stop_progress=None,entry_phase=.125)
    np.testing.assert_array_equal(high_j2(None,gait,.625,nominal),nominal)


def test_both_entry_steps_preserved_before_symmetric_recovery():
    nominal=np.tile([-9.,55.,90.],4)
    for elapsed in np.linspace(0.,.9999,101):
        gait=SimpleNamespace(stop_progress=None,entry_phase=elapsed)
        np.testing.assert_array_equal(high_j2_after_entry(None,gait,(.5+elapsed)%1,nominal),nominal)
    gait=SimpleNamespace(stop_progress=None,entry_phase=1.)
    np.testing.assert_array_equal(high_j2_after_entry(None,gait,.5,nominal),nominal)
    gait.entry_phase=1.125
    q=high_j2_after_entry(None,gait,.625,nominal)
    np.testing.assert_allclose(q[[4,7]],85.)
    np.testing.assert_allclose(q[[5,8]],104.)
    gait.entry_phase=1.625
    q=high_j2_after_entry(None,gait,.125,nominal)
    np.testing.assert_allclose(q[[1,10]],85.)
    # FL/RR have now completed their first support interval.
    gait.entry_phase=.625
    q=high_j2(None,gait,.125,nominal)
    np.testing.assert_allclose(q[[1,10]],85.)
    np.testing.assert_allclose(q[[2,11]],104.)
    np.testing.assert_array_equal(q[3:9],nominal[3:9])
    # FR/RL become eligible only after their first landing and support.
    gait.entry_phase=1.125
    q=high_j2(None,gait,.625,nominal)
    np.testing.assert_allclose(q[[4,7]],85.)
    np.testing.assert_allclose(q[[5,8]],104.)
    np.testing.assert_array_equal(q[::3],nominal[::3])
    gait.stop_progress=0.
    np.testing.assert_array_equal(high_j2(None,gait,.625,nominal),nominal)
