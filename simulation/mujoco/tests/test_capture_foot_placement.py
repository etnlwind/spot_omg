
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
from types import SimpleNamespace
import numpy as np
from simulation.mujoco.scripts.analysis.capture_foot_placement import CaptureFootPlacement


def sensor(roll, rate=0):
    return SimpleNamespace(filtered=[roll * 10, 0], previous=[roll * 10, 0],
                           rate=[rate * 10, 0], failures=0)


def test_positive_roll_catches_toward_negative_y():
    planner = CaptureFootPlacement()
    a = sensor(5, 30)
    planner.update(.61, .62, a, .2)
    planner.update(.621, .62, a, .2)
    out = planner.update(.999999, .62, a, .2)
    expected = -.25 * .2 * (np.radians(5) + np.radians(30) / np.sqrt(9.81 / .2))
    np.testing.assert_allclose(out[[0, 3]], expected, atol=1e-12)
    assert out[0] < 0


def test_stance_offsets_are_fixed_when_sensor_changes():
    planner = CaptureFootPlacement()
    planner.update(.621, .62, sensor(5), .2)
    old = planner.update(0., .62, sensor(5), .2)
    new = planner.update(.3, .62, sensor(-5), .2)
    np.testing.assert_array_equal(old[[0, 3]], new[[0, 3]])


def test_bound_synchrony_and_continuity():
    planner = CaptureFootPlacement(); rows=[]
    for phase in np.arange(.60, 1.02, .0005):
        out=planner.update(phase % 1, .62, sensor(20, 90), .2)
        assert max(abs(out)) <= .01000001
        assert out[0] == out[3]
        rows.append(out)
    assert max(abs(np.diff(rows, axis=0)).ravel()) < .00006


def test_missing_sensor_retains_finite_latched_endpoint():
    planner = CaptureFootPlacement()
    planner.update(.621, .62, sensor(5), .2)
    endpoint=planner.end.copy()
    planner.update(.9, .62, None, .2)
    np.testing.assert_array_equal(planner.end, endpoint)
    assert np.isfinite(planner.offset).all()
    assert not planner.diagnostic['sensor_valid']
