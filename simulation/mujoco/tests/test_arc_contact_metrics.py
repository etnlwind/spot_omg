
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import math

import pytest

from simulation.mujoco.runtime.arc_contact_metrics import arc_contact_metrics


def samples():
    # One partial opening swing, one complete swing, one partial closing swing.
    # All four are synchronous here solely to exercise the evaluator boundaries.
    return [dict(time_s=i*.1, leg=leg, phase=((i+6) % 10)/10,
                 clearance_mm=20 if (i+6) % 10 in (7, 8) else 0,
                 force_n=0 if (i+6) % 10 in (7, 8) else 10, moving='True')
            for i in range(20) for leg in ('FL', 'FR', 'RL', 'RR')]


def measure(rows, **kwargs):
    return arc_contact_metrics(rows, start_s=0, period_s=1., sample_interval_s=.1,
                               target_lead_s=0, **kwargs)


def test_partial_swings_are_excluded_and_complete_peaks_are_per_cycle():
    result = measure(samples())
    leg = result['phase_bases']['raw']['legs']['FL']
    assert leg['complete_swings'] == 1
    assert leg['excluded_partial_swings'] == 2
    assert leg['swing_peak_clearance_mm'] == dict(count=1, min=20., p10=20., median=20., max=20.)
    assert leg['liftoff_phase']['median'] == pytest.approx(.7)
    assert leg['touchdown_phase']['median'] == pytest.approx(.9)
    assert result['phase_bases']['raw']['diagonal_sync']['FL-RR']['clearance_difference_rms_mm'] == 0
    assert sum(result['contact_count']['occupancy'].values()) == pytest.approx(1)


@pytest.mark.parametrize('missing', ['removed', 'nonfinite', 'duplicate'])
def test_missing_frame_invalidates_complete_swing_without_inventing_zero(missing):
    rows = samples()
    if missing == 'removed':
        rows = [row for row in rows if not (row['leg'] == 'FL' and math.isclose(row['time_s'], 1.2))]
    elif missing == 'nonfinite':
        next(row for row in rows if row['leg'] == 'FL' and math.isclose(row['time_s'], 1.2))['clearance_mm'] = float('nan')
    else:
        rows.append(dict(next(row for row in rows if row['leg'] == 'FL' and math.isclose(row['time_s'], 1.2))))
    result = measure(rows)
    leg = result['phase_bases']['raw']['legs']['FL']
    assert leg['complete_swings'] == 0
    assert leg['excluded_gap_swings'] == 1
    assert leg['swing_peak_clearance_mm']['min'] is None
    assert result['contact_count']['incomplete_frames'] == 1


def test_explicit_target_phase_is_separate_from_raw_and_takes_precedence():
    rows = samples()
    for row in rows:
        row['target_phase'] = (row['phase'] + .2) % 1
    result = measure(rows)
    raw = result['phase_bases']['raw']['legs']['FL']
    target = result['phase_bases']['target']['legs']['FL']
    assert raw['first_airborne_phase']['median'] == pytest.approx(.7)
    assert target['first_airborne_phase']['median'] == pytest.approx(.9)
    assert result['target_phase_source']['explicit_rows'] == len(rows)
    assert result['target_phase_source']['inferred_rows'] == 0


def test_empty_or_excluded_data_reports_missing_metrics():
    result = measure([])
    assert result['phase_bases']['raw']['legs']['FL']['complete_swings'] == 0
    assert result['phase_bases']['raw']['legs']['FL']['middle_swing_contact_fraction'] is None
    assert all(value is None for value in result['contact_count']['occupancy'].values())
