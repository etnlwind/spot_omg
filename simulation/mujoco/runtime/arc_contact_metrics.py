"""Evaluation-only foot contact metrics for the arc gait.

Contact forces and clearances are simulator observations, never controller inputs.
The recorded/raw phase and the commanded target phase are reported separately.
Only swings bounded by observed stance samples and without missing frames count
toward per-swing clearance statistics; a high peak in one cycle cannot validate
all the other cycles.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
from collections import defaultdict
import math

import numpy as np


LEGS = ('FL', 'FR', 'RL', 'RR')
PAIRS = (('FL', 'RR'), ('FR', 'RL'))


def _summary(values):
    values = np.asarray([value for value in values if value is not None and math.isfinite(value)], dtype=float)
    if not len(values):
        return dict(count=0, min=None, p10=None, median=None, max=None)
    return dict(count=len(values), min=float(values.min()), p10=float(np.percentile(values, 10)),
                median=float(np.median(values)), max=float(values.max()))


def _truth(value):
    return value if isinstance(value, bool) else str(value).lower() not in ('false', '0', 'none', '')


def _complete_swings(samples, phase_key, duty, threshold, sample_interval_s):
    completed, active = [], None
    excluded_partial = excluded_gap = 0
    previous = None
    for sample in samples:
        swing = sample[phase_key] >= duty
        if previous is None:
            if swing:
                active = dict(samples=[sample], before=None, gap=False)
            previous = sample
            continue
        gap = sample['time_s'] - previous['time_s'] > 1.5 * sample_interval_s
        if swing:
            if active is None:
                active = dict(samples=[], before=previous, gap=gap)
            active['samples'].append(sample)
            active['gap'] |= gap
        elif active is not None:
            active['gap'] |= gap
            if active['before'] is None:
                excluded_partial += 1
            elif active['gap']:
                excluded_gap += 1
            else:
                span = active['samples']
                airborne = [r for r in span if r['force_n'] <= threshold]
                # The adjacent stance samples distinguish a contact crossing
                # from an event already under way at the observation boundary.
                sequence = [active['before'], *span, sample]
                liftoffs = [(a, b) for a, b in zip(sequence, sequence[1:])
                            if a['force_n'] > threshold >= b['force_n'] and b in span]
                touchdowns = [(a, b) for a, b in zip(sequence, sequence[1:])
                              if a['force_n'] <= threshold < b['force_n'] and a in span]
                first = airborne[0] if airborne else None
                last = airborne[-1] if airborne else None
                lift = liftoffs[0][1] if liftoffs else None
                touch = touchdowns[-1][1] if touchdowns else None
                # Touchdown at the following stance boundary has phase near
                # zero. Keep it unwrapped (>=1) for meaningful phase summaries.
                touch_phase = touch[phase_key] if touch else None
                if touch_phase is not None and touch_phase < duty:
                    touch_phase += 1.
                completed.append(dict(
                    start_s=span[0]['time_s'], end_s=sample['time_s'], samples=len(span),
                    peak_clearance_mm=max(r['clearance_mm'] for r in span),
                    airborne_fraction=len(airborne) / len(span),
                    first_airborne_phase=first[phase_key] if first else None,
                    last_airborne_phase=last[phase_key] if last else None,
                    first_airborne_time_s=first['time_s'] if first else None,
                    last_airborne_time_s=last['time_s'] if last else None,
                    liftoff_phase=lift[phase_key] if lift else None,
                    touchdown_phase=touch_phase,
                ))
            active = None
        previous = sample
    if active is not None:
        excluded_partial += 1
    return completed, excluded_partial, excluded_gap


def arc_contact_metrics(rows, start_s=5., end_s=None, duty=.5, target_lead_s=.04,
                        period_s=1.2, sample_interval_s=.02, contact_threshold_n=.2):
    """Summarize raw ``rows`` or ``csv.DictReader`` rows without mutating them.

    ``phase`` is the per-leg phase before target lead. An optional per-leg
    ``target_phase`` column takes precedence over ``phase + lead / period``.
    Use actual command target phase when speed/period varies. End is exclusive.
    Missing/nonfinite rows are counted and prevent affected swings from being
    classified as complete. Missing measurements produce nulls, never passes.
    """
    if not (.5 <= duty < 1 and period_s > 0 and sample_interval_s > 0):
        raise ValueError('Invalid gait timing')
    if not all(math.isfinite(v) for v in (start_s, duty, target_lead_s, period_s,
                                        sample_interval_s, contact_threshold_n)):
        raise ValueError('Nonfinite evaluation parameters')
    if end_s is not None and (not math.isfinite(end_s) or end_s <= start_s):
        raise ValueError('Invalid evaluation window')
    by_leg = defaultdict(list)
    invalid = 0
    explicit_target = inferred_target = 0
    for row in rows:
        try:
            time = float(row['time_s'])
            if not math.isfinite(time):
                raise ValueError('Nonfinite timestamp')
            if time < start_s or (end_s is not None and time >= end_s):
                continue
            if 'moving' in row and not _truth(row['moving']):
                continue
            leg = str(row['leg']).upper()
            if leg not in LEGS:
                raise ValueError('Unknown leg')
            phase, clearance, force = (float(row[key]) for key in ('phase', 'clearance_mm', 'force_n'))
            has_target = row.get('target_phase') is not None and row.get('target_phase') != ''
            target = float(row['target_phase']) if has_target else phase + target_lead_s / period_s
            if not all(math.isfinite(value) for value in (phase, clearance, force, target)):
                raise ValueError('Nonfinite measurement')
            by_leg[leg].append(dict(time_s=time, phase=phase % 1., target_phase=target % 1.,
                                    clearance_mm=clearance, force_n=force))
            explicit_target += int(has_target)
            inferred_target += int(not has_target)
        except (KeyError, TypeError, ValueError):
            invalid += 1
    # A repeated sample is ambiguous rather than an extra interval of evidence.
    duplicates = 0
    for leg in LEGS:
        grouped = defaultdict(list)
        for sample in by_leg[leg]:
            grouped[sample['time_s']].append(sample)
        duplicates += sum(len(values) for values in grouped.values() if len(values) > 1)
        by_leg[leg] = [values[0] for _, values in sorted(grouped.items()) if len(values) == 1]
    result = dict(evaluation_only=True, start_s=start_s, end_s=end_s,
                  sample_interval_s=sample_interval_s, contact_threshold_n=contact_threshold_n,
                  duty=duty, target_lead_s=target_lead_s, period_s=period_s,
                  invalid_rows=invalid, duplicate_rows_excluded=duplicates,
                  target_phase_source=dict(explicit_rows=explicit_target, inferred_rows=inferred_target),
                  phase_bases={})
    for label, phase_key in (('raw', 'phase'), ('target', 'target_phase')):
        phase_result = dict(legs={}, diagonal_sync={})
        cycles = {}
        for leg in LEGS:
            samples = by_leg[leg]
            swings, partial, gaps = _complete_swings(samples, phase_key, duty, contact_threshold_n, sample_interval_s)
            cycles[leg] = swings
            middle = [sample for sample in samples if .2 <= (sample[phase_key] - duty) / (1-duty) <= .8]
            phase_result['legs'][leg] = dict(
                samples=len(samples), complete_swings=len(swings), excluded_partial_swings=partial,
                excluded_gap_swings=gaps,
                swing_peak_clearance_mm=_summary([s['peak_clearance_mm'] for s in swings]),
                swings_below_15mm=sum(s['peak_clearance_mm'] < 15. for s in swings),
                swings_without_airborne_sample=sum(s['first_airborne_phase'] is None for s in swings),
                liftoff_phase=_summary([s['liftoff_phase'] for s in swings]),
                touchdown_phase=_summary([s['touchdown_phase'] for s in swings]),
                first_airborne_phase=_summary([s['first_airborne_phase'] for s in swings]),
                last_airborne_phase=_summary([s['last_airborne_phase'] for s in swings]),
                middle_swing_samples=len(middle),
                middle_swing_contact_fraction=(sum(s['force_n'] > contact_threshold_n for s in middle) / len(middle)) if middle else None,
                swings=swings,
            )
        for a, b in PAIRS:
            samples_a = {sample['time_s']: sample for sample in by_leg[a]}
            samples_b = {sample['time_s']: sample for sample in by_leg[b]}
            common = sorted(samples_a.keys() & samples_b.keys())
            pairs = [(samples_a[time], samples_b[time]) for time in common
                     if samples_a[time][phase_key] >= duty and samples_b[time][phase_key] >= duty]
            differences = [x['clearance_mm'] - y['clearance_mm'] for x, y in pairs]
            starts_b = {round(s['start_s'] / sample_interval_s): s for s in cycles[b]}
            matched = [(s, starts_b[round(s['start_s'] / sample_interval_s)]) for s in cycles[a]
                       if round(s['start_s'] / sample_interval_s) in starts_b]
            def time_differences(key):
                return [abs(x[key] - y[key]) * 1000 for x, y in matched if x[key] is not None and y[key] is not None]
            phase_result['diagonal_sync'][f'{a}-{b}'] = dict(
                paired_swing_samples=len(pairs), matched_complete_swings=len(matched),
                clearance_difference_rms_mm=float(np.sqrt(np.mean(np.square(differences)))) if differences else None,
                clearance_difference_max_mm=max(map(abs, differences)) if differences else None,
                one_foot_contact_fraction=(sum((x['force_n'] > contact_threshold_n) != (y['force_n'] > contact_threshold_n)
                                                for x, y in pairs) / len(pairs)) if pairs else None,
                first_airborne_time_difference_ms=_summary(time_differences('first_airborne_time_s')),
                last_airborne_time_difference_ms=_summary(time_differences('last_airborne_time_s')),
            )
        result['phase_bases'][label] = phase_result
    frames = defaultdict(dict)
    for leg in LEGS:
        for sample in by_leg[leg]:
            frames[sample['time_s']][leg] = sample
    counts = [sum(s['force_n'] > contact_threshold_n for s in frame.values())
              for frame in frames.values() if len(frame) == len(LEGS)]
    result['contact_count'] = dict(
        complete_frames=len(counts), incomplete_frames=sum(len(frame) != len(LEGS) for frame in frames.values()),
        occupancy={str(n): counts.count(n) / len(counts) if counts else None for n in range(5)},
    )
    return result


if __name__ == '__main__':
    import argparse
    import csv
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--start', type=float, default=5.)
    parser.add_argument('--end', type=float)
    parser.add_argument('--target-lead', type=float, default=.04)
    parser.add_argument('--period', type=float, default=1.2)
    parser.add_argument('--duty', type=float, default=.5)
    args = parser.parse_args()
    with args.csv.open() as stream:
        result = arc_contact_metrics(csv.DictReader(stream), start_s=args.start, end_s=args.end,
                                     target_lead_s=args.target_lead, period_s=args.period, duty=args.duty)
    output = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    else:
        print(output, end='')
