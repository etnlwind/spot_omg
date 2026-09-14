"""Actual diagonal liftoff/landing timing from recorded floor contact forces.

Debounce: a new force state must persist for >=40ms (three 20ms samples).
Accepted event time is backdated to the first sample of that persistent run.
The same script evaluates 0.5/1/2N thresholds. This measures events only and
does not infer a causal compensation from timing correlation.
"""
from pathlib import Path
import argparse
import json
import numpy as np

LEGS = ('FL', 'FR', 'RL', 'RR')
PAIRS = ((0, 3), (1, 2))
OFFSETS = np.array([0., .5, .5, 0.])


def contact_runs(times, force, threshold, debounce_s=.04):
    stable = bool(force[0] > threshold)
    pending = None; pending_at = None; events = []
    for time, value in zip(times[1:], force[1:]):
        raw = bool(value > threshold)
        if raw == stable:
            pending = pending_at = None
        elif pending != raw:
            pending = raw; pending_at = float(time)
        elif time - pending_at >= debounce_s - 1e-8:
            stable = raw
            events.append(dict(type='touchdown' if raw else 'liftoff', time_s=pending_at, confirmed_s=float(time)))
            pending = pending_at = None
    flights = []; started = None
    for event in events:
        if event['type'] == 'liftoff':
            started = event['time_s']
        elif started is not None:
            flights.append(dict(liftoff_s=started, touchdown_s=event['time_s']))
            started = None
    if started is not None:
        flights.append(dict(liftoff_s=started, touchdown_s=None))
    return events, flights


def snapshot(records, time):
    row = min(records, key=lambda r: abs(r['t'] - time))
    command = np.asarray(row['command']).reshape(4, 3)
    actual = np.asarray(row['actual']).reshape(4, 3)
    return dict(time_s=row['t'], roll_deg=row['roll'], pitch_deg=row['pitch'],
        force_n=row['force'], clearance_mm=row['clearance'], command_deg=command.tolist(),
        actual_deg=actual.tolist(), command_minus_actual_deg=(command - actual).tolist())


def statistic(values):
    values = np.asarray([v for v in values if v is not None])
    if len(values) == 0:
        return None
    return dict(count=len(values), signed_median_ms=float(np.median(values) * 1000),
        absolute_median_ms=float(np.median(abs(values)) * 1000),
        absolute_p90_ms=float(np.quantile(abs(values), .9) * 1000),
        absolute_max_ms=float(abs(values).max() * 1000))


def analyze(path, command_start_override=None):
    source = json.loads(Path(path).read_text())
    records = source['records']; config = source.get('config', {})
    times = np.array([r['t'] for r in records])
    forces = np.array([r['force'] for r in records])
    if command_start_override is None:
        active = np.flatnonzero(np.array([r.get('linear', 0.) for r in records]) > .01)
    else:
        active = np.flatnonzero(times >= command_start_override - 1e-8)
    if len(active) == 0:
        raise ValueError('No forward command samples')
    start_index = active[0]; walking = records[start_index:]
    walk_times = times[start_index:]
    phase = np.unwrap(np.array([r['phase'] for r in walking]) * 2 * np.pi) / (2 * np.pi)
    command_start = float(walk_times[0])
    startup_end = command_start + float(config.get('startup_s', 1.))
    duty = float(config.get('duty', .52))
    thresholds = {}
    for threshold in (.5, 1., 2.):
        events = []; flights = []
        for leg in range(4):
            ev, fl = contact_runs(times, forces[:, leg], threshold)
            events.append(ev); flights.append(fl)
        pairs = []
        for front, rear in PAIRS:
            local = phase + OFFSETS[front]
            first_cycle = int(np.floor(local[0]))
            last_cycle = int(np.floor(local[-1]))
            for cycle in range(first_cycle, last_cycle + 1):
                lift_phase = cycle + duty; land_phase = cycle + 1.
                if lift_phase < local[0] or land_phase > local[-1]:
                    continue
                expected_lift = float(np.interp(lift_phase, local, walk_times))
                expected_land = float(np.interp(land_phase, local, walk_times))
                cycle_start = float(np.interp(cycle, local, walk_times))
                complete_cycle = cycle >= local[0] - 1e-8
                period = (expected_land - expected_lift) / (1 - duty)
                margin = .15 * period
                selected = []
                extra_flights = []
                for leg in (front, rear):
                    eligible = []
                    for flight in flights[leg]:
                        on = flight['liftoff_s']; off = flight['touchdown_s']
                        end = float(times[-1]) if off is None else off
                        overlap = max(0., min(expected_land, end) - max(expected_lift, on))
                        # A previous-stance bounce overlapping this swing by
                        # one boundary sample is not a detected swing flight.
                        if overlap >= .04 - 1e-8 and expected_lift - margin <= on <= expected_land:
                            eligible.append((overlap, flight))
                    chosen = max(eligible, key=lambda x:x[0])[1].copy() if eligible else None
                    if chosen is not None:
                        chosen['liftoff_minus_plan_s'] = chosen['liftoff_s'] - expected_lift
                        chosen['touchdown_minus_plan_s'] = None if chosen['touchdown_s'] is None else chosen['touchdown_s'] - expected_land
                    selected.append(chosen)
                    extra_flights.append([f for _, f in eligible])
                front_flight, rear_flight = selected
                lift_delta = (rear_flight['liftoff_s'] - front_flight['liftoff_s']) if all(selected) else None
                land_delta = (rear_flight['touchdown_s'] - front_flight['touchdown_s']) if all(selected) and all(f['touchdown_s'] is not None for f in selected) else None
                pairs.append(dict(pair=[LEGS[front], LEGS[rear]], cycle=cycle, cycle_start_s=cycle_start,
                    complete_cycle=bool(complete_cycle), startup=bool(expected_lift < startup_end),
                    expected_liftoff_s=expected_lift, expected_touchdown_s=expected_land,
                    front=front_flight, rear=rear_flight, rear_minus_front_liftoff_s=lift_delta,
                    rear_minus_front_touchdown_s=land_delta, all_overlapping_flights=extra_flights))
        summaries = {}
        for period_name in ('startup', 'steady'):
            summaries[period_name] = {}
            for front, rear in PAIRS:
                subset = [p for p in pairs if p['pair'] == [LEGS[front], LEGS[rear]]
                          and p['complete_cycle'] and p['startup'] == (period_name == 'startup')]
                summaries[period_name]['/'.join((LEGS[front], LEGS[rear]))] = dict(
                    complete_cycle_count=len(subset), missing_front_flight=sum(p['front'] is None for p in subset),
                    missing_rear_flight=sum(p['rear'] is None for p in subset),
                    multiple_flight_cycle_count=sum(any(len(f) > 1 for f in p['all_overlapping_flights']) for p in subset),
                    liftoff_pair_difference=statistic([p['rear_minus_front_liftoff_s'] for p in subset]),
                    touchdown_pair_difference=statistic([p['rear_minus_front_touchdown_s'] for p in subset]))
        divergent = [p for p in sorted(pairs, key=lambda p:p['expected_liftoff_s']) if
                     p['front'] is None or p['rear'] is None or
                     abs(p['rear_minus_front_liftoff_s'] or 0) >= .04 - 1e-8 or
                     abs(p['rear_minus_front_touchdown_s'] or 0) >= .04 - 1e-8]
        first = divergent[0] if divergent else None
        first_steady = next((p for p in divergent if not p['startup'] and p['complete_cycle']), None)
        snapshots = {}
        for key, event in (('first_divergence', first), ('first_steady_divergence', first_steady)):
            if event:
                when = event['front']['liftoff_s'] if event['front'] else event['expected_liftoff_s']
                snapshots[key] = snapshot(records, when)
                snapshots[key + '_plus_100ms'] = snapshot(records, when + .1)
                if event['front'] and event['front']['touchdown_s'] is not None:
                    snapshots[key + '_front_touchdown'] = snapshot(records, event['front']['touchdown_s'])
                if event['rear'] and event['rear']['liftoff_s'] is not None:
                    snapshots[key + '_rear_liftoff'] = snapshot(records, event['rear']['liftoff_s'])
        thresholds[str(threshold)] = dict(summary=summaries, pairs=pairs,
            first_divergence=first, first_steady_divergence=first_steady,
            snapshots=snapshots, events={LEGS[i]:events[i] for i in range(4)})
    return dict(source=str(path), config=config, command_start_s=command_start,
        startup_end_s=startup_end, duty=duty, record_end_s=float(times[-1]),
        definition='Contact iff summed floor normal force > threshold; state persists 40ms; accepted event backdated to first sample. Force-defined unloading need not imply positive geometric clearance.',
        assignment='Dominant 40ms-debounced flight with at least 40ms overlap inside planned swing; additional eligible flights and missing events retained. Touchdown is first sustained contact after this flight, not necessarily final landing if the foot lifts again.',
        time_resolution_s=float(np.median(np.diff(times))), threshold_results=thresholds,
        sample_near_14s=snapshot(records, 14.),
        limitation='Timing correlation alone does not identify commanded-axis, servo, contact, or body-motion cause; no phase corrections inferred.')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('sources',nargs='+');parser.add_argument('--output',required=True)
    parser.add_argument('--command-start',type=float,default=None,help='Explicit command epoch for causal records without linear field')
    args=parser.parse_args();results=[analyze(source,args.command_start) for source in args.sources]
    Path(args.output).write_text(json.dumps(results,indent=2))
    for result in results:
        print(json.dumps(dict(source=result['source'],summary=result['threshold_results']['1.0']['summary'],
            first_steady_divergence=result['threshold_results']['1.0']['first_steady_divergence']),indent=2))


if __name__=='__main__':main()
