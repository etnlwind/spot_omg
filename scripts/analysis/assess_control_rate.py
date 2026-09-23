"""Summarize recorded hardware timestamps and wire budgets; no robot I/O.

These historical 50 Hz traces do not predict 100 Hz motion or current WCET.
Timestamps have millisecond resolution; zero-duration writes are not instant.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOURCES = [
    'artifacts/servo-profile-restore-v69/hardware-30s/joint-analysis',
    'artifacts/s-native-v6-2-5/hardware-pilot-15s/joint-analysis',
    'artifacts/s-native-v6-2-5/hardware-30s/joint-analysis',
    'artifacts/s-native-v6-2-7/resume-hardware-first-step-06/joint-analysis',
    'artifacts/attitudepd-v2/hardware-eight-steps/trial-02/joint-analysis',
]


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return dict(n=len(values), minimum=float(values.min()),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), maximum=float(values.max()))


def inspect(source):
    with (ROOT/source/'commands.csv').open(encoding='utf-8-sig', newline='') as f:
        commands = list(csv.DictReader(f))
    with (ROOT/source/'samples.csv').open(encoding='utf-8-sig', newline='') as f:
        samples = list(csv.DictReader(f))
    starts = np.array([int(r['send_begin_ms']) for r in commands])
    gaps = np.diff(starts)
    reads = [int(r['end'])-int(r['begin']) for r in samples]
    fresh = []
    for joint in range(12):
        stamps = [int(r['end']) for r in samples
                  if int(r['joint']) == joint and int(r['status']) == 0]
        fresh.extend(np.diff(stamps).tolist())
    keys = [k for k in commands[0] if k.endswith('_deg')]
    targets = np.array([[float(r[k]) for k in keys] for r in commands])
    return dict(source=source, command_count=len(commands),
                captured_span_ms=int(starts[-1]-starts[0]),
                send_interval_ms=statistics(gaps),
                send_duration_ms=statistics([int(r['send_end_ms'])-int(r['send_begin_ms']) for r in commands]),
                read_duration_ms=statistics(reads),
                same_joint_feedback_interval_ms=statistics(fresh),
                intervals_over_21ms=int(np.sum(gaps > 21)),
                intervals_over_30ms=int(np.sum(gaps > 30)),
                unsuccessful_samples=sum(int(r['status']) != 0 for r in samples),
                max_target_change_deg=float(np.max(abs(np.diff(targets, axis=0)))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rates = []
    for hz in (50, 75, 100, 200):
        dt = 1/hz
        rates.append(dict(hz=hz, period_ms=1000*dt,
            position_write_bytes=44, position_write_wire_ms=.44,
            position_write_wire_percent=.44/(1000*dt)*100,
            full_profile_write_wire_percent=1.04/(1000*dt)*100,
            nominal_round_robin_feedback_ms=12*1000*dt,
            held_target_mean_age_ms=500*dt,
            fixed_alpha_heading_time_constant_ms=-1000*dt/math.log(.9),
            heading_alpha_preserving_50hz_response=1-.9**(dt/.02)))
    report = dict(historical_traces=[inspect(s) for s in SOURCES], rates=rates,
        limitations=[
            'Historical versions/settings differ from current ACC254 build; not new measurements.',
            'Recorders cover finite captures, not full gait trials; timestamps are quantized to milliseconds.',
            'Send intervals include compute-to-send variation, not exact control-loop start intervals.',
            'Wire occupancy excludes reads, turnaround, IMU, computation, retries and console work.',
            'One-joint-per-frame feedback assumes no retries, stalls or prioritized joint watching.',
            'Average held-target age assumes uniform sampling; not measured servo response latency.',
            'No hardware-optimal frequency is established by these calculations.',
        ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
