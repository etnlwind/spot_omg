"""Audit historical command gaps from complete MCU jointtrace dumps, offline.

Cross-checks the old CSV exports against the raw trace. Does not connect to a
robot, reconstruct per-attempt UART errors, or predict current firmware timing.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re

from servo.joint_trace import parse

ROOT = Path(__file__).resolve().parents[2]
TRACES = [
    'artifacts/servo-profile-restore-v69/hardware-30s/jointtrace.txt',
    'artifacts/s-native-v6-2-5/hardware-pilot-15s/jointtrace.txt',
    'artifacts/s-native-v6-2-5/hardware-30s/jointtrace-retry.txt',
    'artifacts/s-native-v6-2-7/resume-hardware-first-step-06/jointtrace.txt',
    'artifacts/attitudepd-v2/hardware-eight-steps/trial-02/jointtrace.txt',
]


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def inspect(relative, threshold):
    path = ROOT / relative
    raw = path.read_text(encoding='utf-8-sig')
    meta, joints, commands, samples = parse(raw)
    old_commands = read_csv(path.parent / 'joint-analysis/commands.csv')
    old_samples = read_csv(path.parent / 'joint-analysis/samples.csv')
    command_times = [(c['begin'], c['end']) for c in commands]
    if command_times != [(int(c['send_begin_ms']), int(c['send_end_ms']))
                         for c in old_commands]:
        raise ValueError(f'{relative}: CSV command timestamps differ')
    fields = ('begin', 'end', 'command', 'joint', 'status')
    sample_key = lambda s: tuple(int(s[k]) for k in fields)
    if sorted(map(sample_key, samples)) != sorted(map(sample_key, old_samples)):
        raise ValueError(f'{relative}: CSV sample timestamps/references differ')

    command_lines = {}
    sample_lines = []
    for number, line in enumerate(raw.splitlines(), 1):
        if line.startswith('$JT,C,'):
            command_lines[int(line.split(',')[2])] = number
        elif line.startswith('$JT,S,'):
            sample_lines.append(number)
    origin = int(next(line for line in raw.splitlines()
                      if line.startswith('$JT,C,0,')).split(',')[3])
    durations = Counter(s['end'] - s['begin'] for s in samples)
    events = []
    for index, (command, following) in enumerate(zip(commands, commands[1:])):
        gap = following['begin'] - command['begin']
        if gap <= threshold:
            continue
        reads = []
        for sample, line in zip(samples, sample_lines):
            if sample['command'] != index:
                continue
            joint = sample['joint']
            reads.append({**sample, 'raw_line': line,
                          'duration_ms': sample['end'] - sample['begin'],
                          'servo_id': joints[joint]['id'],
                          'joint_name': f"{('FL', 'FR', 'RL', 'RR')[joint//3]}-J{joint%3+1}"})
        tx = command['end'] - command['begin']
        read_total = sum(r['duration_ms'] for r in reads)
        events.append(dict(command_index=index, next_command_index=index+1,
            command_raw_line=command_lines[index],
            next_command_raw_line=command_lines[index+1],
            begin_ms=command['begin'], send_end_ms=command['end'],
            next_begin_ms=following['begin'], interval_ms=gap,
            send_duration_ms=tx, read_duration_sum_ms=read_total,
            other_uninstrumented_ms=gap-tx-read_total, reads=reads,
            neighboring_commands=[dict(index=i, begin_ms=commands[i]['begin'],
                end_ms=commands[i]['end'])
                for i in range(max(0, index-2), min(len(commands), index+4))]))

    response_path = path.parent / 'drive-response.txt'
    response = response_path.read_text(encoding='utf-8-sig') if response_path.exists() else ''
    retries = re.search(r'Bus read retries: attempts=(\d+) recovered=(\d+) failed=(\d+)', response)
    late = re.search(r'late_frames=(\d+)', response)
    transcript_path = path.parent / 'transcript.jsonl'
    setup = []
    if transcript_path.exists():
        for line in transcript_path.read_text(encoding='utf-8-sig').splitlines():
            entry = json.loads(line)
            if entry.get('kind') == 'tx' and entry.get('text') == 'imu off':
                setup.append(entry)
    return dict(source=relative, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        metadata=meta, timestamp_origin_mcu_ms=origin,
        csv_timing_crosscheck='all command and sample records match',
        read_duration_histogram_ms=dict(sorted(durations.items())),
        max_interval_ms=max(b['begin']-a['begin'] for a,b in zip(commands,commands[1:])),
        full_trial_retry_counts=dict(zip(('attempts','recovered','failed'),map(int,retries.groups()))) if retries else None,
        full_trial_late_frames=int(late[1]) if late else None,
        imu_console_off_commands=setup, events=events)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--threshold-ms', default=30, type=int)
    args = parser.parse_args()
    report = dict(schema=1, threshold_ms=args.threshold_ms,
        traces=[inspect(source, args.threshold_ms) for source in TRACES],
        limitations=[
            'Historical V69-V78 captures; not measurements of the current build.',
            'MCU timestamps have 1 ms resolution; send duration zero is not instantaneous.',
            'Read intervals include retries and interrupts, not just wire transmission.',
            'Final status zero does not imply every attempt succeeded.',
            'Retry counters and late_frames cover the full trial, unlike finite trace captures.',
            'The remainder includes uninstrumented processing, IMU, scheduling and next computation.',
            'No per-attempt error flags, first-byte timing, ISR duration or electrical waveform.',
        ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    for trace in report['traces']:
        print(trace['source'], 'max_interval_ms=', trace['max_interval_ms'],
              'events=', len(trace['events']), 'CSV matches')
        for event in trace['events']:
            print(json.dumps(event, ensure_ascii=False))


if __name__ == '__main__':
    main()
