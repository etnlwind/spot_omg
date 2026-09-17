"""Align one jointly armed joint/IMU capture offline; never controls hardware."""
import argparse
import csv
import json
import re
from pathlib import Path

from servo.imu_trace import parse as parse_imu
from servo.joint_trace import analyze


def align(joint_text, imu_text):
    report, joints = analyze(joint_text)
    origin = int(re.search(r'\$JT,C,0,(\d+),', joint_text)[1])
    _, samples = parse_imu(imu_text)
    imu = [dict(time_ms=((r[1]-origin+2**31) % 2**32)-2**31,
                roll_deg=r[2]/10, pitch_deg=r[3]/10,
                phase=r[4]/1000, rate=r[5]/1000, valid=r[6], fault=r[7])
           for r in samples]
    valid = [r for r in imu if r['valid']]
    if not joints or not valid or max(r['end'] for r in joints) < valid[0]['time_ms'] or min(r['begin'] for r in joints) > valid[-1]['time_ms']:
        raise ValueError('No overlapping valid joint/IMU observations')
    aligned = []
    for sample in sorted(joints, key=lambda r: r['time_ms']):
        row = dict(sample)
        # Motor front J1 signs differ from CAD; J2/J3 retain their signs.
        if row['joint'] in (0, 3):
            for key in ('target_deg', 'actual_deg', 'error_deg'):
                row[key] *= -1
        nearest = min(imu, key=lambda r: abs(r['time_ms']-row['time_ms']))
        gap = abs(nearest['time_ms']-row['time_ms'])
        row.update(imu_gap_ms=gap, imu_time_ms=None, roll_deg=None, pitch_deg=None)
        # Do not interpolate across missing/invalid IMU data.
        if nearest['valid'] and gap <= 40:
            row.update(imu_time_ms=nearest['time_ms'], roll_deg=nearest['roll_deg'], pitch_deg=nearest['pitch_deg'])
        aligned.append(row)
    crossings = {}
    for threshold in (3, 5, 10):
        first = next((r for r in valid if max(abs(r['roll_deg']), abs(r['pitch_deg'])) >= threshold), None)
        crossings[str(threshold)] = first
    return dict(joint_report=report, absolute_tilt_first_observation=crossings,
                limitations=['Inputs must be from the same jointly armed capture; overlapping clocks alone cannot verify provenance.',
                             'Joint readings are sequential, not a simultaneous pose.',
                             'Threshold observations are not exact onset times or proof of causation.',
                             'No foot contact sensor: angle error does not establish foot dragging.']), aligned, imu


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('jointtrace', type=Path)
    ap.add_argument('imutrace', type=Path)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    report, aligned, imu = align(args.jointtrace.read_text(), args.imutrace.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    report['inputs'] = [str(args.jointtrace.resolve()), str(args.imutrace.resolve())]
    (args.output/'alignment.json').write_text(json.dumps(report, indent=2))
    for name, rows in [('aligned-joints', aligned), ('imu', imu)]:
        with (args.output/(name+'.csv')).open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == '__main__':
    main()
