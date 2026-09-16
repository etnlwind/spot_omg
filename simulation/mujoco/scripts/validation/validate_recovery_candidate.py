"""Offline early STOP and uncertainty checks, without changing saved servo settings."""
import argparse
from contextlib import nullcontext
import gzip
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from simulation.mujoco.runtime.servo_profile import ServoProfile
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import trial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    args.output.mkdir(parents=True, exist_ok=True)
    original = ServoProfile.acceleration_limit
    cases = [(f'stop-{s}', s, 11.1, 1.) for s in (.3, .8, 1.5, 4.)]
    cases += [('voltage-10.5', 8., 10.5, 1.), ('voltage-12.6', 8., 12.6, 1.),
              ('assumed-accel-half', 8., 11.1, .5), ('assumed-accel-double', 8., 11.1, 2.)]
    results = {}
    for name, seconds, voltage, scale in cases:
        # Sensitivity only: the conversion from register to dynamics is unmeasured.
        # This does not raise hardware caps or select a faster model for adoption.
        context = patch.object(ServoProfile, 'acceleration_limit',
                               lambda servo: original(servo)*scale) if scale != 1 else nullcontext()
        with context:
            summary, rows = trial(config['j1_mode'], config['trajectory'], seconds, voltage)
        summary['hypothetical_acceleration_scale'] = scale
        with gzip.open(args.output/(name+'.json.gz'), 'wt', encoding='utf-8') as f:
            json.dump(dict(summary=summary, rows=rows), f)
        results[name] = summary
        print(json.dumps({name: {k: summary[k] for k in (
            'completed', 'first_fault_s', 'max_walk_tilt_deg', 'max_stop_tilt_deg',
            'final_s_error_deg')}}), flush=True)
    assert ServoProfile.acceleration_limit is original
    (args.output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
