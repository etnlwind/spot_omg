"""Fit a bounded offline candidate from recorded roll; never approve a policy.

Input is evaluate_ground.py JSON. Re-run physics after every update: contact
changes make this iterative-learning approximation nonlinear and nonmonotonic.
The resulting coefficients use phase only at runtime, not simulation truth.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def fit(recording, gain=.25, start_s=18.):
    if not 0 < gain <= .5:
        raise ValueError('Learning gain must be within (0, .5]')
    rows = [r for r in recording['records'] if r['t'] >= start_s]
    config = dict(recording['config'])
    config.pop('duration_s', None)
    coefficients = np.asarray(config.get('roll_feedforward', np.zeros(8)), dtype=float)
    if coefficients.shape != (8,) or len(rows) < 200:
        raise ValueError('Expected eight coefficients and at least 200 samples')
    phase = np.asarray([r['phase'] for r in rows])
    matrix = np.column_stack([f(2 * np.pi * h * phase)
                              for h in (1, 3, 5, 7) for f in (np.sin, np.cos)])
    roll = np.radians([r['roll'] for r in rows])
    if not np.isfinite(np.r_[matrix.ravel(), roll, coefficients]).all():
        raise ValueError('Nonfinite input')
    residual = np.linalg.lstsq(matrix, roll, rcond=None)[0]
    proposed = coefficients + gain * residual
    # Conservative triangle bound for the complete periodic rotation.
    envelope = np.linalg.norm(proposed.reshape(4, 2), axis=1).sum()
    if envelope > .15:
        raise ValueError('Candidate may exceed the 0.15 rad rotation bound')
    config['roll_feedforward'] = proposed.tolist()
    return config


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--gain', type=float, default=.25)
    parser.add_argument('--start-s', type=float, default=18.)
    args = parser.parse_args()
    candidate = fit(json.loads(args.recording.read_text()), args.gain, args.start_s)
    args.output.write_text(json.dumps(candidate, indent=2) + '\n')
