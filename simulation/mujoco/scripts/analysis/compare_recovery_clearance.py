"""Bounded offline comparison of recovery lift and flexion timing."""
import argparse
import copy
import gzip
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
import numpy as np
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import trial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sweep = json.loads(args.sweep.read_text(encoding='utf-8'))
    base = json.loads((ROOT/sweep['base_config']).read_text(encoding='utf-8'))
    args.output.mkdir(parents=True, exist_ok=True)
    report = {}
    for name, overrides in sweep['cases'].items():
        config = copy.deepcopy(base)
        config['trajectory'].update(overrides)
        summary, rows = trial(config['j1_mode'], config['trajectory'], args.seconds,
                              config.get('pack_open_circuit_voltage', 11.1))
        # Screening only. Material contact slip is measured separately for survivors.
        chosen = [r for r in rows if 4 <= r['time_s'] < 2+args.seconds]
        feet = np.array([r['actual_feet_m'] for r in chosen])
        phase = (np.array([r['phase'] for r in chosen])[:, None]+[.5, 0, 0, .5]) % 1
        loaded = np.array([r['force_n'] for r in chosen]) > 1
        moving = np.diff(feet[:, :, 0], axis=0)/.02 > .02
        summary['loaded_forward_recovery_seconds'] = (
            ((phase[1:] >= .5) & loaded[1:] & moving).sum(axis=0)*.02).tolist()
        with gzip.open(args.output/(name+'.json.gz'), 'wt', encoding='utf-8') as f:
            json.dump(dict(config=config, summary=summary, rows=rows), f)
        report[name] = summary
        (args.output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps({name: {k: summary.get(k) for k in (
            'completed', 'first_fault_s', 'max_walk_tilt_deg', 'phase_hold_seconds',
            'mean_phase_rate', 'loaded_forward_recovery_seconds')}}), flush=True)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
