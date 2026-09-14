"""Follow-up comparison after the parent's precise Cartesian IK fix."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

from run_tuning import OUT, SIM, source_hashes

CASES = [(0.1, 0.01, 30), (0.2, 0.03, 30), (0.1, 0.01, 70)]


def single(index, attempt):
    kp, kd, duration = CASES[index]
    name = f'precise_kp{kp:g}_kd{kd:g}_{duration}s_attempt{attempt}'
    destination = OUT / (name + '.json')
    before = source_hashes()
    sys.path.insert(0, str(SIM))
    from validate_body_stabilization import run, metrics
    base = json.loads((OUT / 'base-config.json').read_text())
    config = copy.deepcopy(base)
    config.update(kp_roll=kp, kp_pitch=kp, kd_roll_s=kd, kd_pitch_s=kd)
    assert config['max_foot_offset_m'] == 0.005
    result = run(True, destination, config, duration=duration, linear=1., yaw=0.)
    after = source_hashes()
    changed = [p for p in before if before[p] != after[p]]
    full = metrics(result['records'], 10, duration)
    metadata = dict(name=name, index=index, attempt=attempt, kp=kp, kd_s=kd,
                    duration_s=duration, forward_window_s=[10, duration],
                    solver='precise Cartesian IK 1um tolerance + 5um cap margin',
                    source_hashes_before=before, source_hashes_after=after,
                    source_changed_during_run=bool(changed), changed_sources=changed,
                    full_window=(full.get('samples') == (duration-10)*50 and result['summary']['fault'] is None),
                    full_window_metrics=full, summary=result['summary'])
    destination.with_suffix('.meta.json').write_text(json.dumps(metadata, indent=2))
    print('PRECISE COMPLETE', name, 'source_changes', changed, 'full_window', metadata['full_window'], flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--single', type=int)
    parser.add_argument('--attempt', type=int, default=1)
    args = parser.parse_args()
    if args.single is not None:
        single(args.single, args.attempt)
        return
    completed = []
    for i, (kp, kd, duration) in enumerate(CASES):
        for attempt in (1, 2):
            name = f'precise_kp{kp:g}_kd{kd:g}_{duration}s_attempt{attempt}'
            subprocess.run([sys.executable, '-u', __file__, '--single', str(i), '--attempt', str(attempt)], check=True)
            metadata = json.loads((OUT / (name + '.meta.json')).read_text())
            completed.append(metadata)
            (OUT / 'precise-manifest.json').write_text(json.dumps(completed, indent=2))
            if not metadata['source_changed_during_run']:
                break


if __name__ == '__main__':
    main()
