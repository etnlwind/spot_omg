"""Bounded, isolated gain comparison. Does not modify production settings."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
SIM = ROOT / 'simulation/mujoco'
PAIRS = [(0.1, 0.01), (0.1, 0.05), (0.3, 0.01), (0.3, 0.05), (0.5, 0.015), (0.5, 0.05)]


def source_hashes():
    files = [*sorted((ROOT / 'firmware/stm32-learning/Inc').glob('*.h')),
             *sorted(SIM.glob('*.py')), *sorted(SIM.glob('*.c')),
             SIM / 'cad_300mm/physics_parameters.json', SIM / 'cad_300mm/gait_scene.xml',
             SIM / 'foot_cushion_10mm.json', ROOT / 'config/body_stabilization.json',
             ROOT / 'config/locomotion_profiles.json',
             ROOT / 'tools/servo_tool/servo/gait_policy_host.c',
             ROOT / 'tools/servo_tool/servo/shared_gait.py', Path(__file__)]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def single(index, attempt):
    kp, kd = PAIRS[index]
    name = f'kp{kp:g}_kd{kd:g}_attempt{attempt}'
    destination = OUT / (name + '.json')
    before = source_hashes()
    sys.path.insert(0, str(SIM))
    from validate_body_stabilization import run
    base = json.loads((OUT / 'base-config.json').read_text())
    config = copy.deepcopy(base)
    config.update(kp_roll=kp, kp_pitch=kp, kd_roll_s=kd, kd_pitch_s=kd)
    assert config['max_foot_offset_m'] == 0.005
    result = run(True, destination, config, duration=30, linear=1., yaw=0.)
    after = source_hashes()
    changed = [p for p in before if before[p] != after[p]]
    observed = result['summary']['observed']
    metadata = dict(name=name, index=index, attempt=attempt, kp=kp, kd_s=kd,
                    source_hashes_before=before, source_hashes_after=after,
                    source_changed_during_run=bool(changed), changed_sources=changed,
                    full_window=(observed.get('samples') == 1000 and observed.get('start_s') == 10
                                 and observed.get('end_s') == 30 and result['summary']['fault'] is None),
                    summary=result['summary'])
    destination.with_suffix('.meta.json').write_text(json.dumps(metadata, indent=2))
    print('COMPLETE', name, 'source_changes', changed, 'full_window', metadata['full_window'], flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--single', type=int)
    parser.add_argument('--attempt', type=int, default=1)
    args = parser.parse_args()
    if args.single is not None:
        single(args.single, args.attempt)
        return
    base = json.loads((ROOT / 'config/body_stabilization.json').read_text())
    (OUT / 'base-config.json').write_text(json.dumps(base, indent=2))
    off = ROOT / 'artifacts/audits/body-pd-2026-09-14/off.json'
    reference = json.loads(off.read_text())
    (OUT / 'off-reference-summary.json').write_text(json.dumps(dict(
        path=str(off), sha256=hashlib.sha256(off.read_bytes()).hexdigest(),
        summary=reference['summary'], config=reference['config'], gait=reference['gait']), indent=2))
    manifest = dict(pairs=PAIRS, duration_s=30, window_s=[10, 30], required_samples=1000,
                    max_foot_offset_m=.005, source_hashes_start=source_hashes(), completed=[])
    for i, (kp, kd) in enumerate(PAIRS):
        for attempt in (1, 2):
            name = f'kp{kp:g}_kd{kd:g}_attempt{attempt}'
            subprocess.run([sys.executable, '-u', __file__, '--single', str(i), '--attempt', str(attempt)], check=True)
            metadata = json.loads((OUT / (name + '.meta.json')).read_text())
            manifest['completed'].append(metadata)
            (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2))
            if not metadata['source_changed_during_run']:
                break


if __name__ == '__main__':
    main()
