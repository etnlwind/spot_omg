"""Exercise the actual headless scene/controller/report path without hardware."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys


def test_neutral_drive_is_stationary_and_reports_joint_feedback(tmp_path):
    script = Path(__file__).with_name('drive_lab.py')
    subprocess.run([sys.executable, str(script), '--check', '--linear', '0',
                    '--duration', '1', '--output', str(tmp_path)],
                   check=True, capture_output=True, text=True, timeout=30)
    summary = json.loads((tmp_path / 'summary.json').read_text())
    assert summary['state'] == 'UPRIGHT'
    assert summary['samples'] >= 49
    assert abs(summary['displacement_m'][0]) < 0.01
    assert all(math.isfinite(value) for value in summary['displacement_m'])
    with (tmp_path / 'frames.csv').open() as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == summary['samples']
    for leg in ('FL', 'FR', 'RL', 'RR'):
        for joint in (1, 2, 3):
            for field in ('target_deg', 'actual_deg', 'torque_nm'):
                assert math.isfinite(float(rows[-1][f'{leg}_J{joint}_{field}']))


def test_drive_rejects_nonfinite_inputs_without_starting_simulation(tmp_path):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name('drive_lab.py')),
                             '--check', '--linear', 'nan', '--output', str(tmp_path)],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode != 0
    assert not (tmp_path / 'summary.json').exists()
