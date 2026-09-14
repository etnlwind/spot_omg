"""Plot recorded MuJoCo evidence; no kinematic replay or plant modification."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / 'causal-nocorrection.json').read_text())
rows = [r for r in data['records'] if 10.4 <= r['t'] <= 11.5]
t = np.array([r['t'] for r in rows])
command = np.array([r['filtered'] for r in rows]).reshape(-1, 4, 3)
actual = np.array([r['actual'] for r in rows]).reshape(-1, 4, 3)
clearance = np.array([r['clearance'] for r in rows])
fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True, layout='constrained')
pairs = [(0, 3, 'FL', 'RR'), (1, 2, 'FR', 'RL')]
for col, (front, rear, fn, rn) in enumerate(pairs):
    axes[0, col].set_title(f'{fn} + {rn}: diagonal pair', weight='bold')
    for joint, color in [(1, '#137cc1'), (2, '#d06220')]:
        label = f'J{joint + 1}'
        axes[0, col].plot(t, command[:, front, joint] - command[:, rear, joint],
                         color=color, label=label)
        axes[1, col].plot(t, actual[:, front, joint] - actual[:, rear, joint],
                         color=color, label=label)
    axes[0, col].set_ylim(-.1, .1)
    axes[0, col].set_ylabel('Filtered goal difference (deg)')
    axes[1, col].set_ylabel('Actual joint difference (deg)')
    axes[2, col].plot(t, clearance[:, front], color='#137cc1', label=fn)
    axes[2, col].plot(t, clearance[:, rear], color='#d06220', label=rn)
    axes[2, col].axhline(0, color='#777777', linewidth=1)
    axes[2, col].set_ylabel('Actual floor clearance (mm)')
    axes[2, col].set_xlabel('Recording time (s); forward starts at 10.0 s')
    for ax in axes[:, col]:
        ax.axvline(11, color='#777777', linestyle=':', linewidth=1)
        ax.grid(alpha=.18)
        ax.legend(loc='upper left', ncols=2)
fig.suptitle('Same diagonal servo goals do not guarantee simultaneous ground clearance\n'
             'MuJoCo | 2.754 kg | heading and balance corrections OFF | original 1.35 s cycle',
             fontsize=14)
fig.savefig(HERE / 'diagonal-sync-evidence.png', dpi=170)
print(HERE / 'diagonal-sync-evidence.png')
