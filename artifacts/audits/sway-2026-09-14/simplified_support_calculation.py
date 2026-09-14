"""Snapshot point-support statics; does not change or run a robot policy."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
source = json.loads((HERE / 'contact-geometry-decomposition.json').read_text())
row = next(r for r in source['records']
           if r['source'].endswith('causal-nocorrection.json') and abs(r['time_s'] - 10.8) < .005)
geometry = row['support_geometry']
C = np.array(geometry['com_body_rotated_m'])[:2]
A, B = np.array(geometry['feet_body_rotated_m'])[[1, 2], :2]
v = B - A
mg = source['mass_kg'] * 9.81
fraction = np.clip(np.dot(C - A, v) / np.dot(v, v), 0., 1.)
minimum = C - (A + fraction * v)
lambda_x = (C[1] - A[1]) / v[1]
lambda_y = (C[0] - A[0]) / v[0]
options = {
    'minimum_xy': minimum,
    'x_only': np.array([C[0] - A[0] - lambda_x * v[0], 0.]),
    'y_only': np.array([0., C[1] - A[1] - lambda_y * v[1]]),
}
results = {}
for name, shift in options.items():
    aa, bb = A + shift, B + shift
    f = np.dot(C - aa, v) / np.dot(v, v)
    forces = mg * np.array([1. - f, f])
    # For vertical forces at ground points, both horizontal moments vanish
    # exactly when this force-weighted support coordinate equals COM XY.
    residual = forces[0] * (aa - C) + forces[1] * (bb - C)
    assert 0 <= f <= 1 and np.all(forces > 0)
    assert abs(forces.sum() - mg) < 1e-10 and np.linalg.norm(residual) < 1e-10
    results[name] = dict(common_shift_mm=(shift * 1000).tolist(),
        distance_mm=float(np.linalg.norm(shift) * 1000),
        FR_after_mm=(aa * 1000).tolist(), RL_after_mm=(bb * 1000).tolist(),
        FR_force_N=float(forces[0]), RL_force_N=float(forces[1]),
        fraction_FR_to_RL=float(f), moment_balance_residual_Nm=float(np.linalg.norm(residual)))

out = dict(source='contact-geometry-decomposition.json: causal-nocorrection.json @ 10.80 s',
    mass_kg=source['mass_kg'], weight_N=mg,
    COM_mm=(C * 1000).tolist(), FR_before_mm=(A * 1000).tolist(), RL_before_mm=(B * 1000).tolist(),
    support_line_offset_mm=float(np.linalg.norm(minimum) * 1000),
    gravity_moment_proxy_Nm=float(mg * np.linalg.norm(minimum)), options=results,
    assumptions=['COM held at recorded position; CAD mass distribution remains estimated',
                 'Only FR/RL vertical point reactions; other two feet removed counterfactually',
                 'Zero linear acceleration and angular-momentum rate',
                 'No finite cushion patch, COP shift, friction, IK, or motor feasibility validation'],
    recorded_all_four_forces_N=row['force_n'],
    coordinate_frame='Recorded yaw removed; X forward, Y left, projected onto horizontal plane',
    status='snapshot_geometric_equilibrium_only_not_a_validated_gait')
(HERE / 'simplified-support-calculation.json').write_text(json.dumps(out, indent=2))

fig, ax = plt.subplots(figsize=(10, 6.5))
fig.subplots_adjust(left=.1, right=.98, top=.84, bottom=.24)
for name, color, label in [('original', '#b65c20', 'Recorded FR-RL support line'),
                         ('x_only', '#007cad', 'Candidate: both feet 46.54 mm backward')]:
    shift = np.zeros(2) if name == 'original' else options[name]
    points = (np.array([A, B]) + shift - C) * 1000
    ax.plot(points[:, 0], points[:, 1], 'o-', color=color, linewidth=2.2, markersize=8, label=label)
    for i, leg in enumerate(['FR', 'RL']):
        ax.annotate(leg, points[i], xytext=(5, -18 if i == 0 else 9), textcoords='offset points', color=color)
before = (np.array([A, B]) - C) * 1000
after = before + options['x_only'] * 1000
for p, q in zip(before, after):
    ax.annotate('', q, p, arrowprops=dict(arrowstyle='->', color='#505050', lw=1.8))
closest = -minimum * 1000
ax.plot([0, closest[0]], [0, closest[1]], ':', color='#b65c20', linewidth=2)
ax.annotate('19.91 mm offset', closest, xytext=(24, 18), textcoords='offset points', color='#8f4518')
ax.scatter([0], [0], color='#ca208d', s=80, zorder=5, label='COM ground projection (held fixed)')
ax.axhline(0, color='#999', linewidth=.7); ax.axvline(0, color='#999', linewidth=.7)
ax.set(xlabel='Forward +X (mm), relative to COM', ylabel='Left +Y (mm), relative to COM',
       title='Simplified diagonal support: 2.754 kg / 10.80 s snapshot\n'
             'Two point feet + fixed COM; acceleration and cushion contact area excluded')
ax.set_aspect('equal'); ax.set_ylim(-125, 125); ax.grid(alpha=.15)
ax.legend(loc='upper center', bbox_to_anchor=(.5, -.22), ncols=1, fontsize=9)
fig.savefig(HERE / 'simplified-support-calculation.png', dpi=160, bbox_inches='tight')
print(json.dumps(out, indent=2))
