"""Audit CAD recovery direction independently of any servo dynamics model."""
import argparse
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters, parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait, PROFILES
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import j1_experiment

LEGS = ('FL', 'FR', 'RL', 'RR')


def sample_geometry(plant, config, samples=501):
    """One steady cycle after the original FR entry; no physics integration."""
    with tuck_experiment(**config['trajectory']), j1_experiment(config['j1_mode']):
        gait = SNativeGait(plant.model, plant.stand_target,
                          copy.deepcopy(PROFILES['s_native_v6_2_6']))
        gait.prepare_support(1., 0.)
        for phase in np.linspace(.5, 2.5, 201, endpoint=False):
            gait.targets(phase % 1, 1., 1., 0.)
        rows = []
        for phase in np.linspace(2.5, 3.5, samples):
            angles = gait.targets(phase % 1, 1., 1., 0.)
            kin = gait.kin
            kin.set_angles(angles)
            feet = np.array([kin.foot(i) for i in range(4)])
            knee_angles, gradients = [], []
            for i, leg in enumerate(LEGS):
                kin.set_angles(angles)
                hip = kin.data.xanchor[plant.model.joint(leg.lower()+'_j2').id]
                knee = kin.data.xanchor[plant.model.joint(leg.lower()+'_j3').id]
                # Geometric proxy uses the cushion centre, not a claimed anatomical
                # inner angle or encoder zero. Its change tests flexion direction.
                upper = hip-knee
                lower = kin.data.geom_xpos[kin.feet[i]]-knee
                knee_angles.append(float(np.degrees(np.arccos(np.clip(
                    upper@lower/np.linalg.norm(upper)/np.linalg.norm(lower), -1., 1.)))))
                derivative = []
                for joint in (1, 2):
                    offset = angles.copy()
                    offset[3*i+joint] += .1
                    kin.set_angles(offset)
                    derivative.append(((kin.foot(i)-feet[i])*1000/.1).tolist())
                gradients.append(derivative)
            rows.append(dict(phase=phase % 1, cycle_fraction=float(phase-2.5), target_deg=angles.tolist(),
                             feet_from_s_mm=((feet-gait.origin)*1000).tolist(),
                             knee_proxy_deg=knee_angles,
                             xyz_mm_per_positive_degree_j2_j3=gradients))
        return rows


def summarize(rows):
    phase = (np.array([r['phase'] for r in rows])[:, None]+[.5, 0, 0, .5]) % 1
    feet = np.array([r['feet_from_s_mm'] for r in rows])
    angles = np.array([r['target_deg'] for r in rows])
    summaries = {}
    for leg, name in enumerate(LEGS):
        # The starting sample exactly at q=.5 is included. Exclude wrap at q=0.
        recovery = (phase[:-1, leg] >= .5) & (phase[1:, leg] >= .5)
        dx = np.diff(feet[:, leg, 0])
        bad = recovery & (dx < -.01)  # 10 um threshold excludes IK tolerance noise.
        summaries[name] = dict(
            reverse_travel_mm=float(-dx[bad].sum()),
            reverse_swing_fractions=((phase[:-1, leg][bad]-.5)*2).tolist(),
            j3_command_range_deg=[float(angles[:, leg*3+2].min()),
                                  float(angles[:, leg*3+2].max())],
            target_peak_clearance_mm=float(feet[:, leg, 2].max()))
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--configs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    plant = Simulation(load_parameters(parse_args([])))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    report = {}
    for path in args.configs:
        config = json.loads(path.read_text(encoding='utf-8'))
        rows = sample_geometry(plant, config)
        report[path.stem] = summarize(rows)
        (args.output/(path.stem+'.json')).write_text(
            json.dumps(dict(config=config, summary=report[path.stem], rows=rows)), encoding='utf-8')
        # RL follows the full swing over the first half of this cycle.
        chosen = [r for r in rows if r['cycle_fraction'] <= .5]
        u = np.array([r['cycle_fraction'] for r in chosen])*2
        feet = np.array([r['feet_from_s_mm'][2] for r in chosen])
        angles = np.array([r['target_deg'] for r in chosen])
        label = 'r3: preserve X' if config['trajectory'].get('tuck_projection') == 'vertical' else 'r2: independent pulses'
        axes[0].plot(u, feet[:, 0], label=label)
        axes[1].plot(u, feet[:, 2], label=label)
        axes[2].plot(u, angles[:, 7], label=label+' J2')
        axes[2].plot(u, angles[:, 8], '--', label=label+' J3')
    for ax, ylabel in zip(axes, ['RL X from S (mm)', 'RL target lift (mm)', 'CAD joint command (deg)']):
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.25)
        ax.legend()
    axes[-1].set_xlabel('Recovery fraction (0: scheduled push ends, 1: placement)')
    fig.suptitle('Recovery geometry only | no measured actuator dynamics')
    fig.tight_layout()
    fig.savefig(args.output/'recovery-geometry.png', dpi=150)
    plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({key: {leg: {k: v for k, v in values.items() if k != 'reverse_swing_fractions'}
                                  for leg, values in result.items()} for key, result in report.items()}))


if __name__ == '__main__':
    main()
