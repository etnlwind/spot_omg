"""Offline A/B of periodic J1 correction versus the requested fixed adduction.

Keep the FR-first angular entry sequence, X/Z targets, coupled J2/J3 recovery,
servo limits, collisions and Stop placements. Do not alter registered models.
"""
import argparse
from contextlib import contextmanager
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import numpy as np
from simulation.mujoco.runtime.s_native_gait import SNativeGait
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_s_native_hesitation import trial as observe


@contextmanager
def j1_experiment(mode):
    """Constrain the final candidate targets; restore methods on exit."""
    original_targets=SNativeGait.fr_entry_targets
    targets=original_targets
    support=SNativeGait.support_displacement
    def fixed_targets(gait,phase,amplitude,linear,yaw):
        result=targets(gait,phase,amplitude,linear,yaw)
        if (not gait.profile.get('uniform_recovery') or gait.stop_progress is not None
                or amplitude<=1e-8 or abs(linear)+abs(yaw)<=1e-8):
            return result
        # FR alone goes to 2x on step one; all reach 1x by step two.
        # Remove only the periodically recomputed normal-J1 residual.
        locked=gait.standing[::3]+gait.placement_fraction*gait.normal_adduction
        points=gait.last_target_points.copy()
        result,error=gait.kin.solve_xz(points,result,locked,iterations=60)
        if error>.0002:raise ValueError('Fixed J1 target unreachable')
        gait.previous=result.copy()
        gait.last_target_points=np.array([gait.kin.foot(i) for i in range(4)])
        return result
    def without_support(gait,phase,linear,yaw):
        return 0. if gait.profile.get('uniform_recovery') else support(gait,phase,linear,yaw)
    if mode not in ('baseline','fixed_j1','without_lateral_transfer'):raise ValueError(mode)
    if mode=='fixed_j1':SNativeGait.fr_entry_targets=fixed_targets
    elif mode=='without_lateral_transfer':SNativeGait.support_displacement=without_support
    try:
        yield
    finally:
        SNativeGait.fr_entry_targets=original_targets
        SNativeGait.support_displacement=support


def trial(mode,trajectory,seconds,voltage=11.1):
    # Constrain J1 after the coupled recovery, preserving its final X/Z goals.
    with tuck_experiment(**trajectory),j1_experiment(mode):
        summary,rows=observe(profile='s_native_v6_2_6',floor=True,seconds=seconds,voltage=voltage)
    chosen=[r for r in rows if 4<=r['time_s']<2+seconds and r['safety']=='ok'
            and r['motion'] and not r['stopping'] and r['entry_phase']>=1]
    summary['j1_mode']=mode
    summary.update(trajectory=trajectory,requested_walk_seconds=seconds,
        max_walk_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows if 2<=r['time_s']<2+seconds),
        max_stop_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows if r['time_s']>=2+seconds))
    if chosen:
        roll=np.array([r['roll_deg'] for r in chosen])
        pitch=np.array([r['pitch_deg'] for r in chosen])
        summary.update(steady_sample_seconds=len(chosen)*.02,
            steady_roll_rms_deg=float(np.sqrt(np.mean(roll**2))),
            steady_roll_peak_deg=float(np.max(abs(roll))),
            steady_pitch_rms_deg=float(np.sqrt(np.mean(pitch**2))),
            steady_j1_target_range_deg=np.ptp(np.array([r['target'][::3] for r in chosen]),axis=0).tolist(),
            steady_j1_actual_range_deg=np.ptp(np.array([r['actual'][::3] for r in chosen]),axis=0).tolist(),
            steady_j1_target_mean_deg=np.mean(np.array([r['target'][::3] for r in chosen]),axis=0).tolist())
    return summary,rows


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('config/experiments/post_push_j2_j3_recovery.json'))
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--modes',nargs='+',default=['baseline','without_lateral_transfer','fixed_j1'])
    args=parser.parse_args();config=json.loads(args.config.read_text())
    args.output.mkdir(parents=True,exist_ok=True)
    reports=[]
    for mode in args.modes:
        summary,rows=trial(mode,config['trajectory'],args.seconds,config.get('pack_open_circuit_voltage',11.1))
        with gzip.open(args.output/f'{mode}.json.gz','wt',encoding='utf-8') as f:
            json.dump(dict(summary=summary,rows=rows),f)
        reports.append(summary)
        print(json.dumps({k:v for k,v in summary.items() if 'excursion' not in k}),flush=True)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
