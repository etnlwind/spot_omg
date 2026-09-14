"""Summarize full, equal measurement windows without truncating failed runs."""
import hashlib
import json
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]


def metrics(records, end=30):
    rows = [r for r in records if 10 <= r['t'] < end]
    a = lambda k: np.asarray([r[k] for r in rows])
    angle = a('roll_pitch')
    return dict(samples=len(rows), start_s=10, end_s=end,
                roll_pitch_pp_deg=np.ptp(angle, axis=0).tolist(),
                roll_pitch_rms_deg=np.sqrt(np.mean(angle**2, axis=0)).tolist(),
                roll_pitch_max_deg=np.abs(angle).max(axis=0).tolist(),
                roll_pitch_mean_deg=angle.mean(axis=0).tolist(),
                roll_pitch_std_deg=angle.std(axis=0).tolist(),
                roll_pitch_min_deg=angle.min(axis=0).tolist(),
                roll_pitch_signed_max_deg=angle.max(axis=0).tolist(),
                gyro_xy_rms_deg_s=np.degrees(np.sqrt(np.mean(a('gyro_truth')[:, :2]**2, axis=0))).tolist(),
                forward_m=float(a('com')[-1, 0]-a('com')[0, 0]),
                tracking_max_deg=float(np.max(abs(a('command')-a('actual')))),
                correction_peak_mm=float(max(max(abs(x) for x in r['placement'].get('applied_dz', [0]*4)) for r in rows)*1000),
                all_safety_ok=all(r['safety']=='ok' for r in rows))


def values(m):
    return m['roll_pitch_pp_deg']+m['roll_pitch_rms_deg']+m['gyro_xy_rms_deg_s']+[m['tracking_max_deg']]


def entry(path, epoch, reference=None):
    data = json.loads(path.read_text())
    end = data['summary']['duration_s']
    m = metrics(data['records'], end)
    meta_path = path.with_suffix('.meta.json')
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    result = dict(name=path.stem, json=str(path), json_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  solver_epoch=epoch, kp=data['config']['kp_roll'], kd_s=data['config']['kd_roll_s'],
                  fixed_window_metrics=m, first20s_metrics=metrics(data['records'], 30),
                  fault=data['summary']['fault'], source_changed_during_run=meta.get('source_changed_during_run'),
                  source_metadata=str(meta_path) if meta else None,
                  full_forward_window=m['all_safety_ok'] and data['summary']['fault'] is None,
                  validated=False)
    if reference is not None:
        result['first20s_delta_percent_vs_off']=[100*(a/b-1) for a,b in zip(values(result['first20s_metrics']),values(reference))]
    return result


def table(entries, window='fixed_window_metrics'):
    lines=['| Condition | Roll P-P ° | Pitch P-P ° | Roll RMS ° | Pitch RMS ° | Gyro X RMS °/s | Gyro Y RMS °/s | Tracking max ° | Safety |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for e in entries:
        safety='complete' if e['full_forward_window'] else f"FAILED tilt at {e['fault']['t']:.2f}s"
        lines.append('| '+e['name']+' | '+' | '.join(f'{v:.3f}' for v in values(e[window]))+' | '+safety+' |')
    return lines


def main():
    off=entry(OUT.parent/'off.json','original OFF (PD bypassed)')
    baseline=off['fixed_window_metrics']
    original_on=entry(OUT.parent/'on.json','legacy IK',baseline)
    old=[entry(p,'legacy IK',baseline) for p in sorted(OUT.glob('kp*_attempt*.json')) if not p.name.endswith('.meta.json')]
    precise=[entry(p,'precise IK 1um + 5um cap margin',baseline) for p in sorted(OUT.glob('precise_kp*_attempt*.json')) if not p.name.endswith('.meta.json')]
    index=dict(metric_order=['roll_pp','pitch_pp','roll_rms','pitch_rms','gyro_x_rms','gyro_y_rms','tracking_max'],
               original_off=off, original_on=original_on, legacy_gain_sweep=old, precise_followup=precise,
               experiment_changed_production_config=False, auto_promoted=False,
               video_candidate=str(OUT/'precise_kp0.1_kd0.01_30s_attempt1.json'),
               note='Faulted entries are recomputed on the same full 10–30s window, including the post-fault state, and cannot qualify as successful 20s forward trials.')
    (OUT/'index.json').write_text(json.dumps(index,indent=2))
    lines=['# Attitude-PD bounded gain comparison','','This experiment did not change production settings or automatically promote a candidate. Its frozen base-config.json was kp=0.2, kd=0.03s on both axes; later parent integration choices are separate from this experiment. All runs retain the 5mm Cartesian limit, original gait, physics, safety and motor model.','',
           'Each 30s run stands for 10s, then requests full forward for 20s. The tables use exactly t∈[10,30), 1,000 samples. Faulted runs are evaluated over this same window (including their post-stop state) and marked failed; the raw validator summary instead stops at the fault, so its shorter-window values must not be substituted.','',
           '## Original six candidates: legacy IK','']
    lines+=table([off,original_on]+old)
    lines+=['','All six candidate runs had identical before/after source SHA snapshots. The three kd=0.05s runs tripped tilt safety at 24.06s, 27.02s and 27.44s; none completed the required forward duration. The legacy low-gain candidate improved RMS, gyro and tracking, but roll peak-to-peak increased versus OFF.','',
            '## Precise IK follow-up','',
            'The parent tightened the final Cartesian IK to 1µm and added a 5µm numerical margin inside the same 5mm cap. Old solver trials are preserved separately. These three runs also had unchanged before/after source SHA. PD OFF bypasses the adapter and preserves nominal targets.','']
    short=[e for e in precise if e['fixed_window_metrics']['end_s']==30]
    long=[e for e in precise if e['fixed_window_metrics']['end_s']==70]
    lines+=table([off]+short)
    lines+=['','60s forward follow-up (10–70s, 3,000 samples; not an equal-duration OFF comparison):','']+table(long)
    lines+=['','| Condition | Max roll / pitch ° | Forward m | Applied correction peak mm |','|---|---:|---:|---:|']
    for e in [off]+precise:
        m=e['fixed_window_metrics'];lines.append(f"| {e['name']} | {m['roll_pitch_max_deg'][0]:.3f} / {m['roll_pitch_max_deg'][1]:.3f} | {m['forward_m']:.3f} | {m['correction_peak_mm']:.3f} |")
    low=next(e for e in short if e['kp']==.1)
    lines+=['','The low-gain precise candidate (kp=0.1, kd=0.01s) completes both durations and improves five of the six angle/rate measures plus tracking versus the 20s OFF reference. Roll peak-to-peak worsens. It still fails the prior maximum-roll 3° and roll-RMS 1.5° goals, and remains unvalidated. The default kp=0.2, kd=0.03s after precise IK gives much larger excursions and tracking error.','',
            'Low-gain 20s deltas versus OFF, in table metric order: '+', '.join(f'{v:+.1f}%' for v in low['first20s_delta_percent_vs_off'])+'.','',
            'Mean and oscillation must be distinguished: OFF mean roll is '+f"{baseline['roll_pitch_mean_deg'][0]:.3f}° with standard deviation {baseline['roll_pitch_std_deg'][0]:.3f}°; low-gain precise mean roll is {low['fixed_window_metrics']['roll_pitch_mean_deg'][0]:.3f}° with standard deviation {low['fixed_window_metrics']['roll_pitch_std_deg'][0]:.3f}°. "+'A lower RMS relative to zero is therefore not proof that every swing amplitude decreased.','',
            'Machine-readable results and final JSON paths: `index.json`. Every tuning run has a `.meta.json` with source hashes and its unmodified raw JSON/CSV. Feedback uses delayed/quantized IMU and body gyro; simulator truth is evaluation-only. No hardware validation or phone installation was performed.','']
    (OUT/'TUNING.md').write_text('\n'.join(lines))
    print(json.dumps(dict(index=str(OUT/'index.json'),report=str(OUT/'TUNING.md'),low_gain_delta=low['first20s_delta_percent_vs_off']),indent=2))


if __name__=='__main__':main()
