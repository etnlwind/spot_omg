"""Compare recorded RR J3 steps, retaining timing bounds and static load error."""
import argparse
import csv
import json
import statistics as stats
from pathlib import Path

from servo.joint_trace import parse, signed_target, wrapped_error


def analyze_trial(path):
    meta, joints, commands, samples = parse((path/'jointtrace.txt').read_text())
    summary = json.loads((path/'summary.json').read_text())
    rr_all = [s for s in samples if s['joint'] == 11]
    rr = [s for s in rr_all if s['status'] == 0 and not s['hardware_error']
          and s['temperature_c'] < 70 and s['voltage_mv'] >= 10500]
    degree = 360/4096
    rows = []
    for s in rr:
        target = signed_target(commands[s['command']]['target'][11])
        rows.append(dict(time_ms=(s['begin']+s['end'])/2,
            relative_to_fold_ms=(s['begin']+s['end'])/2-commands[min(1,len(commands)-1)]['begin'],
            begin_ms=s['begin'], end_ms=s['end'],
            command=s['command'], target_deg=(target-joints[11]['center'])*joints[11]['direction']*degree,
            actual_deg=(s['raw']-joints[11]['center'])*joints[11]['direction']*degree,
            position_ticks=s['raw'], voltage_mv=s['voltage_mv'], load_raw=s['load_raw'],
            current_raw=s['current_raw'], temperature_c=s['temperature_c'], hardware_error=s['hardware_error']))
    baseline = [r for r in rows if r['command'] == 0]
    result = dict(path=str(path.resolve()), success=summary['success'], metadata=meta,
                  peak_temperature_c=max((s['temperature_c'] for s in rr_all if s['status']==0), default=None),
                  excluded_rr_samples=len(rr_all)-len(rr),
                  failed_reads=sum(s['status'] != 0 for s in samples), phases=[])
    gaps = [b['time_ms']-a['time_ms'] for a, b in zip(rows, rows[1:])]
    result['sampling'] = dict(median_gap_ms=stats.median(gaps) if gaps else None,
        max_gap_ms=max(gaps, default=None), max_transaction_width_ms=max((r['end_ms']-r['begin_ms'] for r in rows), default=None))
    for n in range(1, len(commands)):
        phase = [r for r in rows if r['command'] == n]
        preceding = [r for r in rows if r['command'] == n-1]
        if not phase or not preceding:
            continue
        initial = stats.median(r['position_ticks'] for r in preceding[-10:])
        finish = stats.median(r['position_ticks'] for r in phase[-10:])
        direction = 1 if commands[n]['target'][11] > commands[n-1]['target'][11] else -1
        first = next((r for r in phase if direction*(r['position_ticks']-initial) >= 3), None)
        before_first = [r for r in phase if first and r['end_ms'] < first['begin_ms'] and direction*(r['position_ticks']-initial) < 3]
        # Bounds on first observed 3-tick displacement, not motor latency: the
        # servo encoder update period is unknown, and there is quantization.
        bound = None
        if first:
            lower = max(0, (before_first[-1]['begin_ms'] if before_first else commands[n]['begin'])-commands[n]['end'])
            upper = first['end_ms']-commands[n]['begin']
            bound = [lower, upper]
        result['phases'].append(dict(command=n, direction='fold' if n==1 else 'return',
            commanded_delta_deg=(commands[n]['target'][11]-commands[n-1]['target'][11])*degree*joints[11]['direction'],
            observed_delta_deg=(finish-initial)*degree*joints[11]['direction'], first_3tick_displacement_bounds_ms=bound,
            final_goal_error_deg=wrapped_error(finish,commands[n]['target'][11])*degree*joints[11]['direction']))
    return result, rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trials', nargs='+', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    results=[];all_rows=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
    palette=['#0072B2','#E69F00','#009E73','#CC79A7','#D55E00']
    for trial_index,path in enumerate(a.trials):
        result, rows=analyze_trial(path);results.append(result)
        for row in rows: all_rows.append(dict(trial=path.name,**row))
        label=f"{path.name} | acc={result['metadata']['acceleration']}"
        times=[r['relative_to_fold_ms']/1000 for r in rows]
        color=palette[trial_index%len(palette)]
        axes[0].plot(times,[r['actual_deg'] for r in rows],label=label,color=color)
        axes[0].step(times,[r['target_deg'] for r in rows],where='post',linestyle='--',alpha=.35,color=color)
        axes[1].plot(times,[r['load_raw'] for r in rows],color=color)
        axes[2].plot(times,[r['voltage_mv']/1000 for r in rows],color=color)
    axes[0].set_ylabel('RR J3 mechanical angle (deg)');axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Servo load (raw)');axes[2].set_ylabel('Voltage (V)');axes[2].set_xlabel('MCU time relative to fold command (s)')
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle('RR J3 loaded floor observations | dashed: commanded\nNo fitted acceleration; incomplete trials retain their failure status')
    fig.tight_layout();fig.savefig(a.output/'response.png',dpi=150);plt.close(fig)
    (a.output/'analysis.json').write_text(json.dumps(dict(trials=results,
        limitation='MCU bus transaction bounds, not internal encoder timestamps. Small loaded step does not establish universal acceleration or maximum capability.'),indent=2),encoding='utf-8')
    if all_rows:
        with (a.output/'samples.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(all_rows[0]));w.writeheader();w.writerows(all_rows)
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
