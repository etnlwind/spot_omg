"""Replay recorded errors through the C governor; never predicts unmeasured motion."""
import argparse,csv,json,collections,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from servo import SharedGaitPolicy
from simulation.mujoco.runtime.gait_tracking import GaitTracking

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--responsive',action='store_true');a=p.parse_args()
    commands=list(csv.DictReader((a.source/'commands.csv').open()))
    samples=sorted(csv.DictReader((a.source/'samples.csv').open()),key=lambda r:int(r['end']))
    tracker=GaitTracking(SharedGaitPolicy());tracker.reset(0)
    errors=[0.]*12;stamps=[0]*12;index=0;last=0;rows=[]
    for command in commands:
        now=int(command['send_begin_ms'])
        while index<len(samples) and int(samples[index]['end'])<now:
            s=samples[index];index+=1
            if int(s['status']):continue
            j=int(s['joint']);errors[j]=float(s['error_deg']);stamps[j]=int(s['end'])
            tracker.lib.spot_tracking_sample(tracker.state,j,errors[j],stamps[j])
        dt=(now-last)/1000 if now else .02;last=now
        rate=tracker.step(now/1000,dt,False,a.responsive)
        j=max(range(12),key=lambda k:abs(errors[k]))
        rows.append(dict(time_ms=now,dt=dt,rate=rate,peak_joint=j,
                         peak_error=abs(errors[j]),peak_age_ms=now-stamps[j]))
    selected=[r for r in rows if 1000<=r['time_ms']<=4900]
    span=sum(r['dt'] for r in selected)
    summary=dict(window_ms=[1000,4900],responsive=a.responsive,
        mean_progress_rate=sum(r['dt']*r['rate'] for r in selected)/span,
        min_rate=min(r['rate'] for r in selected),
        time_fraction_below_80pct=sum(r['dt'] for r in selected if r['rate']<.8)/span,
        peak_joint_counts=dict(collections.Counter(r['peak_joint'] for r in selected)),
        mean_peak_age_ms=sum(r['peak_age_ms']*r['dt'] for r in selected)/span)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(summary=summary,rows=rows,limitations=[
        'Recorded errors only; not a faster-motion prediction.',
        'Frame time estimated by send timestamp; compute-to-send offset unavailable.']),indent=2)+'\n')
    print(json.dumps(summary))
if __name__=='__main__':main()
