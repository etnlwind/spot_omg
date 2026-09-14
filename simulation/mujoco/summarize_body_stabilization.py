"""Reproducible full-window PD comparison and scientific plot."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from validate_body_stabilization import metrics,ROOT

def summarize():
    folder=ROOT/'artifacts/audits/body-pd-2026-09-14'
    paths=[folder/'final-off-70s.json',folder/'tuning/precise_kp0.1_kd0.01_70s_attempt1.json']
    trials=[json.loads(p.read_text()) for p in paths]
    off,on=[metrics(t['records'],10,70) for t in trials]
    nominal=[np.array([r['nominal'] for r in t['records']]) for t in trials]
    assert nominal[0].shape==nominal[1].shape and np.array_equal(*nominal)
    summary=dict(window_s=[10,70],samples=3000,nominal_targets_identical=True,
        off=off,on=on,faults=[t['summary']['fault'] for t in trials],
        max_3deg_rms_1_5deg_pass=bool(max(on['roll_pitch_max_deg'])<=3 and max(on['roll_pitch_rms_deg'])<=1.5),
        sources=[str(p.relative_to(ROOT)) for p in paths],config=trials[1]['config'])
    (folder/'final-comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
    table=['| Metric (60 seconds forward) | OFF | ON | Change |','|---|---:|---:|---:|']
    for key,names in [('roll_pitch_rms_deg',['Roll RMS (deg)','Pitch RMS (deg)']),
                      ('roll_pitch_pp_deg',['Roll peak-to-peak (deg)','Pitch peak-to-peak (deg)']),
                      ('gyro_xy_rms_deg_s',['Gyro X RMS (deg/s)','Gyro Y RMS (deg/s)']),
                      ('roll_pitch_max_deg',['Max absolute roll (deg)','Max absolute pitch (deg)'])]:
        for i,name in enumerate(names):
            a,b=off[key][i],on[key][i];table.append(f'| {name} | {a:.3f} | {b:.3f} | {(b/a-1)*100:+.1f}% |')
    a,b=off['tracking_max_deg'],on['tracking_max_deg'];table.append(f'| Max joint tracking error (deg) | {a:.3f} | {b:.3f} | {(b/a-1)*100:+.1f}% |')
    (folder/'final-comparison.md').write_text('\n'.join(table)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(14,7),sharex=True,layout='constrained')
    for trial,label,color in zip(trials,['OFF','ON Kp .1 / Kd .01s'],['#7b8794','#007f9e']):
        rows=[r for r in trial['records'] if 10<=r['t']<70];t=np.array([r['t']-10 for r in rows])
        for ax,key,index,unit in [(axes[0,0],'roll_pitch',0,'deg'),(axes[0,1],'roll_pitch',1,'deg'),
                                  (axes[1,0],'gyro_truth',0,'deg/s'),(axes[1,1],'gyro_truth',1,'deg/s')]:
            values=np.array([r[key][index] for r in rows])
            if key=='gyro_truth':values=np.degrees(values)
            ax.plot(t,values,color=color,alpha=.7,lw=.65,label=label);ax.set_ylabel(unit);ax.grid(alpha=.2)
    for ax,title in zip(axes.flat,['Roll','Pitch','Body gyro X (evaluation)','Body gyro Y (evaluation)']):ax.set_title(title)
    for ax in axes[0]:ax.axhline(3,color='#bc413c',lw=.6,ls='--');ax.axhline(-3,color='#bc413c',lw=.6,ls='--')
    for ax in axes[1]:ax.set_xlabel('Time since forward command (s)')
    axes[0,0].legend(loc='upper right')
    fig.suptitle('Spot OMG 2.754kg | identical nominal gait | full 60s forward | 50Hz position PD\nNo safety stops; roll peak-to-peak remains worse. Hardware gyro mapping not verified.')
    fig.savefig(folder/'final-comparison.png',dpi=150);plt.close(fig)
    print('\n'.join(table))

if __name__=='__main__':summarize()
