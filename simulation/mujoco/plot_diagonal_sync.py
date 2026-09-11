"""Plot measured foot heights and support mismatch alongside torso attitude."""
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from check_body_height import OUT
name='diagonal-preload-1'
rows=list(csv.DictReader((OUT/(name+'-sync.csv')).open()))
a=[r for r in rows if r['leg']=='FL' and 8<=float(r['time_s'])<=10.6]
b=[r for r in rows if r['leg']=='RR' and 8<=float(r['time_s'])<=10.6]
t=np.array([float(r['time_s']) for r in a]);ha=np.array([float(r['clearance_mm']) for r in a]);hb=np.array([float(r['clearance_mm']) for r in b])
ca=np.array([float(r['force_n'])>.2 for r in a]);cb=np.array([float(r['force_n'])>.2 for r in b])
fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True,gridspec_kw={'height_ratios':[2,1,1.3]},layout='constrained')
axes[0].plot(t,ha,label='FL clearance',color='#009e73',lw=2);axes[0].plot(t,hb,label='RR clearance',color='#cc79a7',lw=2)
axes[0].axhline(0,color='black',lw=.8);axes[0].set_ylabel('Ground clearance (mm)');axes[0].legend(loc='upper left')
axes[0].annotate('RR touches at 9.98s\nFL touches at 10.18s',xy=(9.98,0),xytext=(9.75,18),arrowprops={'arrowstyle':'->'})
axes[1].step(t,ca.astype(float),where='post',label='FL',color='#009e73');axes[1].step(t,cb.astype(float)-1.5,where='post',label='RR',color='#cc79a7')
axes[1].set_yticks([-1.5,-.5,0,1],['RR air','RR contact','FL air','FL contact'])
axes[2].plot(t,[float(r['roll_deg']) for r in a],label='Roll',color='#0072b2');axes[2].plot(t,[float(r['pitch_deg']) for r in a],label='Pitch',color='#e69f00')
axes[2].set_ylabel('Torso angle (deg)');axes[2].legend(loc='upper left');axes[2].set_xlabel('Simulation time (s)')
for ax in axes:
 ax.grid(alpha=.2)
 ax.fill_between(t,0,1,where=ca!=cb,color='#e69f00',alpha=.15,transform=ax.get_xaxis_transform())
fig.suptitle('Same diagonal command, different actual foot clearance and contact\nShown-video configuration | shaded: only one foot contacts | 20 ms samples')
fig.savefig(OUT/'diagonal-sync.png',dpi=150)
summary=json.loads((OUT/(name+'-sync.json')).read_text());events=[e for e in summary['events'] if e['pair']=='FL-RR']
print('Median absolute touchdown gap (ms)',np.median([abs(e['touchdown_s_rear_minus_front_ms']) for e in events]))
