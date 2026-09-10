"""Plot measured paths; actual lateral distance is explicitly in centimetres."""
import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).with_name('diagnostics')/'right-drift'
fig,axes=plt.subplots(2,1,figsize=(10,8))
for file,label in [('bias0.0-h0-b0-seed55.json','Symmetric / IMU corrections off'),('bias-0.6-h0-b0-seed55.json','Right J2 zero -0.6 deg / corrections off'),('bias-0.6-h1-b0-seed55.json','Same offset / heading hold on')]:
 data=json.loads((ROOT/file).read_text());r=[x for x in data['frames'] if 2<=x['time_s']<=32];a=r[0];theta=math.radians(a['yaw_deg'])
 dx=np.array([x['x']-a['x'] for x in r]);dy=np.array([x['y']-a['y'] for x in r]);forward=dx*math.cos(theta)+dy*math.sin(theta);right=dx*math.sin(theta)-dy*math.cos(theta)
 yaw=np.degrees(np.unwrap(np.radians([x['yaw_deg'] for x in r])));yaw=-(yaw-yaw[0])
 axes[0].plot(forward,right*100,label=label);axes[1].plot([x['time_s']-2 for x in r],yaw,label=label)
axes[0].set(xlabel='Forward travel (m)',ylabel='Right deviation (cm)',title='Measured path — lateral scale expanded, not equal aspect')
axes[1].set(xlabel='Time since forward command (s)',ylabel='Rightward heading change (deg)')
for ax in axes:ax.axhline(0,color='gray',linewidth=.8);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('30s straight command, yaw command = 0 / estimated MuJoCo physics\nBody leveling off in all three traces; seed 55')
fig.tight_layout();fig.savefig(ROOT/'right-drift-comparison.png',dpi=140)
