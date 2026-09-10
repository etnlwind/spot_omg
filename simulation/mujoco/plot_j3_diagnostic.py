"""Render recorded front-left joint timing without rerunning physics."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).with_name('diagnostics')/'j3-lift'
rows=json.loads((root/'baseline.json').read_text())['frames'];rows=[r for r in rows if 8<=r['time_s']<=10.8]
t=np.array([r['time_s'] for r in rows]);fig,axes=plt.subplots(5,1,figsize=(12,10),sharex=True)
for ax,j,label in [(axes[0],1,'J2 (higher torque)'),(axes[1],2,'J3 (knee)')]:
    for field,name,style in [('nominal_deg','Planned','--'),('command_deg','Command','-'),('actual_deg','Actual','-')]:ax.plot(t,[r[field][j] for r in rows],style,label=name)
    ax.set_ylabel(label+'\nangle (deg)');ax.legend(loc='upper right',ncol=3)
axes[2].plot(t,[r['clearance_mm'][0] for r in rows],label='Actual foot clearance')
axes[2].fill_between(t,0,15,where=[('fl_foot' in r['contacts']) for r in rows],alpha=.15,label='Measured contact')
axes[2].set_ylabel('Clearance (mm)');axes[2].legend(loc='upper right')
for j in (1,2):axes[3].plot(t,[r['torque_nm'][j] for r in rows],label=f'J{j+1} torque')
axes[3].set_ylabel('Torque (Nm)');axes[3].legend(loc='upper right')
for j in (1,2):axes[4].plot(t,[r['correction_deg'][j] for r in rows],label=f'J{j+1} IMU correction')
axes[4].set_ylabel('Correction (deg)');axes[4].set_xlabel('Simulation time (s)');axes[4].legend(loc='upper right')
for ax in axes:
    ax.grid(alpha=.2)
    ax.fill_between(t,0,1,where=[r['phase']>=.64 for r in rows],transform=ax.get_xaxis_transform(),color='orange',alpha=.10)
fig.suptitle('Level15 / front-left leg — orange: scheduled swing\nEstimated physics, delayed BNO055, unchanged motor limits')
fig.tight_layout();fig.savefig(root/'joint-timing.png',dpi=150)
