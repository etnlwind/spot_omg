"""Compare full paths; exaggerated lateral scale is explicit in the figure."""
import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).with_name('diagnostics')
fig,axes=plt.subplots(1,2,figsize=(12,4.6),sharey=True)
for balance,ax in enumerate(axes):
    for folder,heading,label,color in [
        ('right-drift',0,'Heading OFF','#d97612'),
        ('right-drift',1,'Previous heading hold','#777777'),
        ('heading-v24/30s-1000',1,'V24 heading hold','#167951')]:
        frames=json.loads((ROOT/folder/f'bias-0.6-h{heading}-b{balance}-seed55.json').read_text())['frames']
        a=frames[100];theta=math.radians(a['yaw_deg']);frames=frames[100:1601]
        dx=np.array([r['x']-a['x'] for r in frames]);dy=np.array([r['y']-a['y'] for r in frames])
        ax.plot(dx*math.cos(theta)+dy*math.sin(theta),100*(dx*math.sin(theta)-dy*math.cos(theta)),label=label,color=color,lw=1.5)
    ax.axhline(0,color='black',lw=.6);ax.grid(alpha=.2)
    ax.set_title('Body leveling '+('ON' if balance else 'OFF'));ax.set_xlabel('Forward travel (m)');ax.legend(fontsize=9)
axes[0].set_ylabel('Rightward deviation (cm)')
fig.suptitle('30s straight command / right J2 offset -0.6 deg / seed 55\nEstimated physics; lateral scale expanded, not equal aspect')
fig.tight_layout()
fig.savefig(ROOT/'heading-v24/comparison.png',dpi=150)
