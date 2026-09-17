"""Plot recorded evidence; no simulation control or hardware access."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'artifacts/imu-trace-v77/j2-horizontal'

def main():
    old=json.loads((OUT/'causal-ablation/joint85-104.json').read_text())['rows']
    new=json.loads((OUT/'long-validation/height45-duty70.json').read_text())['rows']
    failure=[r for r in old if 4.1<=r['time_s']<=4.72]
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    ax=axes[0,0]
    ax.plot([r['time_s'] for r in old],[max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in old],label='85-degree candidate before fix',color='#be3b39')
    ax.plot([r['time_s'] for r in new],[max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in new],label='Lower body + support overlap',color='#167052')
    ax.set(xlim=(0,36),ylim=(0,20),xlabel='Video time (s)',ylabel='Max |roll|, |pitch| (deg)',title='30 s walking + return to S, protection enabled')
    ax.axvline(32,ls=':',color='gray');ax.legend(fontsize=8)
    ax=axes[0,1]
    for i,name in enumerate(['FL','FR','RL','RR']):
        ax.plot([r['time_s'] for r in failure],[r['loads_n'][i] for r in failure],label=name)
    ax.set(xlabel='Original video time (s)',ylabel='Foot normal force (N)',title='FR/RL unload before the fall grows');ax.legend(ncol=4,fontsize=8)
    ax=axes[1,0]
    for i,name in [(1,'FR'),(2,'RL')]:
        ax.plot([r['time_s'] for r in failure],[r['clearance_mm'][i] for r in failure],label=name)
    ax.axhline(0,color='gray',lw=.7);ax.set(xlabel='Original video time (s)',ylabel='Ground clearance (mm)',title='RL recontacts after body rotation has grown');ax.legend()
    ax=axes[1,1]
    ax.plot([r['time_s'] for r in failure],[abs(r['support_line_distance_mm']) for r in failure],label='COM to support center line (mm)',color='#7f5199')
    ax.plot([r['time_s'] for r in failure],[abs(r['roll_deg']) for r in failure],label='Absolute roll (deg)',color='#be3b39')
    ax.set(xlabel='Original video time (s)',title='Support mismatch grows while both swing feet are clear');ax.legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('MuJoCo evidence only: estimated physical parameters; foot-clearance audit still fails',fontsize=12)
    fig.savefig(OUT/'balance-cause-and-improvement.png',dpi=150);plt.close(fig)

if __name__=='__main__':main()
