"""Render measured speed and planned contact timing, not animation-only claims."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from gait_profiles import load_profiles, PHASES
from pathlib import Path
RESULTS=Path(__file__).parent/"gait_search"/"profiles"


def main():
    data=json.loads((RESULTS/'validation.json').read_text())['nominal']
    names=['legacy','crawl','cruise','trot','highstep']
    colors=['#8290a4','#d4a454','#34a594','#4b83ce','#a978bd']
    fig,axes=plt.subplots(2,1,figsize=(10,7),gridspec_kw={'height_ratios':[1,1.3]},layout='constrained')
    speeds=[data[n]['speed_m_s'] for n in names]
    bars=axes[0].bar(names,speeds,color=colors)
    axes[0].bar_label(bars,labels=[f'{s:.3f}' for s in speeds],padding=3)
    axes[0].set_ylim(0,.24);axes[0].set_ylabel('Forward speed (m/s)')
    axes[0].set_title('Same CAD robot, same motors and 11.1 V supply — nominal 20 s trials')
    axes[0].spines[['top','right']].set_visible(False)
    profiles=load_profiles();row=0;labels=[]
    for name in ['crawl','cruise','trot','highstep']:
        p=profiles[name];duty=p['params'][1]
        for leg,offset in zip(['FL','FR','RL','RR'],PHASES[p['family']]):
            x=np.linspace(0,1,401);stance=((x+offset)%1)<duty
            axes[1].fill_between(x,row-.32,row+.32,where=stance,color=colors[names.index(name)],step='mid')
            labels.append(f'{name} / {leg}');row+=1
        row+=.6;labels.append('')
    yticks=[];r=0
    for _ in range(4):
        yticks.extend([r,r+1,r+2,r+3,r+4]);r+=4.6
    axes[1].set_yticks(yticks,labels);axes[1].invert_yaxis()
    axes[1].set_xlabel('Normalized gait cycle (filled = planned stance; gap = swing)')
    axes[1].set_title('Different footfall schedules; physics determines actual ground contact')
    axes[1].spines[['top','right']].set_visible(False)
    fig.savefig(RESULTS/'comparison.png',dpi=150)


if __name__=='__main__':main()
