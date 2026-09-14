exec(open('artifacts/audits/sway-2026-09-14/trajectory_stages.py').read().split('results={}')[0])
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
f=ROOT/'artifacts/gait-videos/2026-09-14/shoulder-j2-highlift32-mass2754g-four-views.json';d=json.loads(f.read_text());rows=[r for r in d['rows'] if 12<=r['time_s']<=14.7];goals=np.array(d['geometry']['foot_goals_m'])
fig,axes=plt.subplots(3,2,figsize=(13,9),sharex=True)
for j,field in enumerate(('nominal_deg','command_deg','actual_deg')):
 a=[]
 for r in rows:
  kin.set_angles(r[field]);a.append([kin.foot(i) for i in range(4)])
 a=(np.array(a)-goals)*1000
 for c,axis in enumerate((0,2)):
  for leg,label,color in ((1,'FR front','#1688dc'),(2,'RL rear','#d94399')):axes[j,c].plot([r['time_s'] for r in rows],a[:,leg,axis],label=label,color=color)
  axes[j,c].set_title(field+' | '+('forward X' if axis==0 else 'vertical Z'));axes[j,c].set_ylabel('Displacement (mm)');axes[j,c].grid(alpha=.25);axes[j,c].legend()
axes[2,0].set_xlabel('Time (s)');axes[2,1].set_xlabel('Time (s)');fig.suptitle('Same-phase diagonal comparison | body-fixed CAD frame | mass 2.754 kg\nZ here is relative to neutral, not ground clearance')
fig.tight_layout();fig.savefig(ROOT/'artifacts/audits/sway-2026-09-14/trajectory-stages.png',dpi=150)
