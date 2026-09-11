import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from check_body_height import OUT
frames=json.loads((OUT/'sync-cause-baseline-frames.json').read_text())
f=min(frames,key=lambda f:abs(f['time_s']-9.98))
rows=list(csv.DictReader((OUT/'diagonal-preload-1-sync.csv').open()))
lookup={(float(r['time_s']),r['leg']):float(r['clearance_mm']) for r in rows}
errors=[abs(x['world_gap_mm']-(lookup[(x['time_s'],'FL')]-lookup[(x['time_s'],'RR')])) for x in frames]
assert max(errors)<.2  # Recomputed FK vs recorded mesh/step state: audit separately.
assert max(abs(sum(x['gap_components_mm'])-x['world_gap_mm']) for x in frames)<1e-8
cases=['baseline','no_attitude','no_height','no_preload','no_heading','world_swing']
labels=['Current','No attitude correction','No height correction','No load preload','No heading correction','World swing correction*']
values=[json.loads((OUT/f'sync-cause-{x}.json').read_text())['gap_rms_mm'] for x in cases]
fig,ax=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
a=f['gap_components_mm'];names=['Nominal path','Feedback + preload','Command filter','Joint tracking','Torso orientation']
ax[0].barh(names,a,color=['#8997a6']*4+['#d55e00']);ax[0].invert_yaxis();ax[0].set_xlim(-1,30)
for i,v in enumerate(a):ax[0].text(max(v,0)+.3,i,f'{v:+.2f}',va='center')
ax[0].set_xlabel('Contribution to FL minus RR height (mm)');ax[0].set_title('FK height-gap decomposition at 9.98 s')
ax[1].barh(labels,values,color=['#d55e00']+['#0072b2']*4+['#8997a6']);ax[1].invert_yaxis();ax[1].set_xlim(0,16)
for i,v in enumerate(values):ax[1].text(v+.2,i,f'{v:.2f}',va='center')
ax[1].set_xlabel('Mid-swing FL–RR gap RMS (mm)');ax[1].set_title('One change per free-body simulation')
fig.suptitle('Root cause: torso rotation changes ground-frame foot heights\n*World-swing trial: 13.9 mm IK residual with J1 held; not adopted',fontsize=13)
fig.savefig(OUT/'sync-cause.png',dpi=150)
print('recomputed FK vs cached simulation clearance: max difference',max(errors),'mm')
