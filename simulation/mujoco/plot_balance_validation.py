"""Plot recorded A/B measurements; never alter simulation or sensor state."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).parent
rows=json.loads((root/'balance_validation.json').read_text())
labels=['Stand / roll','Stand / pitch','Walk-stop / roll','Walk-stop / pitch']
values=[]
for enabled in (False,True):
    values.append([abs(next(r for r in rows if r['enabled']==enabled and r['moving']==moving and r['axis']==axis and r['slope']==4)['final_deg']) for moving in (False,True) for axis in (0,1)])
fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained')
x=np.arange(4)
for offset,arr,label,color in ((-.18,values[0],'Feedback OFF','#9ca3af'),(.18,values[1],'BNO055 feedback ON','#168c82')):
    bars=ax.bar(x+offset,arr,.36,label=label,color=color)
    ax.bar_label(bars,fmt='%.2f°',padding=4)
ax.set_xticks(x,labels);ax.set_ylim(0,5.4)
ax.set_ylabel('Absolute body tilt at 18 s (degrees)')
ax.set_title('4° slope + 2 Nm disturbance · estimated robot physics')
ax.legend(frameon=False);ax.spines[['top','right']].set_visible(False)
fig.savefig(root/'balance_comparison.png',dpi=170)
