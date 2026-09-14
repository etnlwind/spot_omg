
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=REPO_ROOT/'artifacts/upright/2026-09-11/cushion'
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
for ax,short,gain in zip(axes,('v2','push'),('2.0','1.0')):
    for name,label in [(f'height-{short}-before','Before'),(f'height-{short}-feedback{gain}','Height hold')]:
        rows=[r for r in json.loads((P/(name+'-height.json')).read_text()) if r['time_s']>=5]
        ax.plot([r['time_s'] for r in rows],[1000*r['body_height_m'] for r in rows],label=label)
    ax.axhline(rows[0]['target_m']*1000,linestyle='--',color='grey',label='Constant reference')
    ax.set_title('V2' if short=='v2' else 'V2 propulsion timing');ax.set_ylabel('Rigid torso height (mm)')
    ax.set_ylim(242,274);ax.grid(alpha=.25);ax.legend(ncol=3)
axes[-1].set_xlabel('Simulation time (s)');fig.suptitle('Measured torso height - free-body physics, no body pinning')
fig.savefig(P/'body-height-comparison.png',dpi=150)
