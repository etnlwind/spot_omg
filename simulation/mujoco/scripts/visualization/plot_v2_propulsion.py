"""Measured contact-force comparison, with explicitly labeled smoothing."""

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
for stem,label in [('v2-push-0','V2 before'),('v2-push-pulse10-phase80','Shared push timing')]:
    rows=json.loads((P/(stem+'-forces.json')).read_text());t=np.array([r['time_s'] for r in rows])
    for ax,key in zip(axes,('forward_force_n','forward_speed_m_s')):
        raw=np.array([r[key] for r in rows]);filtered=np.convolve(raw,np.ones(5)/5,mode='same')
        ax.plot(t,filtered,label=label)
for ax in axes:
    ax.axvspan(6,7,color='grey',alpha=.12);ax.axhline(0,color='black',linewidth=.7)
    ax.grid(alpha=.25);ax.legend();ax.set_xlim(5.5,7.5)
axes[0].set_ylabel('Net foot force forward (N)');axes[1].set_ylabel('Measured forward speed (m/s)');axes[1].set_xlabel('Simulation time (s)')
fig.suptitle('Actual free-body MuJoCo: 100ms moving average\n6-7s: net force -0.440 to -0.126 N; speed 0.0371 to 0.0455 m/s')
fig.savefig(P/'v2-push-force-speed.png',dpi=150)
