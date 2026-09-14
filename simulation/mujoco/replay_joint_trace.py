"""Replay recorded *sent* joint commands in MuJoCo; compare at read timestamps.

No hardware I/O or automatic parameter fitting. Plant mass/contact/servo models
remain estimates. Replay discretization is 20ms, independent of sparse reads.
"""
import argparse,bisect,collections,json
from pathlib import Path
import numpy as np
import mujoco
from servo.joint_trace import parse,signed_target,DEG,analyze
from cad_physics import Simulation,foot_clearance
from search_gait_profiles import physics


def replay(path,output=None,parameter_overrides=None):
    meta,mapping,commands,samples=parse(path.read_text());observed,rows=analyze(path.read_text())
    params,_=physics('nominal')
    params.update(parameter_overrides or {})
    params['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    params['embedded_servo_quantization']=False # Already encoded and decoded on the physical path.
    plant=Simulation(params)
    targets=np.array([[(signed_target(c['target'][j])-mapping[j]['center'])*mapping[j]['direction']*DEG for j in range(12)] for c in commands])
    times=np.array([c['end'] for c in commands],float)
    initial=np.radians(targets[0]);plant.desired=initial.copy();plant.filtered=initial.copy();plant.target_velocity[:]=0
    plant.delay=collections.deque([initial.copy() for _ in range(round(params['command_delay_s']/.02))])
    plant.data.qpos[plant.q]=initial
    mujoco.mj_forward(plant.model,plant.data)
    feet=[plant.model.geom(f'{leg}_foot').id for leg in ('fl','fr','rl','rr')]
    plant.data.qpos[2]+=.001-min(foot_clearance(plant.model,plant.data,i) for i in feet)
    mujoco.mj_forward(plant.model,plant.data)
    trajectory=[(0.,np.degrees(plant.data.qpos[plant.q]).copy())]
    max_roll=max_pitch=0.
    for t in np.arange(0,max(s['end'] for s in samples)+20,20):
        index=max(0,bisect.bisect_right(times,t)-1)
        plant.step(targets_deg=targets[index],balance=False)
        row=plant.row();max_roll=max(max_roll,abs(row['roll_deg']));max_pitch=max(max_pitch,abs(row['pitch_deg']))
        trajectory.append((t+20,np.array(row['actual_deg'])))
    ts=np.array([v[0] for v in trajectory]);angles=np.array([v[1] for v in trajectory])
    comparison=[]
    for row in rows:
        j=row['joint'];pred=float(np.interp(row['time_ms'],ts,angles[:,j]))
        comparison.append({**row,'sim_actual_deg':pred,'sim_minus_hardware_deg':pred-row['actual_deg']})
    result=dict(revision=meta.get('revision'),input=str(path.resolve()),plant_mode='estimated replay; optional empirical parameter overrides',
        mass_kg=float(plant.model.body_mass.sum()),command_delay_s=params['command_delay_s'],
        replay_step_ms=20,excluded_startup_ms=1000,max_sim_roll_deg=max_roll,max_sim_pitch_deg=max_pitch,joints={})
    for j in range(12):
        part=[r for r in comparison if r['joint']==j and r['time_ms']>=1000]
        e=np.array([r['sim_minus_hardware_deg'] for r in part])
        result['joints'][f"{('FL','FR','RL','RR')[j//3]}-J{j%3+1}"]=dict(samples=len(part),prediction_rmse_deg=float(np.sqrt(np.mean(e*e))) if len(e) else None)
    if output is None:
        return result, comparison
    output.mkdir(parents=True,exist_ok=True)
    (output/'comparison.json').write_text(json.dumps(result,indent=2))
    import csv
    with (output/'comparison.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(comparison[0]));w.writeheader();w.writerows(comparison)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,2,figsize=(12,10),sharex=True)
    for leg in range(4):
        for column,joint in enumerate((1,2)):
            j=leg*3+joint;ax=axes[leg,column];part=[r for r in comparison if r['joint']==j]
            ax.step(times/1000,targets[:,j],where='post',color='gray',alpha=.6,label='Sent target')
            ax.plot(ts/1000,angles[:,j],color='#e78b2c',label='MuJoCo prediction')
            ax.plot([r['time_ms']/1000 for r in part],[r['actual_deg'] for r in part],'o',ms=4,color='#1874b5',label='Hardware samples')
            ax.set_title(f"{('FL','FR','RL','RR')[leg]} J{joint+1}");ax.set_ylabel('deg');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);axes[-1,0].set_xlabel('Seconds');axes[-1,1].set_xlabel('Seconds')
    fig.suptitle('Recorded commands replayed in MuJoCo: targets / prediction / hardware')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=150);plt.close(fig)
    return result, comparison
if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('trace',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--parameters',type=Path);a=p.parse_args();r,_=replay(a.trace,a.output,json.loads(a.parameters.read_text()) if a.parameters else None);print(json.dumps(r,indent=2))
