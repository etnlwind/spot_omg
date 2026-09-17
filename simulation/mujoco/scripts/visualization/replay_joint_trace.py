"""Replay recorded *sent* joint commands in MuJoCo; compare at read timestamps.

No hardware I/O or automatic parameter fitting. Plant mass/contact/servo models
remain estimates. Replay discretization is 20ms, independent of sparse reads.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import argparse,bisect,collections,json,re
from pathlib import Path
import numpy as np
import mujoco
from servo.joint_trace import parse,signed_target,DEG,analyze
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
from servo.imu_trace import parse as parse_imu
from simulation.mujoco.runtime.gait_evidence import foot_loads


def decode_targets(meta,mapping,commands):
    targets=np.array([[(signed_target(c['target'][j])-mapping[j]['center'])*mapping[j]['direction']*DEG for j in range(12)] for c in commands])
    native=meta.get('revision','').startswith('s-native-')
    if native:targets[:,[0,3]]*=-1
    return targets,native


def replay(path,output=None,parameter_overrides=None,imu_path=None,initial_attitude=False):
    meta,mapping,commands,samples=parse(path.read_text());observed,rows=analyze(path.read_text())
    params=load_parameters(parse_args([]))
    params.update(parameter_overrides or {})
    params['foot_cushion']=json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    params['embedded_servo_quantization']=False # Already encoded and decoded on the physical path.
    plant=Simulation(params)
    targets,native=decode_targets(meta,mapping,commands)
    if native:
        for row in rows:
            if row['joint'] in (0,3):
                for key in ('target_deg','actual_deg','error_deg'):row[key]*=-1
    imu=[]
    if imu_path:
        _,raw=parse_imu(imu_path.read_text())
        origin=int(re.search(r'\$JT,C,0,(\d+),',path.read_text())[1])
        imu=[dict(time_ms=((r[1]-origin+2**31)%2**32)-2**31,roll=r[2]/10,pitch=r[3]/10,phase=r[4]/1000) for r in raw if r[6]]
        if not imu or imu[-1]['time_ms']<0 or imu[0]['time_ms']>max(s['end'] for s in samples):raise ValueError('IMU capture does not overlap joint trace')
    times=np.array([c['end'] for c in commands],float)
    initial=np.radians(targets[0]);plant.desired=initial.copy();plant.filtered=initial.copy();plant.target_velocity[:]=0
    plant.delay=collections.deque([initial.copy() for _ in range(round(params['command_delay_s']/.02))])
    plant.data.qpos[plant.q]=initial
    if initial_attitude:
        if not imu:raise ValueError('Initial attitude requires a valid matching IMU capture')
        r,p=np.radians([imu[0]['roll'],imu[0]['pitch']])/2
        plant.data.qpos[3:7]=[np.cos(r)*np.cos(p),np.sin(r)*np.cos(p),np.cos(r)*np.sin(p),-np.sin(r)*np.sin(p)]
    mujoco.mj_forward(plant.model,plant.data)
    feet=[plant.model.geom(f'{leg}_foot').id for leg in ('fl','fr','rl','rr')]
    plant.data.qpos[2]+=.001-min(foot_clearance(plant.model,plant.data,i) for i in feet)
    mujoco.mj_forward(plant.model,plant.data)
    trajectory=[(0.,np.degrees(plant.data.qpos[plant.q]).copy())]
    max_roll=max_pitch=0.;frames=[]
    for t in np.arange(0,max(s['end'] for s in samples)+20,20):
        index=max(0,bisect.bisect_right(times,t)-1)
        plant.step(targets_deg=targets[index],balance=False,native_servo=native)
        row=plant.row();max_roll=max(max_roll,abs(row['roll_deg']));max_pitch=max(max_pitch,abs(row['pitch_deg']))
        trajectory.append((t+20,np.array(row['actual_deg'])))
        measured=min(imu,key=lambda r:abs(r['time_ms']-(t+20))) if imu else None
        frames.append(dict(time_ms=float(t+20),roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],
                           measured_roll_deg=measured['roll'] if measured else None,
                           measured_pitch_deg=measured['pitch'] if measured else None,
                           phase=measured['phase'] if measured else None,
                           clearance_mm=[foot_clearance(plant.model,plant.data,f)*1000 for f in feet],contacts=row['contacts'],loads_n=foot_loads(plant.model,plant.data)))
    ts=np.array([v[0] for v in trajectory]);angles=np.array([v[1] for v in trajectory])
    comparison=[]
    for row in rows:
        j=row['joint'];pred=float(np.interp(row['time_ms'],ts,angles[:,j]))
        comparison.append({**row,'sim_actual_deg':pred,'sim_minus_hardware_deg':pred-row['actual_deg']})
    result=dict(revision=meta.get('revision'),input=str(path.resolve()),plant_mode='estimated replay; optional empirical parameter overrides',
        mass_kg=float(plant.model.body_mass.sum()),command_delay_s=params['command_delay_s'],
        replay_step_ms=20,excluded_startup_ms=1000,max_sim_roll_deg=max_roll,max_sim_pitch_deg=max_pitch,joints={})
    result['initial_attitude_source']='first recorded IMU; axis convention assumed' if initial_attitude else 'level assumption'
    result['startup_validation_excluded']=False
    result['limitations']=['Command-driven replay, not independent closed-loop prediction.',
                          'Absolute initial body height/contact, floor carpet properties and physical IMU offsets are unmeasured.',
                          'Servo and cushion mechanics remain estimates; no fitting to force agreement.',
                          'Contact counts are model evidence, not real contact measurements.']
    if imu:
        result['imu_rmse_deg']={axis:float(np.sqrt(np.mean([(f[axis+'_deg']-f['measured_'+axis+'_deg'])**2 for f in frames]))) for axis in ('roll','pitch')}
        first=[]
        for f in frames:
            if f['phase']<.5:break
            if .6<=f['phase']<=.9:first.append(f)
        result['first_swing_midphase']={}
        for i,leg in ((1,'fr'),(2,'rl')):
            result['first_swing_midphase'][leg]=dict(samples=len(first),
                contact_fraction=sum(leg+'_foot' in f['contacts'] for f in first)/len(first) if first else None,
                loaded_fraction=sum(f['loads_n'][i]>.5 for f in first)/len(first) if first else None,
                min_clearance_mm=min((f['clearance_mm'][i] for f in first),default=None),
                max_clearance_mm=max((f['clearance_mm'][i] for f in first),default=None))
        result['first_swing_clearance_pass']=bool(first) and all(v['loaded_fraction']==0 and v['min_clearance_mm']>2 for v in result['first_swing_midphase'].values())
    for j in range(12):
        part=[r for r in comparison if r['joint']==j and r['time_ms']>=1000]
        e=np.array([r['sim_minus_hardware_deg'] for r in part])
        result['joints'][f"{('FL','FR','RL','RR')[j//3]}-J{j%3+1}"]=dict(samples=len(part),prediction_rmse_deg=float(np.sqrt(np.mean(e*e))) if len(e) else None)
        startup=[r['sim_minus_hardware_deg'] for r in comparison if r['joint']==j and r['time_ms']<1000]
        result['joints'][f"{('FL','FR','RL','RR')[j//3]}-J{j%3+1}"]['first_second_prediction_rmse_deg']=float(np.sqrt(np.mean(np.square(startup)))) if startup else None
    if output is None:
        return result, comparison
    output.mkdir(parents=True,exist_ok=True)
    (output/'comparison.json').write_text(json.dumps(result,indent=2))
    (output/'frames.json').write_text(json.dumps(frames))
    import csv
    with (output/'comparison.csv').open('w',newline='') as f:
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
    if imu:
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
        ft=np.array([f['time_ms']/1000 for f in frames])
        for ax,axis in zip(axes[:2],('roll','pitch')):
            ax.plot(ft,[f[axis+'_deg'] for f in frames],label='MuJoCo prediction')
            ax.plot(ft,[f['measured_'+axis+'_deg'] for f in frames],label='Hardware IMU')
            ax.set_ylabel(axis+' (deg)');ax.grid(alpha=.2);ax.legend()
        for i,leg in ((1,'FR'),(2,'RL')):
            axes[2].plot(ft,[f['clearance_mm'][i] for f in frames],label=leg+' model clearance')
        axes[2].axhline(0,color='black',lw=.8);axes[2].axhline(2,color='gray',ls='--')
        axes[2].set_ylabel('mm');axes[2].set_xlabel('Seconds after first recorded command');axes[2].legend();axes[2].grid(alpha=.2)
        axes[2].set_xlim(0,1.2)
        fig.suptitle('First swing: actual-speed recorded commands; model clearance is not measured clearance')
        fig.tight_layout();fig.savefig(output/'first-swing.png',dpi=130);plt.close(fig)
    return result, comparison
if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('trace',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--parameters',type=Path)
    p.add_argument('--imu',type=Path);p.add_argument('--initial-attitude',action='store_true')
    a=p.parse_args();r,_=replay(a.trace,a.output,json.loads(a.parameters.read_text()) if a.parameters else None,a.imu,a.initial_attitude);print(json.dumps(r,indent=2))
