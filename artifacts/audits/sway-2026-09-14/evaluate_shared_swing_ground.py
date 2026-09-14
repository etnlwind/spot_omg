"""Independent FF + gravity-frame swing experiment. Never imports hardware I/O."""
from pathlib import Path
import json,sys,numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'simulation/mujoco'))
from cad_physics import Simulation,foot_clearance
from virtual_robot import RobotController
from ground_frame_preview import configure
from shared_swing_ground import install

gain=float(sys.argv[1]);case='shared-swing-'+str(gain)
stop_at=float(sys.argv[2]) if len(sys.argv)>2 else None
if stop_at is not None:case+='-stop'+str(stop_at)
config=json.loads((ROOT/'simulation/mujoco/support_transition_feedforward_candidate.json').read_text())['config']
p=json.loads((ROOT/'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
p.update(timestep_s=.0005,experimental_stow=True)
p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
plant=Simulation(p);robot=RobotController(plant);robot.select_profile('centerpivot')
geometry=configure(robot,config);control=install(robot,gain)
m,d=plant.model,plant.data;feet=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')];floor=m.geom('floor').id
records=[];fault=None
for i in range(1500 if stop_at is None else round((stop_at+3)/.02)):
    t=i*.02
    if i==500:robot.command('drive 1000 0 1',t)
    if stop_at is not None and abs(t-stop_at)<.009:robot.command('stop',t)
    if 500<i and i%10==0 and (stop_at is None or t<stop_at):robot.command(f'@D {i} 1000 0',t)
    robot.tick(t);robot.drain()
    force=np.zeros(4)
    for j,c in enumerate(d.contact):
        if floor not in (c.geom1,c.geom2):continue
        other=c.geom2 if c.geom1==floor else c.geom1
        if other in feet:
            f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f);force[feet.index(other)]+=max(0,f[0])
    row=plant.row()
    records.append(dict(t=t,phase=getattr(robot,'ground_evaluated_phase',robot.phase),linear=robot.linear,
        roll=row['roll_deg'],pitch=row['pitch_deg'],com=row['com_m'],force=force.tolist(),
        clearance=[foot_clearance(m,d,f)*1000 for f in feet],command=robot.command_target.tolist(),
        actual=np.degrees(d.qpos[plant.q]).tolist(),sat=plant.saturated,ground=control.diagnostic.copy(),
        pose=robot.pose,motion=robot.motion,transition_active=robot.transition is not None))
    if robot.safety!='ok':fault=dict(t=t,reason=robot.safety);break
summary={'gain':gain,'fault':fault,'record_end_s':records[-1]['t']}
if stop_at is not None:
    after=[r for r in records if r['t']>=stop_at]
    completed=next((r['t'] for r in after if r['motion'] is None and not r['transition_active']),None)
    transition_start=next((j for j,r in enumerate(records) if r['t']>=stop_at and r['motion'] is None),None)
    summary['stop']=dict(requested_s=stop_at,completed_s=completed,final_pose=robot.pose,
        transition_start_s=None if transition_start is None else records[transition_start]['t'],
        transition_start_command_step_deg=None if transition_start is None or transition_start==0 else float(np.max(abs(np.array(records[transition_start]['command'])-records[transition_start-1]['command']))),
        transition_start_previous_correction_deg=None if transition_start is None else records[transition_start]['ground'].get('previous_correction_deg'),
        max_stop_command_step_deg=float(np.max(abs(np.diff(np.array([r['command'] for r in after]),axis=0)))))
for label,epoch in [('full_forward',10.),('steady',14.)]:
    rows=[r for r in records if r['t']>=epoch]
    if not rows:summary[label]=None;continue
    arr=lambda key:np.array([r[key] for r in rows]);phase=(arr('phase')[:,None]+[0,.5,.5,0])%1
    u=(phase-.7)/.3;mid=(u>1/6)&(u<5/6);f=arr('force')
    summary[label]=dict(roll_rms_deg=float(np.sqrt(np.mean(arr('roll')**2))),roll_max_deg=float(abs(arr('roll')).max()),
        pitch_rms_deg=float(np.sqrt(np.mean(arr('pitch')**2))),pitch_max_deg=float(abs(arr('pitch')).max()),
        mid_swing_contact_fraction=[float(np.mean(f[mid[:,j],j]>1)) if mid[:,j].any() else None for j in range(4)],
        peak_clearance_mm=[float(arr('clearance')[mid[:,j],j].max()) if mid[:,j].any() else None for j in range(4)],
        forward_speed_m_s=float((arr('com')[-1,0]-arr('com')[0,0])/max(.02,rows[-1]['t']-rows[0]['t'])))
active=[r['ground'] for r in records if r['ground'].get('enabled') and not r['ground'].get('all_stance')]
summary['controller']=dict(active_samples=len(active),invalid_samples=sum(not r['ground'].get('enabled',True) and r['ground'].get('reason')!='not-moving' for r in records if r['t']>=10),
    nonmoving_samples=sum(r['ground'].get('reason')=='not-moving' for r in records if r['t']>=10),
    stance_preserved=all(r['stance_preserved'] for r in active),
    max_correction_deg=max((max(abs(np.array(r['correction_deg']))) for r in active),default=0.),
    max_correction_step_deg=max((r['max_correction_step_deg'] for r in active),default=0.),
    limited_fraction=float(np.mean([r['correction_limited'] for r in active])) if active else 0.)
output=Path(__file__).with_name('ground-'+case+'.json')
output.write_text(json.dumps(dict(summary=summary,config=config,experiment_gain=gain,geometry=geometry,records=records),indent=2))
print(json.dumps(summary),flush=True)
