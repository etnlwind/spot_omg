"""Causal diagnostic, not a gait policy. Fixed-base mode is a bench test."""
from pathlib import Path
import sys, json, numpy as np, mujoco
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'simulation/mujoco'),str(ROOT/'artifacts/gait-videos/2026-09-14')]
from cad_physics import Simulation,foot_clearance
from virtual_robot import RobotController
from shoulder_j2_lift32_preview import configure
case=sys.argv[1]
p=json.loads((ROOT/'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
p.update(timestep_s=.0005,experimental_stow=True)
if 'nolag' in case:p.update(command_delay_s=0.,target_acceleration_rad_s2=1e6)
p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
plant=Simulation(p);robot=RobotController(plant);robot.select_profile('centerpivot');geometry=configure(robot)
if 'nocorrection' in case:robot.heading.enabled=False;robot.balance.enabled=False
m,d=plant.model,plant.data;feet=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')];floor=m.geom('floor').id
if 'bench' in case:
    # Explicit intervention: all feet airborne, torso clamped, gravity retained.
    # This separates the servo path from free-body/contact coupling.
    m.geom_pos[floor,2]=-10.
    m.geom_contype[floor]=0;m.geom_conaffinity[floor]=0
    root_pose=d.qpos[:7].copy();observe=plant.sensor_observer
    def fixed_body(model,data):
        data.qpos[:7]=root_pose;data.qvel[:6]=0.
        mujoco.mj_forward(model,data)
        observe(model,data)
    plant.sensor_observer=fixed_body
records=[]
for i in range(800):
    t=i*.02
    if i==500:robot.command('drive 1000 0 1',t)
    if i>500 and i%10==0:robot.command(f'@D {i} 1000 0',t)
    delayed=np.degrees(plant.delay[0]).tolist() if plant.delay else None
    phase=robot.phase;robot.tick(t);robot.drain();row=plant.row()
    force=np.zeros(4)
    self_contact=[]
    for j,c in enumerate(d.contact):
        if floor in (c.geom1,c.geom2):
            other=c.geom2 if c.geom1==floor else c.geom1
            if other in feet:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f);force[feet.index(other)]+=max(0,f[0])
        else:
            f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f)
            self_contact.append(dict(geoms=[m.geom(c.geom1).name,m.geom(c.geom2).name],distance_m=float(c.dist),normal_n=float(f[0])))
    records.append(dict(t=t,phase=phase,yaw_command=robot.yaw,roll=row['roll_deg'],pitch=row['pitch_deg'],nominal=robot.target.tolist(),command=robot.command_target.tolist(),encoded=np.degrees(plant.desired).tolist(),delayed=delayed,filtered=np.degrees(plant.filtered).tolist(),actual=np.degrees(d.qpos[plant.q]).tolist(),velocity=d.qvel[plant.v].tolist(),target_velocity=plant.target_velocity.tolist(),torque=d.ctrl.tolist(),bias=d.qfrc_bias[plant.v].tolist(),constraint=d.qfrc_constraint[plant.v].tolist(),force=force.tolist(),clearance=[foot_clearance(m,d,f)*1000 for f in feet],safety=robot.safety))
    records[-1]['self_contact']=self_contact
    if delayed is None:records[-1]['delayed']=records[-1]['encoded']
    if 'bench' in case and np.any(force):raise RuntimeError('Bench intervention did not remove floor contact')
    if robot.safety!='ok':break
summary={}
rows=[r for r in records if r['t']>=10]
for field in ('nominal','command','encoded','delayed','filtered','actual','torque','bias','constraint'):
    a=np.array([r[field] for r in rows]).reshape(-1,4,3)
    summary[field]={'pair_rms_FL_RR_FR_RL':np.sqrt(np.mean((a[:,[0,1]]-a[:,[3,2]])**2,axis=0)).tolist(),'pair_max_FL_RR_FR_RL':np.max(abs(a[:,[0,1]]-a[:,[3,2]]),axis=0).tolist()}
summary['roll_max']=max(abs(r['roll']) for r in rows)
out=dict(case=case,geometry=geometry,model=dict(mass_kg=float(m.body_mass.sum()),speed_rad_s=plant.speed.tolist(),stall_nm=plant.stall.tolist(),delay_s=p['command_delay_s'],acceleration_rad_s2=p['target_acceleration_rad_s2'],zero_error_deg=plant.joint_zero_error.tolist()),summary=summary,records=records)
Path(__file__).with_name('causal-'+case+'.json').write_text(json.dumps(out,indent=2))
print(json.dumps({'case':case,'summary':summary}),flush=True)
