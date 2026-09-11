"""Free-body interventions plus exact telescoping FL-RR clearance decomposition."""
import copy,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import mujoco
from check_body_height import OUT,ROOT
from diagnose_turn_clearance import run
from cad_physics import foot_clearance
from support_shift import SupportShift

def evaluate(case):
    cfg=json.loads((OUT/'diagonal-preload-1.json').read_text())['profile'];s=cfg['support_shift']
    if case=='no_attitude':s.update(feedback_gain=0,feedback_damping_s=0,ki=0)
    if case=='no_height':s['height_feedback_gain']=0
    if case=='no_preload':s['load_preload_gain']=0
    if case=='nominal_only':s.update(feedback_gain=0,feedback_damping_s=0,ki=0,height_feedback_gain=0,load_preload_gain=0)
    if case=='world_swing':s['world_swing']=True
    if case=='no_height_preload':s.update(height_feedback_gain=0,load_preload_gain=0)
    frames=[];kin=None
    def observe(t,r):
        nonlocal kin
        if case=='no_heading':r.heading.enabled=False
        if t<5:return
        if kin is None:kin=SupportShift(r.plant.model)
        stages=[]
        for q in [r.target,r.command_target,np.degrees(r.plant.filtered),np.degrees(r.plant.data.qpos[r.plant.q])]:
            kin.set_angles(q);stages.append([kin.foot(i)[2] for i in range(4)])
        kin.data.qpos[3:7]=r.plant.data.qpos[3:7]
        mujoco.mj_kinematics(kin.model,kin.data);mujoco.mj_comPos(kin.model,kin.data)
        stages.append([kin.foot(i)[2] for i in range(4)])
        stages=np.asarray(stages)*1000
        gaps=stages[:,0]-stages[:,3]
        row=r.plant.row()
        m,d=r.plant.model,r.plant.data
        com=d.subtree_com[m.body('robot').id].copy()
        feet=np.array([d.geom_xpos[m.geom(l+'_foot').id] for l in ('fl','fr','rl','rr')])
        forces=np.zeros((4,3));moment=np.zeros(3);floor=m.geom('floor').id
        foot_ids=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
        cached_min=[float(np.min((kin.vertices[i]@d.geom_xmat[f].reshape(3,3).T+d.geom_xpos[f])[:,2])) for i,f in enumerate(foot_ids)]
        cached_legacy=[foot_clearance(m,d,f) for f in foot_ids]
        for k,c in enumerate(d.contact):
            if floor not in (c.geom1,c.geom2):continue
            other=c.geom2 if c.geom1==floor else c.geom1
            f=np.zeros(6);mujoco.mj_contactForce(m,d,k,f)
            sign=1 if c.geom1==floor else -1
            force=sign*c.frame.reshape(3,3).T@f[:3]
            moment+=np.cross(c.pos-com,force)+sign*c.frame.reshape(3,3).T@f[3:]
            if other in foot_ids:forces[foot_ids.index(other)]+=force
        frames.append(dict(time_s=t,phase=r.support_shift.last_phase,yaw_command=r.yaw,
            stages_z_mm=stages.tolist(),gap_components_mm=np.r_[gaps[0],np.diff(gaps)].tolist(),
            world_gap_mm=gaps[-1],roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],
            foot_centers_world_m=feet.tolist(),com_world_m=com.tolist(),foot_forces_n=forces.tolist(),ground_moment_about_com_nm=moment.tolist(),
            cached_mesh_min_m=cached_min,cached_legacy_clearance_m=cached_legacy,
            nominal_deg=r.target.tolist(),command_deg=r.command_target.tolist(),actual_deg=row['actual_deg'],
            diagnostic=copy.deepcopy(r.support_shift.diagnostic)))
    result,rows=run(1000,0,20.02,profile='sync_'+case,override=cfg,cushion=json.loads((ROOT/'foot_cushion_10mm.json').read_text()),observer=observe)
    a=[r for r in rows if r['leg']=='FL'];b=[r for r in rows if r['leg']=='RR']
    mid=[i for i,r in enumerate(a) if r['middle_swing']]
    result.update(case=case,max_roll_deg=max(abs(f['roll_deg']) for f in frames),max_pitch_deg=max(abs(f['pitch_deg']) for f in frames),
        mismatch_fraction=float(np.mean([(a[i]['force_n']>.2)!=(b[i]['force_n']>.2) for i in mid])),
        gap_rms_mm=float(np.sqrt(np.mean([(a[i]['clearance_mm']-b[i]['clearance_mm'])**2 for i in mid]))),
        decomposition_labels=['nominal_geometry','feedback_and_preload','command_filter','joint_tracking','body_orientation'])
    (OUT/('sync-cause-'+case+'.json')).write_text(json.dumps(result,indent=2))
    (OUT/('sync-cause-'+case+'-frames.json')).write_text(json.dumps(frames,default=lambda x:x.item()))
    print(case,result['safety'],result['max_roll_deg'],result['gap_rms_mm'],result['mismatch_fraction'],flush=True)
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(evaluate,['baseline','no_attitude','no_height','no_preload','nominal_only']))
