"""Isolated CAD stance experiment; no deployed firmware or profile mutation."""
import numpy as np
import mujoco
from support_shift import SupportShift, LEGS
from gait_profiles import foot_targets, smooth
import virtual_robot
from cad_physics import foot_clearance

def configure(robot):
    kin=SupportShift(robot.plant.model)
    old=np.tile([0.,45.,90.],4);kin.set_angles(old)
    original=np.array([kin.foot(i) for i in range(4)])
    shoulders=np.array([kin.data.xanchor[kin.model.joint(l+'_j2').id].copy() for l in LEGS])
    goals=original.copy();goals[:,0]=shoulders[:,0];goals[:,2]=original[:,2].mean()
    neutral,residual=kin.solve(goals,old,iterations=30)
    if residual>.0001:raise RuntimeError(f'Neutral unreachable: {residual}')
    # J2 anchor moves slightly with J1; converge on its solved position.
    for _ in range(3):
        kin.set_angles(neutral)
        shoulders=np.array([kin.data.xanchor[kin.model.joint(l+'_j2').id].copy() for l in LEGS])
        goals[:,0]=shoulders[:,0]
        neutral,residual=kin.solve(goals,neutral,iterations=30)
    kin.set_angles(neutral);actual=np.array([kin.foot(i) for i in range(4)])
    info=dict(shoulders_m=shoulders.tolist(),neutral_deg=neutral.reshape(4,3).tolist(),foot_goals_m=goals.tolist(),old_feet_m=original.tolist(),neutral_residual_m=residual,max_path_residual_m=0.,max_command_step_deg=0.)
    plant=robot.plant;plant.desired=np.radians(neutral);plant.data.qpos[plant.q]=plant.desired
    mujoco.mj_forward(plant.model,plant.data)
    plant.data.qpos[2]+=.001-min(foot_clearance(plant.model,plant.data,g) for g in kin.feet)
    mujoco.mj_forward(plant.model,plant.data)
    plant.filtered=plant.desired.copy();plant.delay=__import__('collections').deque([plant.desired.copy() for _ in plant.delay]);plant.target_velocity[:]=0
    robot.target=neutral.copy();robot.command_target=neutral.copy()
    old_step=virtual_robot.shared_drive_step
    previous=neutral.copy()
    def step(r):
        nonlocal previous
        evaluated_phase=r.phase
        nominal=old_step(r)
        if r.safety!='ok':return nominal
        params=r.active_profile_params()
        base=foot_targets(params,0,0)
        kin.set_angles(base);reference=np.array([kin.foot(i) for i in range(4)])
        kin.set_angles(nominal);points=np.array([kin.foot(i) for i in range(4)])
        targets=goals+(points-reference)
        # Explicit CAD cushion-bottom trajectory: 40mm at full forward.
        # No arbitrary joint-angle boost; solve coordinated J2/J3 via Jacobian.
        amplitude=smooth(min(1.,r.elapsed))*min(1.,abs(r.linear))
        duty=params[1]
        for leg,offset in enumerate((0.,.5,.5,0.)):
            q=(evaluated_phase+offset)%1.
            lift=0.
            if q>=duty:
                u=(q-duty)/(1-duty)
                held=smooth(u/.4) if u<.4 else smooth((1-u)/.3) if u>.7 else 1.
                arch=64*u**3*(1-u)**3
                lift=arch+(held-arch)*smooth(min(1.,max(0.,r.linear/.5)))
            targets[leg,2]=goals[leg,2]+.032*amplitude*lift
        info['peak_lift_target_mm']=32.
        result,error=kin.solve(targets,previous,iterations=20)
        info['max_path_residual_m']=max(info['max_path_residual_m'],error)
        info['max_command_step_deg']=max(info['max_command_step_deg'],float(np.max(abs(result-previous))))
        if error>.001:raise RuntimeError(f'Gait target unreachable: {error}')
        previous=result.copy();return result
    virtual_robot.shared_drive_step=step
    # No separate 1-second shift to the old rearward neutral before first step.
    original_begin=robot.begin
    def begin(motion,now):
        original_begin(motion,now)
        robot.transition=None;robot.target=neutral.copy()
    robot.begin=begin
    return info
