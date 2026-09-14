"""Isolated preview adapter. No deployed firmware changes."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
from pathlib import Path
import sys,numpy as np
ROOT=REPO_ROOT
from simulation.mujoco.runtime.position_wbc import EncoderChannel

def configure(robot,config=None):
    config=config or {}
    from simulation.mujoco.runtime.ground_frame_nominal import configure as original
    info=original(robot,config)
    if config.get("heading_off"):robot.heading.enabled=False
    if 'balance_kp' in config:
        base_apply=robot.balance.apply
        def tuned_apply(*args,**kwargs):
            if robot.motion:
                robot.balance.kp=config['balance_kp']
                robot.balance.ki=config.get('balance_ki',0.)
                robot.balance.kd=config.get('balance_kd',0.)
            return base_apply(*args,**kwargs)
        robot.balance.apply=tuned_apply
    if config.get('legacy_balance'):
        install_lead(robot,config)
        return info
    from simulation.mujoco.runtime.ground_frame_control import GroundFrameControl
    controller=GroundFrameControl(robot.plant.model);channel=EncoderChannel()
    old_apply=robot.balance.apply
    robot.ground_frame=controller
    def apply(nominal,attitude,valid,permitted):
        # Exactly one feedback sample per control tick, with 40ms/4096 delay.
        encoders=channel.read(np.degrees(robot.plant.data.qpos[robot.plant.q]))
        if not robot.motion:
            controller.reset()
            return old_apply(nominal,attitude,valid,permitted)
        params=robot.active_profile_params()
        phase=getattr(robot,"ground_evaluated_phase",(robot.phase-.02/params[0])%1)
        enabled=bool(valid and permitted and robot.imu_reading and robot.imu_reading['age_ms']<=100)
        if config.get('preserve_legacy_stance'):nominal=old_apply(nominal,attitude,valid,permitted)
        return controller.apply(nominal,encoders,attitude,phase,getattr(robot,"ground_duty",params[1]),config,enabled)
    robot.balance.apply=apply
    install_lead(robot,config)
    info['ground_frame_config']=config
    return info


def install_lead(robot,config):
    lead=float(config.get('lead_s',0))
    if not 0<=lead<=.1:raise ValueError('Lead out of range')
    if not lead:return
    original=robot.balance.apply
    previous=None;velocity=np.zeros(12)
    def apply(*args,**kwargs):
        nonlocal previous,velocity
        q=original(*args,**kwargs)
        if previous is not None and robot.motion and robot.safety=='ok':
            velocity+=.5*((q-previous)/.02-velocity)
        else:velocity[:]=0
        previous=q.copy()
        return np.clip(q+np.clip(lead*velocity,-10,10),np.tile([-30,-45,0],4),np.tile([30,100,150],4))
    robot.balance.apply=apply
