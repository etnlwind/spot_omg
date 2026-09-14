"""Experimental Cartesian lift/hold/place trajectory for position servos.

Uses the same legacy two-link geometry, signs and joint limits as locomotion.h.
This is a simulator-only planner; effective timing is tested across plant models.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
from simulation.mujoco.runtime.gait_profiles import foot_targets, smooth

def targets(params,phase,amplitude,linear,yaw,config):
    p=np.asarray(params,dtype=float)
    rise=float(config.get('rise_fraction',.3));fall=float(config.get('fall_fraction',.3))
    if not np.isfinite([rise,fall]).all() or not .15<=rise<=.5 or not .15<=fall<=.5 or rise+fall>1:
        raise ValueError('Invalid lift/hold/place fractions')
    front_extra=float(config.get('front_turn_extra_lift_m',0.))
    front_hold=float(config.get('front_turn_hold',0.))
    rear_extra=float(config.get('rear_turn_extra_lift_m',0.))
    if not np.isfinite(rear_extra) or not -.018<=rear_extra<=.02:
        raise ValueError('Rear turn lift adjustment must be within -18..20mm')
    if not np.isfinite([front_extra,front_hold]).all() or not 0<=front_extra<=.04 or not 0<=front_hold<=1:
        raise ValueError('Front turn lift must be within 0..40mm and hold blend within 0..1')
    # Validate inputs/reach using the deployed C function before experimental shaping.
    base=foot_targets(p,phase*p[0],amplitude,'trot',linear,yaw)
    result=base.copy();pivot=smooth(np.clip(1-abs(linear)/.6,0,1))
    activity=min(1,abs(linear)+(1+pivot)*abs(yaw))
    for i,offset in enumerate((0,.5,.5,0)):
        q=(phase+offset)%1
        if q<p[1]:continue
        u=(q-p[1])/(1-p[1]);k=(1-p[1])/p[1]
        x=p[2]*(-.5+(1+k)*smooth(u)-k*u)
        envelope=smooth(u/rise) if u<rise else smooth((1-u)/fall) if u>1-fall else 1.
        if config.get('forward_only',False):
            standard=64*u**3*(1-u)**3
            envelope=standard+(envelope-standard)*smooth(np.clip(linear/.5,0,1))
        z=p[4]-amplitude*activity*p[3]*envelope
        extra=front_extra if i<2 else rear_extra
        if extra:
            turn=pivot*smooth(abs(yaw)/.25)
            arch=64*u**3*(1-u)**3
            held=smooth(u/rise) if u<rise else smooth((1-u)/fall) if u>1-fall else 1.
            z-=amplitude*turn*extra*(arch+(held-arch)*front_hold)
        lateral=-amplitude*yaw*x*(1.6 if i<2 else -1.6)*pivot
        x=p[5]+amplitude*np.clip(linear+(yaw if i%2==0 else -yaw),-1,1)*x
        c=(x*x+z*z-.141**2-.150**2)/(2*.141*.150)
        if not -1<=c<=1:raise ValueError('Unreachable lift/hold/place target')
        knee=np.arccos(c);hip=np.arctan2(-x,z)+np.arctan2(.150*np.sin(knee),.141+.150*np.cos(knee))
        result[3*i:3*i+3]=[p[6]+np.degrees(np.arctan2(lateral*(1 if i%2==0 else -1),z)),np.degrees(hip),np.degrees(knee)]
    inside=float(config.get('cad_turn_inside_extra_m',0.))
    if not np.isfinite(inside) or not 0<=inside<=.01:raise ValueError('Inside-front lift must be within 0..10mm')
    if config.get('cad_turn_lift_m') is not None or inside:
        from simulation.mujoco.runtime.arc_swing_shape import ArcSwingShape
        delta=np.asarray(config.get('cad_turn_lift_m',[0.,0.,0.,0.]),dtype=float).copy()
        if delta.shape!=(4,):raise ValueError('Expected four CAD lift values')
        delta[0 if yaw<0 else 1]+=inside
        frame={'target_phase':phase,'duty':p[1],'motion_scale':float(amplitude*pivot*smooth(abs(yaw)/.25))}
        result=ArcSwingShape().apply(result,frame,delta)
    if np.any(result<np.tile([-30,-45,0],4)) or np.any(result>np.tile([30,100,150],4)):
        raise ValueError('Lift/hold/place joint limit')
    return result
