"""Simulator-only foot-space locomotion families; not a firmware policy.

Different phase schedules and speed-dependent stride/cadence are evaluated on
unchanged CAD inertia, contact, voltage and servo limits before promotion.
"""
import json
import math
from pathlib import Path
import numpy as np

PROFILE_FILE = Path(__file__).parent/'gait_search'/'profiles'/'selected.json'
PHASES = {
    'trot': (0., .5, .5, 0.),
    'crawl': (0., .5, .75, .25),
    'amble': (0., .5, .75, .25),
    'pace': (0., .5, 0., .5),
    'bound': (0., 0., .5, .5),
}


def smooth(x):
    x = min(1., max(0., float(x)))
    return x*x*x*(10+x*(-15+6*x))


def foot_targets(params, t, scale=1., family='trot', linear=1., yaw=0.):
    """C2 swing path with stance-matched touchdown velocity and analytic IK.

    params: period, stance duty, stride, clearance, hip-foot height, fore/aft
    foot offset, outward hip spread. Yaw combines differential fore/aft stride
    with opposed front/rear lateral foot travel about the body center.
    """
    period,duty,stride,lift,height,offset,spread = map(float, params)
    if not all(math.isfinite(v) for v in (*params,t,scale,linear,yaw)):
        raise ValueError('Nonfinite gait input')
    if period <= 0 or not .45 <= duty <= .9 or not 0 <= scale <= 1:
        raise ValueError('Invalid period, duty or amplitude')
    if max(abs(linear),abs(yaw)) > 1: raise ValueError('Drive input outside -1..1')
    out=[]
    for i, phase in enumerate(PHASES[family]):
        p=(t/period+phase)%1
        # Positive yaw means right turn in the existing app command convention.
        command=max(-1., min(1.,linear+(yaw if i in (0,2) else -yaw)))
        if p < duty:
            x=stride*(.5-p/duty)
            z=height
        else:
            u=(p-duty)/(1-duty)
            k=(1-duty)/duty
            x=stride*(-.5+(1+k)*smooth(u)-k*u)
            z=height-lift*64*u**3*(1-u)**3
        # Rotation needs lateral foot travel as well as differential fore/aft
        # stride. Front and rear feet sweep opposite ways about the body center.
        # Approximate stance half-length / half-width for this CAD assembly.
        # Blend out the pivot sweep during travel: combining a full lateral
        # sweep with a full forward stride can bring links into the floor.
        pivot_weight=smooth(1-abs(linear)/.6)
        lateral=-scale*yaw*x*(1.6 if i < 2 else -1.6)*pivot_weight
        activity=min(1.,abs(linear)+(1+pivot_weight)*abs(yaw))
        x=offset+scale*command*x
        z=height+scale*activity*(z-height)
        upper,lower=.141,.150
        c=(x*x+z*z-upper*upper-lower*lower)/(2*upper*lower)
        if not -1 <= c <= 1: raise ValueError('Unreachable foot target')
        knee=math.acos(c)
        hip=math.atan2(-x,z)+math.atan2(lower*math.sin(knee),upper+lower*math.cos(knee))
        abduction=spread+math.degrees(math.atan2(lateral*(1 if i in (0,2) else -1),z))
        out.extend((abduction,math.degrees(hip),math.degrees(knee)))
    values=np.array(out)
    if np.any(values.reshape(4,3)<[-30,-45,0]) or np.any(values.reshape(4,3)>[30,100,150]):
        raise ValueError('Target outside joint limits')
    return values


def load_profiles():
    return json.loads(PROFILE_FILE.read_text())['profiles']
