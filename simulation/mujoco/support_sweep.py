"""Fore/aft foot endpoints in the torso frame, with C2 contact boundaries.

This plans kinematics; keeping a stance foot fixed in world coordinates still
requires the body velocity to match the planned stance velocity.
"""
import math


def fore_aft_path(phase, duty, period, touchdown, liftoff):
    """Return x, dx/dt, d2x/dt2 (m, m/s, m/s²), without startup scaling.

    The swing matches the stance's backward velocity at both boundaries.
    Consequently its extremes slightly exceed the contact endpoints; report
    those excursions instead of claiming touchdown is the maximum forward x.
    """
    if not all(map(math.isfinite,(phase,duty,period,touchdown,liftoff))):
        raise ValueError('Nonfinite fore/aft path parameter')
    if not 0<duty<1 or period<=0 or touchdown<=liftoff:
        raise ValueError('Invalid fore/aft path parameter')
    q=phase%1.;travel=touchdown-liftoff
    if q<duty:
        return touchdown-travel*q/duty,-travel/(period*duty),0.
    u=(q-duty)/(1-duty);duration=period*(1-duty);k=(1-duty)/duty
    s=10*u**3-15*u**4+6*u**5
    ds=30*u*u*(1-u)**2;dds=60*u*(1-u)*(1-2*u)
    return (liftoff+travel*((1+k)*s-k*u),
            travel*((1+k)*ds-k)/duration,
            travel*(1+k)*dds/duration**2)
