"""One-step geometry only. These tests do not certify physical contact timing."""
import ctypes as C
import numpy as np
import pytest
from simulation.mujoco.tests.test_navigation_v7 import Navigation
from simulation.mujoco.tests.test_s_native_v623 import plant
from simulation.mujoco.runtime.support_shift import SupportShift


def trajectory(cfg):
    n=Navigation();base=n.step(0,0)
    fn=n.lib.spot_navigation_v10_configured
    fn.argtypes=[C.c_float]*3+[C.POINTER(C.c_uint32),C.POINTER(C.c_float),C.POINTER(C.c_float)]
    fn.restype=C.c_int
    def target(phase):
        q=(C.c_float*12)(*base)
        assert fn(phase,1,-1,(C.c_uint32*4)(10,10,10,10),(C.c_float*6)(*cfg),q)
        return np.array(q)
    return base,target


@pytest.mark.parametrize('push',[.25,.75,1.])
def test_landing_push_ends_and_recovery_returns_stand(plant,push):
    cfg=[.04,.02,.12,push,.1,0]
    base,target=trajectory(cfg);kin=SupportShift(plant.model)
    def feet(q):
        kin.set_angles(q);return np.array([kin.foot(i) for i in range(4)])
    origin=feet(base);landing=feet(target(.12))
    np.testing.assert_allclose((landing-origin)[:,1],[.04*(1-push),-.04*push,.04*(1-push),-.04*push],atol=4e-5)
    # Right Y holds at its push endpoint during initial folding, never pushes farther.
    for phase in np.linspace(.12,.13+.35*.12,8):
        np.testing.assert_allclose((feet(target(phase))-origin)[[1,3],1],[-.04*push]*2,atol=4e-5)
    # Left J1 accepts body movement while maintaining X/Z at the planted foot.
    for phase in np.linspace(.18,.27,8):
        np.testing.assert_allclose((feet(target(phase))-origin)[[0,2]][:,[0,2]],0,atol=4e-5)
    np.testing.assert_allclose(target(.45),base,atol=.025)
    np.testing.assert_allclose(target(.95),target(.45),atol=1e-6)
    assert max(abs((target(.19)-base).reshape(4,3)[[1,3],2]))>3


def test_left_only_kernel_does_not_repeat_step():
    n=Navigation();n.fn=n.lib.spot_navigation_v10_step
    n.fn.argtypes=n.lib.spot_navigation_step.argtypes;n.fn.restype=C.c_int
    n.run(0,-1,n=350)
    assert n.diag[0]==pytest.approx(.45)
    n.run(0,-1,n=100)
    assert n.diag[0]==pytest.approx(.45)
