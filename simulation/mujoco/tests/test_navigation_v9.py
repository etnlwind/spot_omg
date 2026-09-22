"""V9 topology/transfer tests; these do not certify dynamic walking stability."""
import ctypes as C
import numpy as np
import pytest
from simulation.mujoco.tests.test_navigation_v7 import Navigation
from simulation.mujoco.tests.test_navigation_v8 import PairedNavigation
from simulation.mujoco.tests.test_s_native_v623 import plant
from simulation.mujoco.runtime.support_shift import SupportShift


class SameSideNavigation(Navigation):
    def __init__(self):
        super().__init__()
        args=self.fn.argtypes
        self.fn=self.lib.spot_navigation_v9_step
        self.fn.argtypes=args;self.fn.restype=C.c_int


@pytest.mark.parametrize('linear,yaw',[(1,0),(-1,0),(.588,0),(-.588,0),(.6,.4),(.6,-.4),(-.6,.4),(-.6,-.4)])
def test_other_directions_preserve_v8(linear,yaw):
    old,new=PairedNavigation(),SameSideNavigation()
    for i in range(300):
        np.testing.assert_array_equal(old.step(linear,yaw,heading=i*.01,enabled=True,stop=i>=200),
                                      new.step(linear,yaw,heading=i*.01,enabled=True,stop=i>=200))
        np.testing.assert_array_equal(old.diag,new.diag)


@pytest.mark.parametrize('direction',[-1,1])
def test_same_side_pairs_and_transfer_preserve_x_z(plant,direction):
    n=SameSideNavigation();kin=SupportShift(plant.model)
    n.run(0,direction,n=200)
    transfer=n.lib.spot_navigation_v9_transfer
    transfer.argtypes=[C.c_float,C.c_float,C.c_float,C.POINTER(C.c_float)];transfer.restype=C.c_int
    positions=[];phases=[]
    for _ in range(240):
        phase=n.diag[0];target=n.step(0,direction)
        kin.set_angles(target);before=np.array([kin.foot(i) for i in range(4)])
        q=(C.c_float*12)(*target);assert transfer(phase,1,.06,q)
        kin.set_angles(q);after=np.array([kin.foot(i) for i in range(4)])
        np.testing.assert_allclose(after[:,[0,2]],before[:,[0,2]],atol=3e-5)
        np.testing.assert_allclose((after-before)[:,1],np.repeat((after-before)[0,1],4),atol=3e-5)
        # At swing centers: all feet translate opposite to the support side.
        p=phase%1
        if .05<p<.2:assert (after-before)[0,1]>.059
        if .55<p<.7:assert (after-before)[0,1]<-.059
        positions.append(before);phases.append(p)
    dz=np.array(positions)[:,:,2];dz-=dz.min(axis=0)
    np.testing.assert_allclose(dz[:,0],dz[:,2],atol=2e-5)
    np.testing.assert_allclose(dz[:,1],dz[:,3],atol=2e-5)
    assert set((dz>.001).sum(axis=1))=={0,2}


def test_stop_and_switch_to_forward_clear_lateral_command():
    n=SameSideNavigation();n.run(0,1,n=300)
    n.run(0,0,n=110,stop=True)
    assert max(abs(n.diag[[1,2,11]]))<.001
    n.run(.6,0,n=100)
    assert n.diag[12]==0 and n.diag[11]==0 and n.diag[1]>.5
