"""Paired lateral geometry, preservation of V7 directions, and reset/stop."""
import ctypes as C
import numpy as np
import pytest
from simulation.mujoco.tests.test_navigation_v7 import Navigation
from simulation.mujoco.tests.test_s_native_v623 import plant
from simulation.mujoco.runtime.support_shift import SupportShift


class PairedNavigation(Navigation):
    def __init__(self):
        super().__init__()
        old=self.fn
        self.fn=self.lib.spot_navigation_v8_step
        self.fn.argtypes=old.argtypes
        self.fn.restype=old.restype


@pytest.mark.parametrize('linear,yaw',[(.588,0),(1,0),(-.588,0),(-1,0),(.6,.4),(.6,-.4),(-.6,.4),(-.6,-.4)])
def test_non_lateral_commands_preserve_v7(linear,yaw):
    old,new=Navigation(),PairedNavigation()
    for frame in range(350):
        stop=frame>=250
        a=old.step(linear,yaw,heading=frame*.01,enabled=True,stop=stop)
        b=new.step(linear,yaw,heading=frame*.01,enabled=True,stop=stop)
        np.testing.assert_array_equal(a,b)
        np.testing.assert_array_equal(old.diag[:14],new.diag[:14])


@pytest.mark.parametrize('direction',[-1,1])
def test_diagonal_pair_clearance_is_synchronized_and_has_full_support_interval(plant,direction):
    n=PairedNavigation();kin=SupportShift(plant.model)
    n.run(0,direction,n=200)
    rows=[]
    for _ in range(300):
        kin.set_angles(n.step(0,direction))
        rows.append(np.array([kin.foot(i) for i in range(4)]))
    rows=np.array(rows)
    dz=rows[:,:,2]-rows[:,:,2].min(axis=0)
    # Independent MuJoCo FK, not just the planner's stance flag.
    np.testing.assert_allclose(dz[:,0],dz[:,3],atol=2e-5)
    np.testing.assert_allclose(dz[:,1],dz[:,2],atol=2e-5)
    raised=dz>.001
    assert set(raised.sum(axis=1))=={0,2}
    assert np.any(raised[:,0]) and np.any(raised[:,1])
    assert np.all(~(raised[:,0] & raised[:,1]))
    dy=np.diff(rows[:,:,1],axis=0)
    supported=(dz[:-1]<.0001)&(dz[1:]<.0001)
    assert np.median(dy[supported])*direction>0


def test_side_stop_and_mode_change_do_not_leave_latent_motion():
    n=PairedNavigation();n.run(0,1,n=200)
    n.run(0,0,n=110,stop=True)
    assert max(abs(n.diag[[1,2,11]]))<.001
    n.run(.6,0,n=100)
    assert n.diag[12]==0 and n.diag[11]==0 and n.diag[1]>.5


def test_contact_audit_rejects_one_dragging_foot_despite_paired_commands():
    from scripts.analysis.verify_v93_side_step import paired_clearance
    row=dict(stage='walk',walk_s=5,navigation=dict(paired_side=True,duty=.75),
             leg_phase=[.875,.375,.375,.875],clearance_mm=[10,0,0,0],load_n=[0,10,10,5])
    assert paired_clearance([row])['both_clear_fraction']==0
    row['clearance_mm'][3]=10;row['load_n'][3]=0
    assert paired_clearance([row])['both_clear_fraction']==1
    row['navigation']['paired_side']=False
    assert paired_clearance([row])['both_clear_fraction'] is None
