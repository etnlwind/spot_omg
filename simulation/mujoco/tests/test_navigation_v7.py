"""V7 uses the exact embedded kernel; geometry checked against CAD MuJoCo FK."""
import ctypes as C
import numpy as np
import pytest
from servo import SharedGaitPolicy
from simulation.mujoco.runtime.drive_controller import NAMES
from simulation.mujoco.runtime.support_shift import SupportShift
from simulation.mujoco.tests.test_s_native_v623 import plant

class Navigation:
    def __init__(self):
        self.lib=SharedGaitPolicy()._library
        self.lib.spot_navigation_size.restype=C.c_uint
        self.state=C.create_string_buffer(self.lib.spot_navigation_size())
        self.fn=self.lib.spot_navigation_step
        fp=C.POINTER(C.c_float)
        self.fn.argtypes=[C.c_void_p,*([C.c_float]*3),*([C.c_int]*3),C.c_float,C.c_float,fp,fp]
        self.fn.restype=C.c_int
        self.diag=np.zeros(16)
    def step(self,linear,yaw,heading=0,enabled=False,valid=True,stop=False,rate=1):
        out=(C.c_float*12)();diag=(C.c_float*16)()
        assert self.fn(self.state,linear,yaw,heading,valid,enabled,stop,.02,rate,out,diag)
        self.diag=np.array(diag);return np.array(out)
    def run(self,linear,yaw,n=150,**kw):
        return np.array([self.step(linear,yaw,**kw) for _ in range(n)])

@pytest.mark.parametrize('linear,yaw,expected',[(1,0,(1,0,0)),(-1,0,(-1,0,0)),
    (0,1,(0,0,1)),(0,-1,(0,0,-1)),(.7,.7,(.7,.5,0)),(.7,-.7,(.7,-.5,0)),
    (-.7,.7,(0,.5,0)),(-.7,-.7,(0,-.5,0))])
def test_all_eight_directions(linear,yaw,expected):
    n=Navigation();n.run(linear,yaw)
    np.testing.assert_allclose(n.diag[[1,2,11]],expected,atol=1e-6)

def test_reverse_half_cadence_preserves_v6_path():
    n=Navigation();n.run(-.588,0)
    phase=n.diag[0];out=n.step(-.588,0)
    fn=n.lib.spot_locomotion_targets;fp=C.POINTER(C.c_float)
    fn.argtypes=[C.c_int,*([C.c_float]*4),fp];fn.restype=C.c_int
    base=(C.c_float*12)()
    assert fn(NAMES.index('attitudepd_v7'),phase,1,-.588,0,base)
    np.testing.assert_allclose(out,base,atol=1e-4)
    period=n.lib.spot_locomotion_period;period.argtypes=[C.c_int,C.c_float,C.c_float];period.restype=C.c_float
    assert n.diag[13]==pytest.approx(2*period(NAMES.index('attitudepd_v6'),-.588,0))
    assert (n.diag[0]-phase)%1==pytest.approx(.02/n.diag[13],abs=1e-7)

@pytest.mark.parametrize('linear',[.588,-.588])
def test_heading_changes_stride_only_and_releases_for_turn(linear,plant):
    baseline=Navigation();corrected=Navigation();kin=SupportShift(plant.model)
    delta=[]
    for frame in range(240):
        a=baseline.step(linear,0)
        b=corrected.step(linear,0,heading=0 if frame<120 else 4,enabled=True)
        if frame>145:
            kin.set_angles(a);fa=np.array([kin.foot(i) for i in range(4)])
            kin.set_angles(b);fb=np.array([kin.foot(i) for i in range(4)])
            np.testing.assert_allclose(fb[:,1:],fa[:,1:],atol=2e-5)
            delta.append(np.max(abs(fb[:,0]-fa[:,0])))
        assert corrected.diag[0]==baseline.diag[0]
        assert corrected.diag[13]==baseline.diag[13]
    assert max(delta)>.001
    assert corrected.diag[7]>0 and corrected.diag[9]==1
    corrected.step(.588,.4,heading=4,enabled=True)
    assert corrected.diag[7]==0 and corrected.diag[9]==0

def test_stale_imu_and_heading_toggle_remove_only_correction():
    n=Navigation();n.run(.6,0,enabled=True);n.run(.6,0,heading=5,enabled=True,n=40)
    assert n.diag[7]>0
    n.step(.6,0,valid=False,enabled=True)
    assert n.diag[7]==0 and n.diag[1]>.5

def test_simulator_keeps_full_lateral_input_until_navigation_decodes_it(plant):
    from simulation.mujoco.runtime.virtual_robot import RobotController
    r=RobotController(plant)
    r.command('gaitprofile attitudepd_v7',0)
    r.command('drive 0 1000 1',0)
    assert r.request==(0,1)
    r.command('@D 2 0 -1000',.1)
    assert r.request==(0,-1)

def test_mode_change_reaches_neutral_and_stop_cancels_restart():
    n=Navigation();n.run(.8,0)
    for _ in range(100):
        previous=n.diag.copy();n.step(0,1)
        if n.diag[12]!=previous[12]:
            assert max(abs(n.diag[[1,2,11]]))<.001
            assert n.diag[3]==0
            break
    else:pytest.fail('side mode never selected')
    n.run(0,1)
    n.run(0,0,n=110,stop=True)
    assert max(abs(n.diag[[1,2,11]]))<.001

def test_sideways_lifts_one_foot_and_stance_moves_opposite_body(plant):
    kin=SupportShift(plant.model);n=Navigation();n.run(0,1)
    rows=[]
    for _ in range(220):
        q=n.step(0,1);kin.set_angles(q)
        rows.append(np.array([kin.foot(i) for i in range(4)]))
    rows=np.array(rows)
    assert np.min(np.ptp(rows[:,:,1],axis=0))>.022
    assert np.max(np.ptp(rows[:,:,0],axis=0))<2e-5
    assert np.min(np.ptp(rows[:,:,2],axis=0))>.022
    # Positive side input = body right (-Y); planted feet move +Y.
    grounded=rows[:-1,:,2] < rows[:,:,2].min(axis=0)+.0005
    dy=np.diff(rows[:,:,1],axis=0)
    assert np.median(dy[grounded])>0
