import numpy as np
import pytest
from types import SimpleNamespace
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait
from simulation.mujoco.scripts.analysis.analyze_large_recovery_balance import Recovery

@pytest.fixture(scope='module')
def plant():
    return Simulation(load_parameters(parse_args([])))

def test_duty_change_waits_for_landing_and_keeps_boundary_continuous(plant):
    gait=SNativeGait(plant.model,plant.stand_target)
    policy=Recovery({'steady_duty':.7})
    gait.entry_phase=.99
    policy.prepare(gait,.49)
    np.testing.assert_array_equal(policy.duties,[.5]*4)
    gait.entry_phase=1.001
    before=gait.points(.499999,1,.344,0)
    policy.prepare(gait,.500001)
    after=gait.points(.500001,1,.344,0)
    np.testing.assert_array_equal(policy.duties,[.7,.5,.5,.7])
    np.testing.assert_allclose(before,after,atol=1e-6)
    policy.prepare(gait,.8)
    np.testing.assert_array_equal(policy.duties,[.7,.5,.5,.7])
    gait.entry_phase=1.501
    policy.prepare(gait,.000001)
    np.testing.assert_array_equal(policy.duties,[.7]*4)

def test_low_stop_places_j1_at_s_before_height_restore(plant):
    gait=SNativeGait(plant.model,plant.stand_target)
    policy=Recovery({'crouch':.045,'low_stop':True})
    policy.setup(gait);gait.entry_phase=2;gait.entry_previous_phase=.8
    original=gait.origin.copy()
    gait.begin_stop()
    for _ in range(81):target=gait.placement_stop_targets()
    assert gait.stop_ready
    np.testing.assert_allclose(target[::3],plant.stand_target[::3],atol=1e-5)
    gait.kin.set_angles(target)
    feet=np.array([gait.kin.foot(i) for i in range(4)])
    np.testing.assert_allclose(feet[:,0],original[:,0],atol=.0002)
    np.testing.assert_allclose(feet[:,2],original[:,2]+.045,atol=.0002)

def test_preserved_fixed_j1_path_discards_lateral_support_goal(plant):
    outputs=[]
    for offset in (0,.01):
        gait=SNativeGait(plant.model,plant.stand_target)
        gait.support_table=np.full(64,offset);gait.entry_phase=2;gait.entry_previous_phase=.65
        outputs.append(gait.targets(.65,1,.344,0))
    np.testing.assert_allclose(outputs[0],outputs[1],atol=1e-7)

def test_j2_85_cannot_keep_old_ground_height_in_cad(plant):
    gait=SNativeGait(plant.model,plant.stand_target);heights=[]
    for j3 in np.linspace(0,130,261):
        q=plant.stand_target.copy();q[6:9]=[-9,85,j3]
        gait.kin.set_angles(q)
        heights.append(gait.kin.foot(2)[2]-gait.origin[2,2])
    # Full scan at normal J1=-9 gives 37.8mm; the original 16mm
    # clearance cannot coexist with this requested J2 angle.
    assert min(heights)>.03


@pytest.mark.parametrize('override', [{'crouch':.001},{'j1':0},{'j1':-9},{'low_stop':True}])
def test_fixed_frame_rejects_height_or_j1_changes(override):
    with pytest.raises(ValueError):
        Recovery({'fixed_frame':True,**override})


def test_fixed_frame_balance_keeps_stance_height_and_j1(plant):
    # Check the physical contract across both support pairs, including the
    # double-support transitions, with a nonzero IMU attitude correction.
    gait=SNativeGait(plant.model,plant.stand_target)
    policy=Recovery({'fixed_frame':True,'steady_duty':.7,'diagonal_x_balance':1.})
    policy.duties[:]=.7;policy.setup(gait)
    robot=SimpleNamespace(imu_reading={'age_ms':0},
        attitude_filter=SimpleNamespace(filtered=[10.,-5.],failures=0))
    gait.entry_phase=2.;gait.entry_previous_phase=0.
    expected_j1=gait.standing[::3]+gait.normal_adduction
    for phase in np.linspace(0,.999,101):
        nominal=gait.targets(phase,1,.344,0)
        result=policy(robot,gait,phase,nominal)
        np.testing.assert_allclose(result[::3],expected_j1,atol=1e-7)
        gait.kin.set_angles(result)
        feet=np.array([gait.kin.foot(i) for i in range(4)])
        stance=((phase+np.array([.5,0,0,.5]))%1)<.7
        np.testing.assert_allclose(feet[stance,2],gait.origin[stance,2],atol=.0002)
