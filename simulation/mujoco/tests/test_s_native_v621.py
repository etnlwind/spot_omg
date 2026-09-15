"""V6.2.1 continuous crossing and knee-recovery regressions."""
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES,NAME

def test_v621_crossing_has_no_speed_dip_or_hold_and_preserves_endpoints():
    plant=Simulation(load_parameters(parse_args([])))
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    phase=np.linspace(0,1,2001)
    path=np.array([gait.points(p,1,1,0)-gait.origin for p in phase])
    speed=np.abs(np.gradient(path[:,2,0],phase*PROFILES[NAME]['params'][0]))
    for crossing in (500,1500):
        # A 16%-of-peak crawl at each crossing was visibly a hesitation.
        assert speed[crossing]>.6*speed.max()
        assert speed[crossing]>.20
        assert np.min(speed[crossing-50:crossing+51])>.15
    assert np.all(np.diff(path[:1001,2,0])<0)
    assert np.all(np.diff(path[1000:,2,0])>0)
    np.testing.assert_allclose(path[:,2,0].min(),-.125,atol=1e-12)
    np.testing.assert_allclose(path[:,2,0].max(),.020,atol=1e-12)
    assert 'continuous_recovery' not in PROFILES['s_native_v6_1']
    assert 'continuous_recovery' not in PROFILES['s_native_v6_2']


def test_v621_folds_knee_while_moving_and_unfolds_before_landing():
    plant=Simulation(load_parameters(parse_args([])))
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    reference=SNativeGait(plant.model,plant.stand_target,dict(PROFILES[NAME],lift_exponent=.5))
    for phase in np.linspace(.5,2.5,101):
        gait.targets(phase%1,1,1,0);reference.targets(phase%1,1,1,0)
    points=[];angles=[];previous=[]
    for phase in np.linspace(.5,1,101):
        angles.append(gait.targets(phase,1,1,0))
        previous.append(reference.targets(phase,1,1,0))
        points.append(gait.points(phase,1,1,0)-gait.origin)
    points=np.asarray(points);angles=np.asarray(angles);previous=np.asarray(previous)
    # RL recovers during this half-cycle. Every interior X sample advances;
    # no lift/translation/landing dwell was introduced.
    assert np.all(np.diff(points[:,2,0])>0)
    assert np.all(points[1:-1,2,2]>0)
    assert points[0,2,2]==0 and points[-1,2,2]==0
    assert angles[50,8]>angles[0,8]+20
    assert angles[90,8]>angles[-1,8]+4
    assert angles[90,8]>previous[90,8]+1
    assert PROFILES['s_native_v6_2']['lift_exponent']==.5
    assert PROFILES['s_native_v6_2']['stop_period_s']==1.2


def test_v621_protected_walk_and_two_step_stop():
    from simulation.mujoco.scripts.validation.validate_s_native_v62 import trial
    report,_=trial(profile=NAME)
    assert report['first_fault_s'] is None and report['completed']
    assert report['walk_displacement_m'][0]>.9 and abs(report['walk_yaw_deg'])<2
    assert report['max_walk_tilt_deg']<6 and report['max_stop_tilt_deg']<7
    assert report['non_floor_contact_frames']==0
    assert report['final_s_target_error_deg']<.01 and report['final_s_actual_error_deg']<1.1
