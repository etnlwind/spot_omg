"""Geometry, preserved V6.1, capability and real-plant checks for V6.2."""
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES,NAME
from simulation.mujoco.scripts.validation.validate_s_native_v62 import geometry,trial


def test_v62_extends_rear_with_nearly_vertical_lower_leg_and_more_flexed_placement():
    plant=Simulation(load_parameters(parse_args([])))
    before,_=geometry(plant,'s_native_v6_1');after,_=geometry(plant,NAME)
    for leg in ('rl','rr'):
        old=before['rear_endpoints'][leg];back=after['rear_endpoints'][leg];front=after['front_endpoints'][leg]
        assert abs(old['x_mm']+65)<.01
        assert abs(back['x_mm']+125)<.01 and 88<back['lower_from_ground_deg']<=90
        assert abs(front['x_mm']-20)<.01
        assert front['j3_deg']>back['j3_deg']+15
    assert PROFILES['s_native_v6_1']['params']==[1.2,.5,.085,.012]
    old=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_1'])
    new=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    for phase in np.linspace(0,1,51):
        np.testing.assert_allclose(new.points(phase,0,1,0),old.origin,atol=1e-12)
        a=new.points(phase,1,1,0)-new.origin
        np.testing.assert_allclose(a[0,[0,2]],a[3,[0,2]],atol=1e-12)
        np.testing.assert_allclose(a[1,[0,2]],a[2,[0,2]],atol=1e-12)


def test_v62_full_forward_walks_straight_and_completes_stop_with_contacts_enabled():
    report,_=trial()
    assert report['first_fault_s'] is None and report['completed']
    assert report['walk_displacement_m'][0]>.8 and abs(report['walk_yaw_deg'])<2
    assert report['max_walk_tilt_deg']<5 and report['max_stop_tilt_deg']<8
    assert report['final_s_target_error_deg']<.01 and report['final_s_actual_error_deg']<1.1
    assert report['non_floor_contact_frames']==0


def test_latest_default_and_v62_reverse_bound_do_not_change_v61():
    robot=RobotController(Simulation(load_parameters(parse_args([]))))
    assert robot.profile==NAME and robot.limited_linear(-1)==-.6
    robot.command('syncstate',0)
    state=robot.drain().decode()
    assert 's_native_v6_2' in state and 'reverse_limit=600' in state
    robot.select_profile('s_native_v6_1')
    assert robot.limited_linear(-1)==-1

