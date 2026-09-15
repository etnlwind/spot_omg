"""Photo reference: lower leg vertical; use the existing feedback supervisor."""
import pytest
from simulation.mujoco.tests.test_s_native_v623 import plant,kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES
NAME="s_native_v6_2_4"
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.validation.validate_s_native_firmware import compare
from simulation.mujoco.scripts.validation.validate_s_native_v62 import geometry

@pytest.mark.parametrize('command,stop',[((1.,0.),0),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4),((-.6,0.),2),((0.,.5),2),((0.,-.5),2),((.6,.25),2)])
def test_c_python_targets_and_stop(kernel,plant,command,stop):
 assert compare(kernel,plant,command,stop,profile=NAME)['max_target_error_deg']<.15

def test_photo_reference_is_lower_link_angle_not_servo_90(plant):
 report,_=geometry(plant,NAME)
 for row in report['rear_endpoints'].values():
  assert 88<row['lower_from_ground_deg']<=90
  assert abs(row['x_mm']+125)<.2
  assert 70<row['j3_deg']<80

def test_tracking_defaults_and_old_profile_preserved(plant):
 robot=RobotController(plant);robot.select_profile(NAME);assert robot.profile==NAME and robot.tracking_enabled
 robot.select_profile('s_native_v6_2_3');assert not robot.tracking_enabled
 robot.select_profile(NAME);assert robot.tracking_enabled
 assert PROFILES['s_native_v6_2_3']['rear_extension_m']==.115
