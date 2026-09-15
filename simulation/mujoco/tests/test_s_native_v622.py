"""V6.2.2: same phase geometry, faster clock, preserved native servo bounds."""
import ctypes as ct
import json
import re
import numpy as np
import pytest
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES,NAME
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args,RobotController
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding,compare,ROOT

@pytest.fixture(scope='module')
def plant():return Simulation(load_parameters(parse_args([])))
@pytest.fixture(scope='module')
def kernel(tmp_path_factory):return load_binding(tmp_path_factory.mktemp('v622'))

def test_preserves_phase_geometry_and_support_reference(plant):
 a=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_2_1'])
 b=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_2_2'])
 a.prepare_support(1.,0.);b.prepare_support(1.,0.)
 np.testing.assert_array_equal(a.support_table,b.support_table)
 for phase in np.linspace(.5,2.5,101):
  np.testing.assert_allclose(a.targets(phase%1,1,1,0),b.targets(phase%1,1,1,0),atol=1e-9)
 assert PROFILES['s_native_v6_2_1']['params'][0]==2.
 assert PROFILES['s_native_v6_2_2']['params'][0]==.5
 assert b.profile['stop_period_s']==a.profile['stop_period_s']==1.6

@pytest.mark.parametrize('command,stop', [((1.,0.),0.),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4.),((-.6,0.),2.),((0.,.5),2.),((0.,-.5),2.),((.6,.25),2.)])
def test_fast_c_python_parity_and_stopping(kernel,plant,command,stop):
 assert compare(kernel,plant,command,stop,profile='s_native_v6_2_2')['max_target_error_deg']<.12  # < 2 servo ticks; compressed support interpolation

def test_new_default_append_only_indices_and_native_encoding(plant):
 data=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
 names=['legacy',*data['profiles']]
 assert names[16:19]==['s_native_v6_1','s_native_v6_2_1','s_native_v6_2_2']
 assert data['default']==NAME=='s_native_v6_2_5'
 robot=RobotController(plant)
 assert robot.profile==NAME
 assert robot.limited_linear(-1.)==-.6

def test_full_input_command_rate_within_supported_servo_profile(kernel):
 kernel.reset_v621();phase=.5;linear=0.;q=[];out=(ct.c_float*12)()
 for i in range(400):
  linear=min(1.,linear+.04);u=min(1,(i+1)*.02);amp=u**3*(10+u*(-15+6*u))
  assert kernel.step(phase,amp,linear,0,1,0,.02,0,out)
  q.append(list(out));phase=(phase+.02/(.5*(1.35-.35*linear)))%1
 limits=(ROOT/'firmware/stm32-learning/Inc/motor_capability.h').read_text()
 v3215=float(re.search(r'MOTOR_STS3215_NOMINAL_MAX_VELOCITY_DEG_S ([0-9.]+)f',limits).group(1))
 v3250=float(re.search(r'MOTOR_STS3250_NOMINAL_MAX_VELOCITY_DEG_S ([0-9.]+)f',limits).group(1))
 assert np.all(np.max(np.abs(np.diff(q,axis=0)),axis=0)/.02<=np.tile([v3215,v3250,v3215],4))


def test_observed_underfold_can_consume_nominal_recovery_clearance(plant):
 # The supported v63 trace measured roughly 14 degrees of J3 tracking error.
 # Replay that error as a sensitivity case, not as a measured time-delay model.
 gait=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_2_2'])
 gait.prepare_support(1.,0.)
 for phase in np.linspace(.5,2.75,151):
  target=gait.targets(phase%1,1.,1.,0.)
 gait.kin.set_angles(target)
 desired=np.array([gait.kin.foot(i)[2] for i in (1,2)])
 underfold=target.copy();underfold[[5,8]]-=14
 gait.kin.set_angles(underfold)
 actual=np.array([gait.kin.foot(i)[2] for i in (1,2)])
 assert np.all(desired-actual>.012)
