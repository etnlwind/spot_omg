"""Rear reach is independent of cycle count; check geometry, rates and C parity."""
import ctypes as ct
import json
import numpy as np
import pytest
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES
NAME="s_native_v6_2_3"
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args,RobotController
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding,compare,ROOT

@pytest.fixture(scope='module')
def plant():return Simulation(load_parameters(parse_args([])))
@pytest.fixture(scope='module')
def kernel(tmp_path_factory):return load_binding(tmp_path_factory.mktemp('v623'))

@pytest.mark.parametrize('command,stop',[((1.,0.),0),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4),((-.6,0.),2),((0.,.5),2),((0.,-.5),2),((.6,.25),2),((.43,.17),2)])
def test_c_python_entry_walk_stop(kernel,plant,command,stop):
 assert compare(kernel,plant,command,stop,profile=NAME)['max_target_error_deg']<.15

def test_rear_endpoint_pair_symmetry_and_preserved_old_models(plant):
 g=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
 p=g.points(0.,1.,1.,0.)-g.origin
 np.testing.assert_allclose(p[:,0],[-.135,.020,.020,-.135],atol=1e-10)
 for phase in np.linspace(0,1,101):
  d=g.points(phase,1.,1.,0.)-g.origin
  np.testing.assert_allclose(d[0,[0,2]],d[3,[0,2]],atol=1e-10)
  np.testing.assert_allclose(d[1,[0,2]],d[2,[0,2]],atol=1e-10)
 assert PROFILES['s_native_v6_2_2']['params'][:4]==[.5,.5,.145,.012]
 assert PROFILES['s_native_v6_2_1']['params'][0]==2.
 d=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
 assert ['legacy',*d['profiles']][16:20]==['s_native_v6_1','s_native_v6_2_1','s_native_v6_2_2',NAME]
 assert RobotController(plant).profile==d['default']=='s_native_v6_2_7'

def test_lift_has_no_boundary_velocity_or_acceleration_jump(plant):
 g=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME]);h=1e-5
 z=lambda phase:g.points(phase,1.,1.,0.)[1,2]-g.origin[1,2]
 for boundary in [0.,.5]:
  a,b,c=[z((boundary+k*h)%1) for k in [-1,0,1]]
  assert abs((c-a)/(2*h))<1e-6
  assert abs((a-2*b+c)/h**2)<.01

def test_dense_steady_trajectory_rates_and_servo_encoding(kernel,plant):
 g=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME]);g.prepare_support(1.,0.)
 # Complete entry before measuring a dense 5ms cycle (including boundaries).
 for phase in np.arange(0,2,.025):g.targets((.5+phase)%1,1.,1.,0.)
 n=round(PROFILES[NAME]['params'][0]/.005)
 q=np.array([g.targets((.5+i/n)%1,1.,1.,0.) for i in range(n+1)])
 speed=np.abs(np.diff(q,axis=0))/.005
 assert np.all(speed.max(axis=0)<=np.tile([270.,451.,270.],4))
 for row in q:
  assert kernel.encode((ct.c_float*12)(*row),(ct.c_uint16*12)())
 # Replay the measured supported-test underfold envelope; do not assume perfect following.
 g.kin.set_angles(q[n//4]);target=np.array([g.kin.foot(i)[2] for i in (1,2)])
 under=q[n//4].copy();under[[5,8]]-=14;g.kin.set_angles(under)
 assert np.all(target-np.array([g.kin.foot(i)[2] for i in (1,2)])>.012)
