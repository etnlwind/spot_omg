import json
from pathlib import Path
import numpy as np
from simulation.mujoco.runtime.servo_profile import ServoProfile
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args

ROOT=Path(__file__).resolve().parents[3]

def test_real_register_snapshot_overrides_ui_nominal_motor_speed():
    rows=json.loads((ROOT/'artifacts/s-native-v6-2-4/hardware-pilot-8s-retry/servo-registers.json').read_text())
    p=ServoProfile([r['goal_speed'] for r in rows],[r['acceleration'] for r in rows])
    nominal=np.tile(np.radians([270,451,270]),4)
    np.testing.assert_allclose(np.degrees(p.velocity_limit(nominal)),300*360/4096)
    np.testing.assert_allclose(np.degrees(p.acceleration_limit()),30*100*360/4096)
    p.set(3400,254)
    np.testing.assert_allclose(np.degrees(p.velocity_limit(nominal)),np.tile([270,3400*360/4096,270],4))

def test_stand_drive_restore_and_reproduce_previous_firmware_bug():
    for restore in [False,True]:
        parameters=load_parameters(parse_args([]));parameters['firmware_servo_profile_restore']=restore
        plant=Simulation(parameters);robot=RobotController(plant)
        robot.select_profile('s_native_v6_2_3')
        robot.begin(('drive',),0)
        assert np.all(plant.servo_profile.speed==300)
        robot.request=(1.,0.);robot.last_packet=0;robot.tick(0)
        assert np.all(plant.servo_profile.speed==(3400 if restore else 300))
        np.testing.assert_array_equal(plant.servo_profile.acceleration,
                                      [50,254,50]*4 if restore else [30]*12)

def test_v67_mixed_readback_keeps_per_servo_acceleration_limits():
    rows=json.loads((ROOT/'artifacts/servo-profile-restore-v67/hardware-pilot-10s-retry/servo-registers.json').read_text())
    p=ServoProfile([r['goal_speed'] for r in rows],[r['acceleration'] for r in rows])
    assert set(p.acceleration)=={50,254}
    assert np.all(p.speed==3400)
    np.testing.assert_allclose(np.degrees(p.acceleration_limit()),
                               np.array([r['acceleration'] for r in rows])*100*360/4096)

def test_installed_profile_matches_all_twelve_v68_readbacks():
    rows=json.loads((ROOT/'artifacts/servo-profile-restore-v68/hardware-30s/servo-registers.json').read_text())
    plant=Simulation(load_parameters(parse_args([])))
    np.testing.assert_array_equal(plant.servo_profile.acceleration,[r['acceleration'] for r in rows])
    plant.servo_profile.set(300,30)
    np.testing.assert_array_equal(plant.servo_profile.acceleration,[30]*12)
    plant.servo_profile.set(3400,254)
    np.testing.assert_array_equal(plant.servo_profile.acceleration,[r['acceleration'] for r in rows])

def test_plant_internal_reference_obeys_register_acceleration_and_speed():
    plant=Simulation(load_parameters(parse_args([])));plant.servo_profile.set(300,30)
    previous=plant.filtered.copy()
    target=plant.stand_target.copy();target[1]+=20
    for _ in range(20):
        plant.step(targets_deg=target,torque_enabled=False,native_servo=True)
        assert np.max(np.abs(plant.filtered-previous))<=300*2*np.pi/4096*.02+1e-9
        assert np.max(np.abs(plant.target_velocity))<=300*2*np.pi/4096+1e-9
        previous=plant.filtered.copy()
