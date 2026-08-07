import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from servo import (
    AttitudeController,
    DynamicLoadBaseline,
    GaitParameters,
    ImuSample,
    LoadContactEstimator,
    ServoState,
    SharedGaitPolicy,
    SpotConfig,
    SpotRobot,
    STS3215,
)
from servo.cli import (
    apply_pose,
    capture_stand,
    configure_directions,
    configure_mapping,
    parse_args,
    resolve_gait,
    resolve_port,
    run_walk,
    swap_servo_ids_on_bus,
)


CONFIG = Path(__file__).resolve().parents[1] / "config" / "joints.json"


class RecordingBus:
    def __init__(self) -> None:
        self.calls = []

    def sync_write(self, address, item_size, values) -> None:
        self.calls.append((address, item_size, values))


class IdChangeBus:
    def __init__(self, *servo_ids: int) -> None:
        self.ids = set(servo_ids)
        self.writes = []

    def ping(self, servo_id: int) -> bool:
        return servo_id in self.ids

    def write(
        self,
        servo_id: int,
        address: int,
        data: bytes,
        *,
        expect_response: bool = True,
    ) -> None:
        if servo_id not in self.ids:
            raise RuntimeError(f"missing servo {servo_id}")
        self.writes.append((servo_id, address, data, expect_response))
        if address == STS3215.ADDR_ID:
            new_id = data[0]
            self.ids.remove(servo_id)
            self.ids.add(new_id)


class StateReadBus:
    def read(self, servo_id: int, address: int, size: int) -> bytes:
        self.last_read = (servo_id, address, size)
        raw = bytearray(15)
        raw[0:2] = (2050).to_bytes(2, "little")
        raw[2:4] = (12).to_bytes(2, "little")
        raw[4:6] = (0x0400 | 100).to_bytes(2, "little")
        raw[6] = 121
        raw[7] = 42
        raw[9] = 4
        raw[10] = 1
        raw[13:15] = (50).to_bytes(2, "little")
        return bytes(raw)


class DiagnosticsBus(StateReadBus):
    VALUES = {
        33: b"\x00",
        40: b"\x01",
        41: b"\x32",
        42: (2100).to_bytes(2, "little"),
        46: (1000).to_bytes(2, "little"),
        48: (900).to_bytes(2, "little"),
    }

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        if address in self.VALUES:
            return self.VALUES[address]
        return super().read(servo_id, address, size)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class SpotConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SpotConfig.load(CONFIG)

    def test_attitude_controller_extends_the_low_side(self) -> None:
        controller = AttitudeController(kp=1.0, kd=0.0, correction_limit=0.2)
        corrections = controller.leg_length_corrections(
            ImuSample(roll=0.1, pitch=0.0, roll_rate=0.0, pitch_rate=0.0)
        )
        self.assertEqual(corrections["FL"], -0.1)
        self.assertEqual(corrections["RL"], -0.1)
        self.assertEqual(corrections["FR"], 0.1)
        self.assertEqual(corrections["RR"], 0.1)

    def test_attitude_controller_combines_pitch_rate_and_clamps(self) -> None:
        controller = AttitudeController(kp=1.0, kd=0.5, correction_limit=0.15)
        corrections = controller.leg_length_corrections(
            ImuSample(roll=0.0, pitch=0.1, roll_rate=0.0, pitch_rate=0.2)
        )
        self.assertEqual(corrections["FL"], 0.15)
        self.assertEqual(corrections["FR"], 0.15)
        self.assertEqual(corrections["RL"], -0.15)
        self.assertEqual(corrections["RR"], -0.15)

    def test_attitude_controller_exposes_axis_efforts_for_foot_placement(self) -> None:
        controller = AttitudeController(kp=0.8, kd=0.1, correction_limit=0.15)
        roll, pitch = controller.axis_controls(
            ImuSample(roll=0.1, pitch=-0.2, roll_rate=0.3, pitch_rate=0.4)
        )
        self.assertAlmostEqual(roll, 0.11)
        self.assertAlmostEqual(pitch, -0.12)

    def test_load_contact_estimator_debounces_engage_and_release(self) -> None:
        baseline = {
            (leg, joint): 10
            for leg in ("FL", "FR", "RL", "RR")
            for joint in (2, 3)
        }
        detector = LoadContactEstimator(
            baseline,
            engage_threshold=100,
            release_threshold=60,
            engage_samples=2,
            release_samples=3,
        )
        unloaded = dict(baseline)
        loaded = dict(baseline)
        loaded[("FL", 3)] = -120

        self.assertFalse(detector.update(loaded)["FL"].contact)
        engaged = detector.update(loaded)["FL"]
        self.assertTrue(engaged.contact)
        self.assertTrue(engaged.changed)
        self.assertEqual(engaged.score, 130)
        self.assertTrue(detector.update(unloaded)["FL"].contact)
        self.assertTrue(detector.update(unloaded)["FL"].contact)
        released = detector.update(unloaded)["FL"]
        self.assertFalse(released.contact)
        self.assertTrue(released.changed)

    def test_load_contact_estimator_tracks_legs_independently(self) -> None:
        baseline = {
            (leg, joint): 0
            for leg in ("FL", "FR", "RL", "RR")
            for joint in (2, 3)
        }
        detector = LoadContactEstimator(
            baseline,
            engage_threshold=100,
            release_threshold=50,
            engage_samples=1,
            release_samples=1,
        )
        sample = dict(baseline)
        sample[("RR", 2)] = 150
        estimates = detector.update(sample)
        self.assertEqual(
            {leg for leg, estimate in estimates.items() if estimate.contact},
            {"RR"},
        )

    def test_dynamic_load_baseline_uses_phase_medians_and_round_trips(self) -> None:
        header = (
            "elapsed_s,phase,amplitude,leg,joint,servo_id,leg_phase,load\n"
        )
        lines = [header]
        servo_id = 1
        for leg in ("FL", "FR", "RL", "RR"):
            for joint in (2, 3):
                for phase in (0.0, 0.2, 0.4, 0.6, 0.8):
                    for variation in (-4, 0, 4):
                        lines.append(
                            f"{phase},{phase},1,{leg},{joint},{servo_id},{phase},"
                            f"{servo_id * 10 + round(phase * 100) + variation}\n"
                        )
                servo_id += 1
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "loads.csv"
            saved = Path(directory) / "baseline.json"
            profile.write_text("".join(lines), encoding="utf-8")
            baseline = DynamicLoadBaseline.from_csv(profile)
            baseline.save(saved)
            restored = DynamicLoadBaseline.load(saved)

        self.assertEqual(restored.expected_load(1, 0.21), 30.0)
        self.assertEqual(restored.residual(1, 0.21, 42), 12.0)
        self.assertEqual(restored.models[1]["engage_threshold"], 24)

    def test_hardware_mapping_and_saved_pose(self) -> None:
        self.assertEqual(self.config.servo_ids, tuple(range(1, 13)))
        first = self.config.joint("FL", 1)
        self.assertEqual(
            first.center, self.config.reference_center + first.offset
        )
        self.assertEqual(self.config.pose("landing")[9], 3238)
        self.assertEqual(self.config.pose("stand45")[12], 1522)
        generated = self.config.stand45_targets()
        expected = {
            2: 1723,
            3: 1036,
            5: 2511,
            6: 3001,
            8: 1630,
            9: 3020,
            11: 2552,
            12: 1023,
        }
        for servo_id, position in expected.items():
            self.assertEqual(generated[servo_id], position)
        self.assertEqual(
            tuple(
                self.config.joint(leg, 1).direction
                for leg in ("FL", "FR", "RL", "RR")
            ),
            (1, -1, -1, 1),
        )

    def test_all_legs_share_the_same_canonical_stand45_angles(self) -> None:
        targets = self.config.stand45_targets()
        for leg in ("FL", "FR", "RL", "RR"):
            for joint_number, expected_angle in ((2, 45.0), (3, 90.0)):
                servo_id = self.config.joint(leg, joint_number).servo_id
                angle = self.config.position_to_angle(
                    leg, joint_number, targets[servo_id]
                )
                self.assertAlmostEqual(angle, expected_angle, delta=0.05)

    def test_joint_angle_raw_conversion_round_trip(self) -> None:
        for leg in ("FL", "FR", "RL", "RR"):
            for joint_number in (1, 2, 3):
                for requested in (-20.0, 0.0, 20.0):
                    raw = self.config.angle_to_position(
                        leg, joint_number, requested
                    )
                    restored = self.config.position_to_angle(
                        leg, joint_number, raw
                    )
                    self.assertAlmostEqual(restored, requested, delta=0.05)

    def test_v3_serializes_one_direction_per_joint(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            self.config.save(path)
            data = json.loads(path.read_text())
        self.assertEqual(data["version"], 3)
        self.assertNotIn("gait_directions", data)
        self.assertNotIn("stand45_directions", data)
        self.assertTrue(
            all(joint["direction"] in (-1, 1) for joint in data["joints"])
        )
        self.assertEqual(
            data["canonical_poses"]["landing"], {"2": 40.0, "3": 130.0}
        )
        self.assertEqual(
            data["gait_forward_signs"],
            {"FL": -1, "FR": -1, "RL": 1, "RR": 1},
        )

    def test_v2_direction_tables_are_migrated_when_loading(self) -> None:
        data = json.loads(CONFIG.read_text())
        data["version"] = 2
        for joint in data["joints"]:
            joint.pop("direction")
        data["stand45_directions"] = {
            "FL": {"upper": -1, "lower": -1},
            "FR": {"upper": 1, "lower": 1},
            "RL": {"upper": -1, "lower": 1},
            "RR": {"upper": 1, "lower": -1},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            path.write_text(json.dumps(data))
            loaded = SpotConfig.load(path)
        self.assertEqual(loaded.directions["FL"], (-1, -1))
        self.assertEqual(loaded.directions["RR"], (1, -1))

    def test_sync_move_uses_sts_position_packet_layout(self) -> None:
        bus = RecordingBus()
        robot = SpotRobot(bus, self.config)
        robot.sync_move(
            self.config.pose("neutral"), speed=1000, acceleration=80
        )
        address, item_size, values = bus.calls[0]
        self.assertEqual((address, item_size), (41, 7))
        position = self.config.pose("neutral")[1].to_bytes(2, "little")
        self.assertEqual(values[1], bytes((80,)) + position + bytes((0, 0, 0xE8, 0x03)))

    def test_position_stream_does_not_rewrite_speed_or_acceleration(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        robot.sync_positions(self.config.pose("neutral"))
        address, item_size, values = robot.bus.calls[0]
        self.assertEqual((address, item_size), (42, 2))
        self.assertEqual(
            values[1], self.config.pose("neutral")[1].to_bytes(2, "little")
        )

    def test_stance_speeds_are_scaled_for_simultaneous_arrival(self) -> None:
        current = self.config.pose("neutral")
        targets = self.config.stand45_targets()
        speeds = SpotRobot.synchronized_arrival_speeds(
            current, targets, speed_cap=1000, acceleration=80
        )

        for leg in ("FL", "FR", "RL", "RR"):
            j1 = self.config.joint(leg, 1).servo_id
            j2 = self.config.joint(leg, 2).servo_id
            j3 = self.config.joint(leg, 3).servo_id
            self.assertEqual(speeds[j1], 1)
            self.assertEqual(speeds[j3], 1000)
            self.assertLess(speeds[j2], speeds[j3])

            acceleration_steps = 80 * 100.0
            j2_distance = abs(targets[j2] - current[j2])
            j3_distance = abs(targets[j3] - current[j3])
            j2_time = (
                j2_distance / speeds[j2]
                + speeds[j2] / acceleration_steps
            )
            j3_time = (
                j3_distance / speeds[j3]
                + speeds[j3] / acceleration_steps
            )
            self.assertAlmostEqual(j2_time, j3_time, delta=0.002)

    def test_zero_acceleration_uses_distance_proportional_speeds(self) -> None:
        speeds = SpotRobot.synchronized_arrival_speeds(
            {1: 1000, 2: 1000, 3: 1000},
            {1: 1000, 2: 1500, 3: 2000},
            speed_cap=1200,
            acceleration=0,
        )
        self.assertEqual(speeds, {1: 1, 2: 600, 3: 1200})

    def test_test_gait_keeps_hip_axis_fixed(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="crawl",
            period=5.0,
            hip_amplitude=5.0,
            lift_amplitude=7.0,
            crouch_amplitude=0,
            speed=30,
            acceleration=20,
        )
        base = self.config.stand45_targets()
        targets = robot.gait_targets(0.125, base, gait)
        for servo_id in (1, 4, 7, 10):
            self.assertEqual(targets[servo_id], base[servo_id])
        self.assertGreater(targets[9], base[9])

    def test_trot_keeps_diagonal_legs_in_sync(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=4.0,
            hip_amplitude=8.0,
            lift_amplitude=10.0,
            crouch_amplitude=0,
            speed=100,
            acceleration=20,
            duty_factor=0.5,
        )
        base = self.config.stand45_targets()
        targets = robot.gait_targets(0.125, base, gait)

        def normalized_foot(leg: str) -> tuple[float, float]:
            upper_id = self.config.joint(leg, 2).servo_id
            knee_id = self.config.joint(leg, 3).servo_id
            upper = self.config.position_to_angle(leg, 2, targets[upper_id])
            knee = self.config.position_to_angle(leg, 3, targets[knee_id])
            forward, down = robot.leg_forward_kinematics(upper, knee)
            return forward * self.config.gait_forward_signs[leg], down

        fl = normalized_foot("FL")
        fr = normalized_foot("FR")
        rl = normalized_foot("RL")
        rr = normalized_foot("RR")
        self.assertAlmostEqual(fl[0], rr[0], delta=0.002)
        self.assertAlmostEqual(fl[1], rr[1], delta=0.002)
        self.assertAlmostEqual(fr[0], rl[0], delta=0.002)
        self.assertAlmostEqual(fr[1], rl[1], delta=0.002)

    def test_shared_c_sim_trot_matches_python_reference(self) -> None:
        policy = SharedGaitPolicy()
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=0.8,
            hip_amplitude=8.8,
            lift_amplitude=30.0,
            crouch_amplitude=0.0,
            speed=60,
            acceleration=30,
            duty_factor=0.5,
            control_rate=50.0,
            stance_j1_angle=4.0,
            stance_j2_angle=45.0,
            stance_j3_angle=90.0,
        )
        signs = {leg: -1 for leg in ("FL", "FR", "RL", "RR")}
        self.config.gait_forward_signs = signs
        base = self.config.angles_to_targets(
            {
                (leg, joint): angle
                for leg in signs
                for joint, angle in ((1, 4.0), (2, 45.0), (3, 90.0))
            }
        )

        for amplitude in (0.0, 0.35, 1.0):
            for phase in (0.0, 0.1, 0.25, 0.49, 0.5, 0.75, 0.9, 0.999):
                shared, support = policy.sim_trot_targets(
                    phase, amplitude, signs
                )
                python = robot.gait_targets(
                    phase, base, gait, amplitude_scale=amplitude
                )
                expected_support = (
                    {"FL", "RR"} if phase < 0.5 else {"FR", "RL"}
                )
                self.assertEqual(support, expected_support)
                for leg in signs:
                    for joint in (1, 2, 3):
                        servo_id = self.config.joint(leg, joint).servo_id
                        expected = self.config.position_to_angle(
                            leg, joint, python[servo_id]
                        )
                        self.assertAlmostEqual(
                            shared[(leg, joint)], expected, delta=0.05
                        )

    def test_shared_c_trot_travel_scale_preserves_forward_policy(self) -> None:
        policy = SharedGaitPolicy()
        signs = self.config.gait_forward_signs
        for phase in (0.0, 0.1, 0.35, 0.5, 0.85, 0.999):
            expected, expected_support = policy.sim_trot_targets(
                phase, 1.0, signs
            )
            actual, actual_support = policy.trot_targets(
                phase, 1.0, 1.0, signs
            )
            self.assertEqual(actual_support, expected_support)
            for key, expected_angle in expected.items():
                self.assertAlmostEqual(actual[key], expected_angle, delta=1e-5)

    def test_shared_c_in_place_trot_has_no_sagittal_travel(self) -> None:
        policy = SharedGaitPolicy()
        robot = SpotRobot(RecordingBus(), self.config)
        signs = self.config.gait_forward_signs

        saw_lift = False
        for phase in (0.0, 0.1, 0.25, 0.35, 0.5, 0.6, 0.85, 0.999):
            targets, support = policy.trot_targets(
                phase, 1.0, 0.0, signs
            )
            expected_support = (
                {"FL", "RR"} if phase < 0.5 else {"FR", "RL"}
            )
            self.assertEqual(support, expected_support)
            for leg in signs:
                forward, down = robot.leg_forward_kinematics(
                    targets[(leg, 2)], targets[(leg, 3)]
                )
                self.assertAlmostEqual(forward, 0.0, delta=1e-5)
                saw_lift |= down < math.sqrt(2.0) - 0.05
        self.assertTrue(saw_lift)

        with self.assertRaises(ValueError):
            policy.trot_targets(0.5, 1.0, 1.01, signs)

    def test_shared_c_smootherstep_is_bounded_and_matches_python(self) -> None:
        policy = SharedGaitPolicy()
        previous = 0.0
        for step in range(-20, 1021):
            progress = step / 1000.0
            actual = policy.smootherstep(progress)
            expected_progress = max(0.0, min(1.0, progress))
            expected = SpotRobot._smoothstep(expected_progress)
            self.assertGreaterEqual(actual, 0.0)
            self.assertLessEqual(actual, 1.0)
            self.assertGreaterEqual(actual + 1e-7, previous)
            self.assertAlmostEqual(actual, expected, delta=2e-6)
            previous = actual

        # Regression for the 1200/1600 ms STM32 ramp frame that previously
        # rounded to 1.004 and was rejected as a configuration error.
        self.assertLessEqual(policy.smootherstep(0.96), 1.0)

    def test_shared_c_jump_starts_and_ends_at_stand(self) -> None:
        policy = SharedGaitPolicy()
        signs = self.config.gait_forward_signs
        start, start_support = policy.jump_targets(0.0, 0.0, signs)
        end, end_support = policy.jump_targets(1.0, 0.0, signs)

        self.assertEqual(start_support, {"FL", "FR", "RL", "RR"})
        self.assertEqual(end_support, start_support)
        for leg in signs:
            self.assertAlmostEqual(start[(leg, 1)], 0.0, delta=0.01)
            self.assertAlmostEqual(start[(leg, 2)], 45.0, delta=0.01)
            self.assertAlmostEqual(start[(leg, 3)], 90.0, delta=0.01)
            for joint in (1, 2, 3):
                self.assertAlmostEqual(
                    end[(leg, joint)], start[(leg, joint)], delta=0.01
                )

    def test_shared_c_in_place_jump_moves_all_legs_together(self) -> None:
        policy = SharedGaitPolicy()
        signs = self.config.gait_forward_signs
        for phase in (0.30, 0.42, 0.55, 0.68, 0.82):
            targets, _ = policy.jump_targets(phase, 0.0, signs)
            reference = tuple(targets[("FL", joint)] for joint in (1, 2, 3))
            for leg in ("FR", "RL", "RR"):
                actual = tuple(targets[(leg, joint)] for joint in (1, 2, 3))
                for value, expected in zip(actual, reference):
                    self.assertAlmostEqual(value, expected, delta=0.01)

        airborne, support = policy.jump_targets(0.50, 0.0, signs)
        self.assertEqual(support, set())
        self.assertGreater(airborne[("FL", 3)], 44.0)
        _, landed_support = policy.jump_targets(0.70, 0.0, signs)
        self.assertEqual(landed_support, {"FL", "FR", "RL", "RR"})

    def test_shared_c_jump_forward_parameter_respects_leg_mirroring(self) -> None:
        policy = SharedGaitPolicy()
        robot = SpotRobot(RecordingBus(), self.config)
        signs = self.config.gait_forward_signs
        in_place, _ = policy.jump_targets(0.60, 0.0, signs)
        forward, _ = policy.jump_targets(0.60, 0.08, signs)

        for leg in signs:
            base_x, _ = robot.leg_forward_kinematics(
                in_place[(leg, 2)], in_place[(leg, 3)]
            )
            moved_x, _ = robot.leg_forward_kinematics(
                forward[(leg, 2)], forward[(leg, 3)]
            )
            normalized_change = (moved_x - base_x) * signs[leg]
            self.assertGreater(normalized_change, 0.0)

        with self.assertRaises(ValueError):
            policy.jump_targets(0.5, 0.31, signs)

    def test_shared_c_balance_uses_support_and_body_axes(self) -> None:
        policy = SharedGaitPolicy()
        signs = {leg: -1 for leg in ("FL", "FR", "RL", "RR")}
        targets, support = policy.sim_trot_targets(0.25, 1.0, signs)
        balanced = policy.balance_targets(
            targets,
            sample=ImuSample(
                roll=0.10,
                pitch=0.0,
                roll_rate=0.0,
                pitch_rate=0.0,
            ),
            support_legs=support,
            forward_signs=signs,
            kp=1.0,
            kd=0.04,
            leg_length_limit=0.15,
            mode="contact-aware",
            j1_gain=5.0,
            j1_limit=5.0,
            foot_placement_gain=0.0,
            foot_placement_limit=0.08,
        )

        self.assertGreater(balanced[("FL", 1)], targets[("FL", 1)])
        self.assertLess(balanced[("RR", 1)], targets[("RR", 1)])
        self.assertGreater(balanced[("FR", 1)], targets[("FR", 1)])
        self.assertLess(balanced[("RL", 1)], targets[("RL", 1)])

        robot = SpotRobot(RecordingBus(), self.config)
        for leg in ("FL", "RL"):
            before = robot.leg_forward_kinematics(
                targets[(leg, 2)], targets[(leg, 3)]
            )[1]
            after = robot.leg_forward_kinematics(
                balanced[(leg, 2)], balanced[(leg, 3)]
            )[1]
            self.assertAlmostEqual(after, before - 0.10, delta=0.001)
        for leg in ("FR", "RR"):
            before = robot.leg_forward_kinematics(
                targets[(leg, 2)], targets[(leg, 3)]
            )[1]
            after = robot.leg_forward_kinematics(
                balanced[(leg, 2)], balanced[(leg, 3)]
            )[1]
            self.assertAlmostEqual(after, before + 0.10, delta=0.001)

    def test_planar_leg_ik_round_trip(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        for upper, knee in ((45.0, 90.0), (35.0, 105.0), (55.0, 80.0)):
            foot = robot.leg_forward_kinematics(upper, knee)
            restored = robot.leg_inverse_kinematics(*foot)
            self.assertAlmostEqual(restored[0], upper, places=7)
            self.assertAlmostEqual(restored[1], knee, places=7)

    def test_forward_ik_stance_pushes_ground_backward(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=4.0,
            hip_amplitude=8.0,
            lift_amplitude=10.0,
            crouch_amplitude=0,
            duty_factor=0.65,
        )
        base = self.config.stand45_targets()

        # Loaded hardware trials establish this progression as forward: during
        # stance each planted foot travels front-to-rear relative to the body,
        # so ground reaction drives the body forward.
        for leg in ("FL", "RR"):
            servo_id = self.config.joint(leg, 2).servo_id
            offset = robot.PHASE_OFFSETS["trot"][leg]
            stance_start = robot.gait_targets((-offset) % 1.0, base, gait)
            stance_end = robot.gait_targets(
                (gait.duty_factor - offset) % 1.0, base, gait
            )
            self.assertGreater(stance_start[servo_id], stance_end[servo_id])
        for leg in ("FR", "RL"):
            servo_id = self.config.joint(leg, 2).servo_id
            offset = robot.PHASE_OFFSETS["trot"][leg]
            stance_start = robot.gait_targets((-offset) % 1.0, base, gait)
            stance_end = robot.gait_targets(
                (gait.duty_factor - offset) % 1.0, base, gait
            )
            self.assertLess(stance_start[servo_id], stance_end[servo_id])

    def test_each_foot_moves_rearward_on_ground_and_forward_in_air(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=2.0,
            hip_amplitude=8.0,
            lift_amplitude=20.0,
            crouch_amplitude=0.0,
            duty_factor=0.82,
            stance_j1_angle=4.0,
            stance_j2_angle=25.0,
            stance_j3_angle=50.0,
        )
        base = self.config.angles_to_targets(
            {
                (leg, joint_number): angle
                for leg in ("FL", "FR", "RL", "RR")
                for joint_number, angle in ((1, 4.0), (2, 25.0), (3, 50.0))
            }
        )

        def body_foot(leg: str, local_phase: float) -> tuple[float, float]:
            global_phase = (
                local_phase - SpotRobot.PHASE_OFFSETS["trot"][leg]
            ) % 1.0
            targets = robot.gait_targets(global_phase, base, gait)
            j2 = self.config.joint(leg, 2).servo_id
            j3 = self.config.joint(leg, 3).servo_id
            upper = self.config.position_to_angle(leg, 2, targets[j2])
            knee = self.config.position_to_angle(leg, 3, targets[j3])
            forward, down = robot.leg_forward_kinematics(upper, knee)
            return forward * self.config.gait_forward_signs[leg], down

        for leg in ("FL", "FR", "RL", "RR"):
            touchdown = body_foot(leg, 0.0)
            toe_off = body_foot(leg, gait.duty_factor)
            mid_swing = body_foot(
                leg, gait.duty_factor + (1.0 - gait.duty_factor) / 2.0
            )
            next_touchdown = body_foot(leg, 1.0 - 1e-6)
            self.assertGreater(touchdown[0], toe_off[0])
            self.assertGreater(next_touchdown[0], mid_swing[0])
            self.assertLess(mid_swing[1], touchdown[1])

    def test_swing_foot_lifts_before_moving_forward(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=2.0,
            hip_amplitude=8.0,
            lift_amplitude=20.0,
            crouch_amplitude=0.0,
            duty_factor=0.58,
            stance_j1_angle=8.0,
            stance_j2_angle=25.0,
            stance_j3_angle=50.0,
        )
        base = self.config.angles_to_targets(
            {
                (leg, joint): angle
                for leg in ("FL", "FR", "RL", "RR")
                for joint, angle in ((1, 8.0), (2, 25.0), (3, 50.0))
            }
        )

        def foot_at_swing_progress(progress: float) -> tuple[float, float]:
            phase = gait.duty_factor + (1.0 - gait.duty_factor) * progress
            targets = robot.gait_targets(phase, base, gait)
            upper = self.config.position_to_angle("FL", 2, targets[2])
            knee = self.config.position_to_angle("FL", 3, targets[3])
            forward, down = robot.leg_forward_kinematics(upper, knee)
            return forward * self.config.gait_forward_signs["FL"], down

        toe_off = foot_at_swing_progress(0.0)
        lifted = foot_at_swing_progress(0.20)
        transferred = foot_at_swing_progress(0.80)
        touchdown = foot_at_swing_progress(1.0 - 1e-6)
        self.assertAlmostEqual(lifted[0], toe_off[0], delta=0.002)
        self.assertLess(lifted[1], toe_off[1])
        self.assertGreater(transferred[0], lifted[0])
        self.assertAlmostEqual(transferred[1], lifted[1], delta=0.002)
        self.assertAlmostEqual(touchdown[0], transferred[0], delta=0.002)
        self.assertGreater(touchdown[1], transferred[1])

    def test_power_gait_transfers_load_before_diagonal_liftoff(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = resolve_gait(parse_args(["walk", "--preset", "power"]))
        base = self.config.angles_to_targets(
            {
                (leg, joint_number): angle
                for leg in ("FL", "FR", "RL", "RR")
                for joint_number, angle in (
                    (1, gait.stance_j1_angle),
                    (2, gait.stance_j2_angle),
                    (3, gait.stance_j3_angle),
                )
            }
        )

        # During the four-foot overlap, transfer toward FL+RR before FR+RL
        # lifts at phase 0.08.
        transfer = robot.gait_targets(0.06, base, gait)

        def angles_and_down(leg: str) -> tuple[float, float]:
            ids = {
                joint: self.config.joint(leg, joint).servo_id
                for joint in (1, 2, 3)
            }
            angles = {
                joint: self.config.position_to_angle(
                    leg, joint, transfer[servo_id]
                )
                for joint, servo_id in ids.items()
            }
            down = robot.leg_forward_kinematics(angles[2], angles[3])[1]
            return angles[1], down

        fl_j1, fl_down = angles_and_down("FL")
        fr_j1, fr_down = angles_and_down("FR")
        self.assertGreater(fl_j1, gait.stance_j1_angle)
        self.assertLess(fr_j1, gait.stance_j1_angle)
        self.assertGreater(fl_down, fr_down)

        # The transfer reaches full amplitude at liftoff and stays there
        # instead of peaking halfway through swing.
        self.assertAlmostEqual(
            robot._trot_support_transfer(0.08, gait.duty_factor), 1.0
        )
        self.assertAlmostEqual(
            robot._trot_support_transfer(0.25, gait.duty_factor), 1.0
        )
        self.assertAlmostEqual(
            robot._trot_support_transfer(0.58, gait.duty_factor), -1.0
        )

        # Once FR+RL enter swing, their feet must be above FL+RR.
        swing = robot.gait_targets(0.25, base, gait)

        def down_at(leg: str) -> float:
            j2 = self.config.joint(leg, 2).servo_id
            j3 = self.config.joint(leg, 3).servo_id
            upper = self.config.position_to_angle(leg, 2, swing[j2])
            knee = self.config.position_to_angle(leg, 3, swing[j3])
            return robot.leg_forward_kinematics(upper, knee)[1]

        self.assertLess(down_at("FR"), down_at("FL"))
        self.assertLess(down_at("RL"), down_at("RR"))

    def test_trot_has_four_foot_support_before_each_diagonal_swing(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(
            pattern="trot",
            period=2.0,
            hip_amplitude=8.0,
            lift_amplitude=10.0,
            crouch_amplitude=0,
            speed=60,
            acceleration=30,
            duty_factor=0.65,
        )
        base = self.config.stand45_targets()

        all_ground = robot.gait_targets(0.0, base, gait)
        for leg in ("FL", "FR", "RL", "RR"):
            upper_id = self.config.joint(leg, 2).servo_id
            knee_id = self.config.joint(leg, 3).servo_id
            base_foot = robot.leg_forward_kinematics(45.0, 90.0)
            upper = self.config.position_to_angle(leg, 2, all_ground[upper_id])
            knee = self.config.position_to_angle(leg, 3, all_ground[knee_id])
            foot = robot.leg_forward_kinematics(upper, knee)
            self.assertAlmostEqual(foot[1], base_foot[1], delta=0.002)

        pair_b_swing = robot.gait_targets(0.25, base, gait)
        for leg in ("FL", "RR"):
            upper_id = self.config.joint(leg, 2).servo_id
            knee_id = self.config.joint(leg, 3).servo_id
            upper = self.config.position_to_angle(leg, 2, pair_b_swing[upper_id])
            knee = self.config.position_to_angle(leg, 3, pair_b_swing[knee_id])
            self.assertAlmostEqual(
                robot.leg_forward_kinematics(upper, knee)[1],
                robot.leg_forward_kinematics(45.0, 90.0)[1],
                delta=0.002,
            )
        for leg in ("FR", "RL"):
            upper_id = self.config.joint(leg, 2).servo_id
            knee_id = self.config.joint(leg, 3).servo_id
            upper = self.config.position_to_angle(leg, 2, pair_b_swing[upper_id])
            knee = self.config.position_to_angle(leg, 3, pair_b_swing[knee_id])
            self.assertLess(
                robot.leg_forward_kinematics(upper, knee)[1],
                robot.leg_forward_kinematics(45.0, 90.0)[1] - 0.01,
            )

    def test_zero_amplitude_returns_the_stance_without_a_jump(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        gait = GaitParameters(pattern="trot", duty_factor=0.5)
        base = self.config.stand45_targets()
        self.assertEqual(
            robot.gait_targets(0.37, base, gait, amplitude_scale=0.0),
            base,
        )

    def test_walk_streams_simple_trot_positions_without_status_checks(self) -> None:
        bus = RecordingBus()
        robot = SpotRobot(bus, self.config)
        gait = GaitParameters(
            pattern="trot",
            period=2.0,
            hip_amplitude=5.0,
            lift_amplitude=7.0,
            crouch_amplitude=0,
            speed=20,
            acceleration=10,
            duty_factor=0.5,
            control_rate=20.0,
        )
        clock = FakeClock()
        with (
            patch.object(robot, "set_torque"),
            patch("servo.cli.time.monotonic", side_effect=clock.monotonic),
            patch("servo.cli.time.sleep", side_effect=clock.sleep),
        ):
            run_walk(robot, gait, cycles=1)
        # Initial profile + 2 seconds at 20Hz + final stand45 command.
        self.assertEqual(len(bus.calls), 42)
        self.assertEqual(bus.calls[0][0:2], (41, 7))
        self.assertEqual(bus.calls[-1][0:2], (41, 7))
        self.assertTrue(
            all(
                address == 42 and item_size == 2 and len(values) == 12
                for address, item_size, values in bus.calls[1:-1]
            )
        )

    def test_walk_load_profile_round_robins_j2_j3_without_extra_writes(self) -> None:
        bus = RecordingBus()
        robot = SpotRobot(bus, self.config)
        gait = GaitParameters(
            pattern="trot",
            period=1.0,
            hip_amplitude=5.0,
            lift_amplitude=7.0,
            crouch_amplitude=0,
            speed=20,
            acceleration=10,
            duty_factor=0.5,
            control_rate=20.0,
        )
        state = ServoState(
            position=2048,
            speed=12,
            load=-32,
            voltage=12.0,
            temperature=30,
            hardware_error=0,
            moving=True,
            current=50,
        )
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "loads.csv"
            with (
                patch.object(robot, "set_torque"),
                patch("servo.cli.STS3215.read_state", return_value=state) as read,
                patch("servo.cli.time.monotonic", side_effect=clock.monotonic),
                patch("servo.cli.time.sleep", side_effect=clock.sleep),
            ):
                run_walk(
                    robot,
                    gait,
                    cycles=1,
                    profile_path=profile,
                    load_rate=1.0,
                )
            rows = profile.read_text(encoding="utf-8").splitlines()

        self.assertEqual(read.call_count, 8)
        self.assertEqual(len(rows), 9)
        self.assertIn("planned_contact", rows[0])
        self.assertIn("position_error", rows[0])
        self.assertEqual(len(bus.calls), 22)

    def test_power_stance_is_extended_but_not_singular(self) -> None:
        args = parse_args(["walk", "--preset", "power"])
        gait = resolve_gait(args)
        robot = SpotRobot(RecordingBus(), self.config)

        power_foot = robot.leg_forward_kinematics(
            gait.stance_j2_angle, gait.stance_j3_angle
        )
        standard_foot = robot.leg_forward_kinematics(45.0, 90.0)
        self.assertAlmostEqual(power_foot[0], 0.0, places=7)
        self.assertGreater(power_foot[1], standard_foot[1])
        self.assertEqual(
            (gait.stance_j2_angle, gait.stance_j3_angle), (30.0, 60.0)
        )
        self.assertEqual(gait.duty_factor, 0.58)
        self.assertLess(gait.hip_amplitude, 19.3)

        base = self.config.angles_to_targets(
            {
                (leg, joint_number): angle
                for leg in ("FL", "FR", "RL", "RR")
                for joint_number, angle in ((2, 30.0), (3, 60.0))
            }
        )
        for phase in (0.0, 0.25, 0.5, 0.75):
            self.assertEqual(len(robot.gait_targets(phase, base, gait)), 12)

    def test_positive_stance_j1_abducts_all_four_legs(self) -> None:
        outward_angle = 4.0
        targets = self.config.angles_to_targets(
            {
                (leg, 1): outward_angle
                for leg in ("FL", "FR", "RL", "RR")
            }
        )
        actual = {
            leg: self.config.position_to_angle(
                leg, 1, targets[self.config.joint(leg, 1).servo_id]
            )
            for leg in ("FL", "FR", "RL", "RR")
        }
        for leg in ("FL", "FR", "RL", "RR"):
            self.assertAlmostEqual(actual[leg], 4.0, delta=0.05)

        raw_deltas = {
            leg: targets[self.config.joint(leg, 1).servo_id]
            - self.config.joint(leg, 1).center
            for leg in ("FL", "FR", "RL", "RR")
        }
        self.assertGreater(raw_deltas["FL"], 0)
        self.assertLess(raw_deltas["FR"], 0)
        self.assertLess(raw_deltas["RL"], 0)
        self.assertGreater(raw_deltas["RR"], 0)

    def test_calibration_and_pose_round_trip(self) -> None:
        self.config.set_joint_center(2, 2053)
        self.assertEqual(self.config.joint("FL", 2).offset, 5)
        self.assertEqual(self.config.pose("neutral")[2], 2053)
        positions = self.config.pose("stand45")
        positions[1] = 2100
        self.config.set_pose("custom", positions)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            self.config.save(path)
            loaded = SpotConfig.load(path)
            calibration_report = (Path(directory) / "servo_calibration.md").read_text()
            pose_report = (Path(directory) / "servo_poses.md").read_text()
        self.assertEqual(loaded.joint("FL", 2).center, 2053)
        self.assertEqual(loaded.pose("custom")[1], 2100)
        self.assertIn("| FL | 2 | 2 | +5 | 2053 |", calibration_report)
        self.assertIn("| custom |", pose_report)

    def test_calibrated_pose_names_cannot_be_overwritten(self) -> None:
        targets = self.config.pose("neutral")
        for name in ("neutral", "stand", "stand45", "landing"):
            with self.assertRaisesRegex(ValueError, "reserved"):
                self.config.set_pose(name, targets)

    def test_capture_stand_saves_present_positions_as_centers(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            config = SpotConfig.load(CONFIG)
            config.save(path)
            positions = {
                servo_id: 2000 + servo_id for servo_id in config.servo_ids
            }

            class CaptureRobot:
                def __init__(self) -> None:
                    self.config = config
                    self.required_all = False

                def require_all(self) -> None:
                    self.required_all = True

                def read_positions(self) -> dict[int, int]:
                    return positions

            robot = CaptureRobot()
            captured = capture_stand(robot)
            loaded = SpotConfig.load(path)

        self.assertTrue(robot.required_all)
        self.assertEqual(captured, positions)
        self.assertEqual(loaded.pose("neutral"), positions)
        for servo_id, position in positions.items():
            joint = next(
                joint for joint in loaded.joints if joint.servo_id == servo_id
            )
            self.assertEqual(joint.center, position)

    def test_landing_uses_same_canonical_angles_on_every_leg(self) -> None:
        self.assertEqual(
            self.config.canonical_poses["landing"], {2: 40.0, 3: 130.0}
        )
        targets = self.config.landing_targets()
        for leg in ("FL", "FR", "RL", "RR"):
            for joint_number, expected_angle in ((2, 40.0), (3, 130.0)):
                servo_id = self.config.joint(leg, joint_number).servo_id
                angle = self.config.position_to_angle(
                    leg, joint_number, targets[servo_id]
                )
                self.assertAlmostEqual(angle, expected_angle, delta=0.05)
        self.assertEqual(130.0 - 40.0, 90.0)

    def test_wait_until_stopped_verifies_final_positions(self) -> None:
        robot = SpotRobot(RecordingBus(), self.config)
        targets = self.config.pose("neutral")
        moving = {
            servo_id: ServoState(
                position=position,
                speed=0,
                load=0,
                voltage=12.0,
                temperature=30,
                hardware_error=0,
                moving=True,
                current=0,
            )
            for servo_id, position in targets.items()
        }
        stopped = {
            servo_id: ServoState(
                position=position,
                speed=0,
                load=0,
                voltage=12.0,
                temperature=30,
                hardware_error=0,
                moving=False,
                current=0,
            )
            for servo_id, position in targets.items()
        }
        with patch.object(robot, "read_states", side_effect=[moving, stopped]):
            result = robot.wait_until_stopped(
                targets, timeout=1.0, poll_interval=0.001
            )
        self.assertEqual(result, stopped)


class CommandLineTest(unittest.TestCase):
    def test_status_command_uses_explicit_port(self) -> None:
        args = parse_args(["--port", "/dev/test-urt2", "status"])
        self.assertEqual(args.command, "status")
        self.assertEqual(resolve_port(args.port), "/dev/test-urt2")

    def test_contacts_command_has_safe_observation_defaults(self) -> None:
        args = parse_args(["contacts"])
        self.assertEqual(args.command, "contacts")
        self.assertEqual(args.baseline, 2.0)
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.threshold, 24)
        self.assertEqual(args.release, 8)

    def test_analyze_loads_command_does_not_require_a_serial_port(self) -> None:
        args = parse_args(["analyze-loads", "loads.csv"])
        self.assertEqual(args.profile, Path("loads.csv"))

    def test_pose_command_needs_no_confirmation_flag(self) -> None:
        args = parse_args(["pose", "stand45"])
        self.assertEqual(args.name, "stand45")

    def test_original_apply_pose_command_is_available_as_alias(self) -> None:
        args = parse_args(["apply-pose", "landing"])
        self.assertEqual(args.command, "apply-pose")
        self.assertEqual(args.name, "landing")

    def test_pose_stand45_uses_current_calibration_not_saved_raw_pose(self) -> None:
        config = SpotConfig.load(CONFIG)

        class Robot:
            def __init__(self) -> None:
                self.config = config

        with patch("servo.cli.apply_targets") as apply:
            apply_pose(
                Robot(),
                "stand45",
                speed=100,
                acceleration=5,
                timeout=10,
                tolerance=30,
            )
        self.assertEqual(apply.call_args.args[1], config.stand45_targets())
        self.assertNotEqual(apply.call_args.args[1], config.pose("stand45"))

    def test_pose_landing_uses_current_calibration_not_saved_raw_pose(self) -> None:
        config = SpotConfig.load(CONFIG)

        class Robot:
            def __init__(self) -> None:
                self.config = config

        with patch("servo.cli.apply_targets") as apply:
            apply_pose(
                Robot(),
                "landing",
                speed=100,
                acceleration=5,
                timeout=10,
                tolerance=30,
            )
        self.assertEqual(apply.call_args.args[1], config.landing_targets())
        self.assertNotEqual(apply.call_args.args[1], config.pose("landing"))

    def test_capture_stand_command_and_alias_are_available(self) -> None:
        self.assertEqual(parse_args(["capture-stand"]).command, "capture-stand")
        self.assertEqual(parse_args(["save-stand"]).command, "save-stand")
        self.assertEqual(parse_args(["capture-stand", "--leg", "RL"]).leg, "RL")

    def test_leg_scoped_torque_and_stand_commands_are_available(self) -> None:
        self.assertEqual(parse_args(["relax", "--leg", "FL"]).leg, "FL")
        self.assertEqual(parse_args(["hold", "--leg", "RR"]).leg, "RR")
        self.assertEqual(parse_args(["stand", "--leg", "RL"]).leg, "RL")

    def test_swap_ids_command_uses_temporary_id(self) -> None:
        args = parse_args(["swap-ids", "9", "12", "--temp-id", "13"])
        self.assertEqual((args.first_id, args.second_id, args.temp_id), (9, 12, 13))

    def test_landing_is_a_direct_stance_command(self) -> None:
        args = parse_args(["landing", "--speed", "100", "--accel", "5"])
        self.assertEqual(args.command, "landing")
        self.assertEqual(args.speed, 100)
        self.assertEqual(args.accel, 5)

    def test_change_id_command_defaults_to_full_scan(self) -> None:
        args = parse_args(["change-id", "1", "12"])
        self.assertEqual((args.old_id, args.new_id), (1, 12))
        self.assertEqual(args.scan_max, 253)

    def test_walk_defaults_to_diagonal_trot(self) -> None:
        args = parse_args(["walk"])
        gait = resolve_gait(args)
        self.assertEqual(gait.pattern, "trot")
        self.assertEqual(gait.duty_factor, 0.65)
        self.assertEqual(gait.control_rate, 30.0)
        self.assertEqual(gait.speed, 60)
        self.assertEqual(gait.period, 4.0)
        self.assertEqual(gait.hip_amplitude, 8.8)

    def test_power_preset_can_override_stance_and_duty(self) -> None:
        args = parse_args(
            [
                "walk",
                "--preset",
                "power",
                "--stance-j1",
                "4",
                "--stance-j2",
                "32",
                "--stance-j3",
                "64",
                "--duty",
                "0.78",
            ]
        )
        gait = resolve_gait(args)
        self.assertEqual(gait.stance_j1_angle, 4.0)
        self.assertEqual(gait.stance_j2_angle, 32.0)
        self.assertEqual(gait.stance_j3_angle, 64.0)
        self.assertEqual(gait.duty_factor, 0.78)

    def test_walk_load_profile_options_are_available(self) -> None:
        args = parse_args(
            [
                "walk",
                "--profile-output",
                "loads.csv",
                "--load-rate",
                "5",
                "--load-baseline",
                "baseline.json",
            ]
        )
        self.assertEqual(args.profile_output, Path("loads.csv"))
        self.assertEqual(args.load_rate, 5.0)
        self.assertEqual(args.load_baseline, Path("baseline.json"))

    def test_direction_configuration_is_saved(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            config = SpotConfig.load(CONFIG)
            config.save(path)
            with patch("builtins.input", side_effect=["-1"] * 12):
                configure_directions(config)
            loaded = SpotConfig.load(path)
        self.assertEqual(loaded.directions["FL"], (-1, -1))
        self.assertEqual(loaded.directions["RR"], (-1, -1))
        self.assertEqual(loaded.joint("FL", 1).direction, -1)

    def test_mapping_configuration_preserves_pose_by_joint(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "joints.json"
            config = SpotConfig.load(CONFIG)
            config.save(path)
            answers = ["12 11 10", "9 8 7", "6 5 4", "3 2 1"]
            with patch("builtins.input", side_effect=answers):
                configure_mapping(config)
            loaded = SpotConfig.load(path)
        self.assertEqual(loaded.joint("FL", 1).servo_id, 12)
        self.assertEqual(loaded.pose("landing")[12], 2084)


class ServoIdChangeTest(unittest.TestCase):
    def test_swap_ids_uses_temporary_address_and_preserves_id_set(self) -> None:
        bus = IdChangeBus(9, 12)
        swap_servo_ids_on_bus(bus, 9, 12, 13, eeprom_wait=0)
        self.assertEqual(bus.ids, {9, 12})
        id_writes = [
            (servo_id, data[0])
            for servo_id, address, data, _ in bus.writes
            if address == STS3215.ADDR_ID
        ]
        self.assertEqual(id_writes, [(9, 13), (12, 9), (13, 12)])

    def test_change_id_unlocks_writes_locks_and_verifies(self) -> None:
        bus = IdChangeBus(1)
        servo = STS3215(bus, 1)
        servo.change_id(12, eeprom_wait=0)
        self.assertEqual(bus.ids, {12})
        self.assertEqual(servo.id, 12)
        self.assertEqual(
            bus.writes,
            [
                (1, STS3215.ADDR_LOCK, b"\x00", True),
                (1, STS3215.ADDR_ID, b"\x0c", True),
                (12, STS3215.ADDR_LOCK, b"\x01", True),
            ],
        )

    def test_change_id_rejects_an_id_that_already_responds(self) -> None:
        bus = IdChangeBus(1, 12)
        servo = STS3215(bus, 1)
        with self.assertRaisesRegex(RuntimeError, "already in use"):
            servo.change_id(12, eeprom_wait=0)
        self.assertEqual(bus.writes, [])


class ServoStateTest(unittest.TestCase):
    def test_state_includes_hardware_error_and_sts_load_direction(self) -> None:
        bus = StateReadBus()
        state = STS3215(bus, 1).read_state()
        self.assertEqual(bus.last_read, (1, 56, 15))
        self.assertEqual(state.position, 2050)
        self.assertEqual(state.load, -100)
        self.assertEqual(state.hardware_error, 4)
        self.assertTrue(state.moving)

    def test_diagnostics_includes_configuration_registers(self) -> None:
        values = STS3215(DiagnosticsBus(), 1).read_diagnostics()
        self.assertTrue(values.torque_enabled)
        self.assertEqual(values.acceleration, 50)
        self.assertEqual(values.goal_position, 2100)
        self.assertEqual(values.goal_speed, 1000)
        self.assertEqual(values.torque_limit, 900)


if __name__ == "__main__":
    unittest.main()
