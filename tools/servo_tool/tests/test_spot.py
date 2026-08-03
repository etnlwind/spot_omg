import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from servo import GaitParameters, ServoState, SpotConfig, SpotRobot, STS3215
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
        base = self.config.pose("stand45")
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

        def normalized(leg: str, joint_number: int) -> int:
            servo_id = self.config.joint(leg, joint_number).servo_id
            target_angle = self.config.position_to_angle(
                leg, joint_number, targets[servo_id]
            )
            base_angle = self.config.position_to_angle(
                leg, joint_number, base[servo_id]
            )
            delta = target_angle - base_angle
            return (
                delta * self.config.gait_forward_signs[leg]
                if joint_number == 2
                else delta
            )

        self.assertEqual(normalized("FL", 2), normalized("RR", 2))
        self.assertEqual(normalized("FL", 3), normalized("RR", 3))
        self.assertEqual(normalized("FR", 2), normalized("RL", 2))
        self.assertEqual(normalized("FR", 3), normalized("RL", 3))
        self.assertEqual(normalized("FL", 2), -normalized("FR", 2))

    def test_forward_stance_restores_hardware_verified_j2_raw_direction(self) -> None:
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

        # Hardware observation: rear-leg direction was correct while both
        # front legs moved backward. Front kinematic signs therefore invert
        # the canonical trajectory without changing motor directions.
        for leg in ("FL", "RR"):
            servo_id = self.config.joint(leg, 2).servo_id
            offset = robot.PHASE_OFFSETS["trot"][leg]
            stance_start = robot.gait_targets((-offset) % 1.0, base, gait)
            stance_end = robot.gait_targets(
                (gait.duty_factor - offset) % 1.0, base, gait
            )
            self.assertLess(stance_start[servo_id], stance_end[servo_id])
        for leg in ("FR", "RL"):
            servo_id = self.config.joint(leg, 2).servo_id
            offset = robot.PHASE_OFFSETS["trot"][leg]
            stance_start = robot.gait_targets((-offset) % 1.0, base, gait)
            stance_end = robot.gait_targets(
                (gait.duty_factor - offset) % 1.0, base, gait
            )
            self.assertGreater(stance_start[servo_id], stance_end[servo_id])

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
            knee_id = self.config.joint(leg, 3).servo_id
            self.assertEqual(all_ground[knee_id], base[knee_id])

        pair_b_swing = robot.gait_targets(0.25, base, gait)
        for leg in ("FL", "RR"):
            knee_id = self.config.joint(leg, 3).servo_id
            self.assertEqual(pair_b_swing[knee_id], base[knee_id])
        for leg in ("FR", "RL"):
            knee_id = self.config.joint(leg, 3).servo_id
            self.assertNotEqual(pair_b_swing[knee_id], base[knee_id])

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
