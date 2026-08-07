"""Regression tests for the simulator-only circular-foot trot2 policy."""

import math
import sys
from pathlib import Path

from servo import SharedGaitPolicy, SpotConfig, SpotRobot

# pytest resolves its rootdir from tools/servo_tool/pyproject.toml, so the
# repository root is not importable here.  Import walk.py from its own
# directory, the same way jump.py does.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from walk import (  # noqa: E402
    CONFIG,
    LEGS,
    SIM_PRESETS,
    TROT2_DEFAULT_FOLD_J2,
    TROT2_DEFAULT_FOLD_J3,
    stance_targets,
    python_trot2_targets,
)


def configured_trot2():
    config = SpotConfig.load(CONFIG)
    config.gait_forward_signs = {leg: -1 for leg in LEGS}
    robot = SpotRobot(None, config)
    gait = SIM_PRESETS["trot2"]
    return config, robot, gait, stance_targets(config, gait)


def foot_position(config, robot, targets, leg="FL"):
    j2 = config.joint(leg, 2)
    j3 = config.joint(leg, 3)
    upper = config.position_to_angle(leg, 2, targets[j2.servo_id])
    knee = config.position_to_angle(leg, 3, targets[j3.servo_id])
    return (*robot.leg_forward_kinematics(upper, knee), upper, knee)


def test_trot2_circle_apex_reaches_folded_l2_pose():
    config, robot, gait, base = configured_trot2()
    apex_phase = gait.duty_factor + (1.0 - gait.duty_factor) * 0.5
    targets, support = python_trot2_targets(
        config,
        robot,
        base,
        gait,
        apex_phase,
        1.0,
        TROT2_DEFAULT_FOLD_J2,
        TROT2_DEFAULT_FOLD_J3,
    )
    _, _, upper, knee = foot_position(config, robot, targets)
    assert abs(upper - TROT2_DEFAULT_FOLD_J2) < 0.1
    assert abs(knee - TROT2_DEFAULT_FOLD_J3) < 0.1
    assert support == {"FR", "RL"}


def test_trot2_swing_toe_stays_on_upper_semicircle():
    config, robot, gait, base = configured_trot2()
    base_forward, ground_down, _, _ = foot_position(config, robot, base)
    folded_forward, folded_down = robot.leg_forward_kinematics(
        TROT2_DEFAULT_FOLD_J2,
        TROT2_DEFAULT_FOLD_J3,
    )
    direction = config.gait_forward_signs["FL"]
    center = (folded_forward - base_forward) / direction
    radius = ground_down - folded_down

    for index in range(21):
        progress = index / 20.0
        phase = gait.duty_factor + (1.0 - gait.duty_factor) * progress
        targets, _ = python_trot2_targets(
            config,
            robot,
            base,
            gait,
            phase,
            1.0,
            TROT2_DEFAULT_FOLD_J2,
            TROT2_DEFAULT_FOLD_J3,
        )
        forward, down, _, _ = foot_position(config, robot, targets)
        travel = (forward - base_forward) / direction
        radial_error = abs(math.hypot(travel - center, down - ground_down) - radius)
        assert radial_error < 0.002
        assert down <= ground_down + 0.002


def test_trot2_keeps_each_diagonal_pair_synchronized():
    config, robot, gait, base = configured_trot2()
    targets, _ = python_trot2_targets(
        config,
        robot,
        base,
        gait,
        0.63,
        1.0,
        TROT2_DEFAULT_FOLD_J2,
        TROT2_DEFAULT_FOLD_J3,
    )
    angles = {
        leg: foot_position(config, robot, targets, leg)[2:]
        for leg in LEGS
    }
    assert angles["FL"] == angles["RR"]
    assert angles["FR"] == angles["RL"]


def test_shared_c_trot2_matches_python_reference():
    config, robot, gait, base = configured_trot2()
    policy = SharedGaitPolicy()
    for phase in (0.0, 0.13, 0.25, 0.50, 0.63, 0.75, 0.99):
        for amplitude in (0.0, 0.4, 1.0):
            expected, expected_support = python_trot2_targets(
                config,
                robot,
                base,
                gait,
                phase,
                amplitude,
                TROT2_DEFAULT_FOLD_J2,
                TROT2_DEFAULT_FOLD_J3,
            )
            actual, actual_support = policy.trot2_targets(
                phase,
                amplitude,
                TROT2_DEFAULT_FOLD_J2,
                TROT2_DEFAULT_FOLD_J3,
                config.gait_forward_signs,
            )
            assert actual_support == expected_support
            for leg in LEGS:
                for joint in (1, 2, 3):
                    servo_id = config.joint(leg, joint).servo_id
                    expected_angle = config.position_to_angle(
                        leg, joint, expected[servo_id]
                    )
                    assert abs(actual[(leg, joint)] - expected_angle) < 0.06


def test_shared_c_trot2_hardware_signs_stay_inside_servo_limits():
    config = SpotConfig.load(CONFIG)
    policy = SharedGaitPolicy()
    hardware_signs = {"FL": 1, "FR": 1, "RL": -1, "RR": -1}
    for index in range(101):
        targets, _ = policy.trot2_targets(
            index / 100.0,
            1.0,
            TROT2_DEFAULT_FOLD_J2,
            TROT2_DEFAULT_FOLD_J3,
            hardware_signs,
        )
        # Uses the measured centers, directions and min/max raw positions.
        config.angles_to_targets(targets)
