"""Compile and run the firmware's host-testable C units.

safety.c and feetech_protocol.c carry no HAL or bus dependency on purpose, so
the decision logic that cuts torque can be exercised on a workstation rather
than only on the robot.  This wrapper exists so those tests run alongside the
Python suite instead of being a separate step someone has to remember.

    pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py \
           firmware/stm32-learning/tests/test_firmware_c.py -q
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
CFLAGS = ["-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", f"-I{PROJECT/'Inc'}"]

CASES = [
    ("safety", ["Src/safety.c", "tests/test_safety.c"]),
    ("actuator_control",
     ["Src/actuator_control.c", "Src/robot_config.c",
      "tests/test_actuator_control.c"]),
    ("feetech_protocol",
     ["Src/feetech_protocol.c", "tests/test_feetech_protocol.c"]),
]


LEGS = ("FL", "FR", "RL", "RR")
DIAGONALS = (("FL", "RR"), ("FR", "RL"))


def test_diagonal_legs_stay_together() -> None:
    """The two legs of a diagonal do the same thing at the same time.

    This is what a trot is, and it is checkable without a robot.  It used to
    depend on a per-leg sign array in robot_config.c, and a combination that
    broke it produced identical joint angles for the two legs on one side while
    the policy marked one in stance and the other in swing -- impossible, since
    equal angles put both feet at the same height, and it took several bench
    runs to recognise.  The array is gone now, but the property it had to
    satisfy is worth keeping under test.
    """
    sys.path.insert(0, str(PROJECT.parents[1] / "tools/servo_tool"))
    from servo import SharedGaitPolicy

    policy = SharedGaitPolicy()
    for step in range(20):
        phase = step / 20.0
        angles, stance = policy.trot2_targets(phase, 1.0, 78.0, 108.0)
        for left, right in DIAGONALS:
            assert (left in stance) == (right in stance), (
                f"phase {phase}: {left} and {right} disagree on stance"
            )
            for joint in (1, 2, 3):
                assert abs(angles[(left, joint)] - angles[(right, joint)]) < 0.01, (
                    f"phase {phase}: {left} J{joint} != {right} J{joint}"
                )


def test_gait_diagnostics_reuse_the_existing_round_robin_read() -> None:
    """Diagnostics must add no servo transaction to the 20 ms frame.

    sample_next_joint() was already the safety sampler: one call per normal
    gait frame and one sts3215_read_state() inside it.  Tracking/power
    diagnostics consume that returned state in memory.  Keep this structural
    contract explicit so a later diagnostic feature cannot quietly add a
    second bus read and move the absolute deadline.
    """
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert gait_body.count(
        "sample_next_joint(robot, targets, global_phase)"
    ) == 1
    assert "sts3215_read_state" not in gait_body

    sample_start = source.index(
        "static RobotResult sample_joint(", source.index("static void safety_trip")
    )
    sample_end = source.index("static RobotResult sample_next_joint(", sample_start)
    sample_body = source[sample_start:sample_end]
    assert sample_body.count("sts3215_read_state") == 1
    assert sample_body.count("actuator_diagnostics_update") == 1


def test_balance_off_keeps_observation_and_tilt_snapshot() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert "if (robot->attitude_reader != NULL &&" in gait_body
    assert "if (robot->balance_enabled)" in gait_body
    assert "tilt_snapshot_store(robot, &trace_frame)" in gait_body
    assert gait_body.index("if (robot->attitude_reader != NULL &&") < (
        gait_body.index("if (robot->balance_enabled)")
    )


def test_balance_derivative_keeps_negative_deltas_signed() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    helper_start = source.index("static int16_t balance_rate_tenths_s(")
    helper_end = source.index("static GaitPolicyBalanceConfig", helper_start)
    helper = source[helper_start:helper_end]
    assert "const int32_t delta" in helper
    assert "/ (int32_t)ROBOT_TROT_FRAME_MS" in helper
    assert "delta * INT32_C(1000)" in helper

    # The observed 0.8 -> 0.7deg transition is -1 tenth in 20ms: -5deg/s,
    # never the +120deg/s clamp seen in the old hardware log.
    assert (-1 * 1000) // 20 == -50

    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert "previous_roll_error = clamp_i16(" in gait_body
    assert "previous_pitch_error = clamp_i16(" in gait_body

    console = (PROJECT / "Src/app_console.c").read_text()
    assert '"balance=%s/%s rev=%s\\r\\n"' in console
    assert "ROBOT_CONTROL_REV" in console


def test_trot_final_frame_keeps_balance_and_does_not_jump_to_raw_stand() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert "frame > 0U && frame < total_frames" not in gait_body
    assert "if (robot->attitude_reader != NULL && frame > 0U)" in gait_body
    completion = gait_body.rindex(
        "robot->gait_elapsed_ms = HAL_GetTick() - started_at;"
    )
    assert "robot_stand_targets" not in gait_body[completion:]


def test_step_sync_monitor_cannot_pause_gait_or_add_bus_reads() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert gait_body.count("observe_step_sync(robot, targets)") == 1
    assert "wait_for_step_sync" not in source
    assert "synchronization_delay_ms" not in gait_body
    assert "HAL_Delay(ROBOT_TROT_STEP_SYNC" not in source
    assert "robot->gait_elapsed_ms = HAL_GetTick() - started_at" in gait_body



@pytest.mark.parametrize("name,sources", CASES, ids=[case[0] for case in CASES])
def test_firmware_unit(name: str, sources: list[str]) -> None:
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")

    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / name
        build = subprocess.run(
            [compiler, *CFLAGS, *[str(PROJECT / src) for src in sources],
             "-o", str(binary), "-lm"],
            capture_output=True,
            text=True,
        )
        assert build.returncode == 0, f"compile failed:\n{build.stderr}"
        # -Werror above already fails the build on a warning; surface any that
        # slipped through as diagnostics rather than letting them pass silently.
        assert not build.stderr.strip(), f"unexpected diagnostics:\n{build.stderr}"

        run = subprocess.run([str(binary)], capture_output=True, text=True)
        assert run.returncode == 0, (
            f"{name} failed:\n{run.stdout}\n{run.stderr}"
        )
