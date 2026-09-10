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
    assert "gait_command_velocity_deg_s" in sample_body
    assert "gait_command_acceleration_deg_s2" in sample_body


def test_balance_off_keeps_observation_and_tilt_snapshot() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert "if (robot->attitude_reader != NULL &&" in gait_body
    assert "if (balance_feedback_enabled)" in gait_body
    assert "tilt_snapshot_store(robot, &trace_frame)" in gait_body
    assert gait_body.index("if (robot->attitude_reader != NULL &&") < (
        gait_body.index("if (balance_feedback_enabled)")
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


def test_bno086_bringup_keeps_cubemx_and_transport_in_sync() -> None:
    ioc = (PROJECT / "stm32-learning.ioc").read_text()
    main = (PROJECT / "Src/main.c").read_text()
    driver = (PROJECT / "Src/bno086.c").read_text()
    console = (PROJECT / "Src/app_console.c").read_text()

    assert "SPI1.CLKPolarity=SPI_POLARITY_HIGH" in ioc
    assert "SPI1.CLKPhase=SPI_PHASE_2EDGE" in ioc
    assert "SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16" in ioc
    assert "PB2.GPIO_Label=IMU_RST" in ioc
    assert "PB2.PinState=GPIO_PIN_SET" in ioc
    assert "hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH" in main
    assert "hspi1.Init.CLKPhase = SPI_PHASE_2EDGE" in main
    assert "hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16" in main

    # Production reads are interrupt-gated; only imuprobe performs blind reads.
    hal_read_start = driver.index("static int hal_read(")
    hal_read_end = driver.index("static int hal_write(", hal_read_start)
    hal_read = driver[hal_read_start:hal_read_end]
    assert "if (!interrupt_asserted())" in hal_read
    assert "total < sizeof(header) || total > len" in hal_read
    assert "chip_select();" in hal_read
    assert "chip_deselect();" in hal_read

    assert "BNO086_RESET_LOW_MS      30U" in driver
    assert "sh2_getProdIds(&imu->product_ids)" in driver
    assert "imu->product_ids.numEntries == 0U" in driver
    assert "SH2_GAME_ROTATION_VECTOR" in driver
    assert "BNO086 RESET:" in main
    assert "BNO086 SHTP:" in main
    assert "BNO086 Product ID:" in main
    assert "BNO086 Rotation Vector:" in main

    # The meter-friendly reset test must hold PB2 low long enough to observe.
    assert 'strcmp(command, "imursttest")' in console
    assert "bno086_test_set_reset(console->imu, false)" in console
    assert "HAL_Delay(5000U)" in console
    assert "bno086_test_set_reset(console->imu, true)" in console
    assert "HAL_GPIO_ReadPin(IMU_RST_GPIO_Port, IMU_RST_Pin)" in driver


def test_esp32_en_is_held_low_then_released_open_drain() -> None:
    ioc = (PROJECT / "stm32-learning.ioc").read_text()
    main = (PROJECT / "Src/main.c").read_text()
    header = (PROJECT / "Inc/main.h").read_text()

    assert "PB5.GPIO_Label=ESP32_EN" in ioc
    assert "PB5.GPIO_Mode=GPIO_MODE_OUTPUT_OD" in ioc
    assert "PB5.PinState=GPIO_PIN_RESET" in ioc
    assert "#define ESP32_EN_Pin GPIO_PIN_5" in header
    assert "#define ESP32_RESET_HOLD_MS 1000U" in main
    assert "HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET)" in main
    assert "GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD" in main
    assert "HAL_Delay(ESP32_RESET_HOLD_MS)" in main
    assert "HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_SET)" in main


def test_bno055_calibration_separates_device_profile_and_logic_zero() -> None:
    driver = (PROJECT / "Src/bno055.c").read_text()
    header = (PROJECT / "Inc/bno055.h").read_text()
    console = (PROJECT / "Src/app_console.c").read_text()
    linker = (PROJECT / "STM32F446RETX_FLASH.ld").read_text()

    assert "BNO055_OFFSET_START_ADDR" in driver
    assert "BNO055_MODE_IMUPLUS       0x08U" in driver
    assert "status.gyro != 3U || status.accel != 3U" in driver
    assert "status.mag != 3U" not in driver
    assert "BNO055_CAL_DEVICE_VALID" in driver
    assert "BNO055_CAL_LEVEL_VALID" in driver
    assert "const int16_t mapped_roll = sensor_pitch" in driver
    assert "const int16_t mapped_pitch = sensor_roll" in driver
    assert "bno055_save_level_calibration" in header
    assert 'strcmp(command, "imucal")' in console
    assert "ORIGIN = 0x08010000" in linker
    assert "LENGTH = 320K" in linker


def test_baltest_reuses_balance_policy_without_servo_io() -> None:
    policy = (PROJECT / "Inc/gait_policy.h").read_text()
    robot = (PROJECT / "Src/robot.c").read_text()
    console = (PROJECT / "Src/app_console.c").read_text()

    target_start = policy.index("static inline bool gait_policy_balance_targets(")
    target_body = policy[target_start:]
    assert "gait_policy_balance_preview(" in target_body

    preview_start = robot.index("bool robot_balance_preview(")
    preview_end = robot.index("static int16_t absolute_i16", preview_start)
    preview_body = robot[preview_start:preview_end]
    assert "shared_balance_config(robot->balance_mode)" in preview_body
    assert "gait_policy_balance_preview(" in preview_body
    assert "GAIT_POLICY_PI / 1800.0f" in preview_body
    assert "robot->bus" not in preview_body
    assert "servo" not in preview_body.lower()

    assert 'strcmp(command, "baltest")' in console
    assert "no servo command sent" in console


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


def test_continuous_drive_uses_realtime_mailbox_and_watchdog() -> None:
    robot = (PROJECT / "Src/robot.c").read_text()
    header = (PROJECT / "Inc/robot.h").read_text()
    console = (PROJECT / "Src/app_console.c").read_text()

    assert "#define ROBOT_DRIVE_WATCHDOG_MS 800U" in header
    assert "robot_drive_update_realtime" in robot
    assert "robot_drive_stop_realtime" in robot
    assert "drive_sequence_is_newer" in robot
    assert "gait_policy_drive_slew(current, target)" in robot
    assert "gait_policy_drive_walk_targets" in robot
    assert "drive_phase_q16" in robot
    assert "ROBOT_DRIVE_WATCHDOG" in robot

    # These packets must bypass the blocked foreground console while drive()
    # owns it; normal command parsing and printf are forbidden in the ISR lane.
    rx_start = console.index("void app_console_on_rx_complete(")
    rx_end = console.index("void app_console_on_uart_error(", rx_start)
    rx_body = console[rx_start:rx_end]
    assert "process_realtime_line(console)" in rx_body
    assert "console->line_ready" in rx_body
    realtime_start = console.index("static void process_realtime_line(")
    realtime_end = console.index("void app_console_init(", realtime_start)
    realtime_body = console[realtime_start:realtime_end]
    assert "robot_drive_update_realtime" in realtime_body
    assert "robot_drive_stop_realtime" in realtime_body
    assert "strtok" not in realtime_body
    assert "write_text" not in realtime_body


def test_continuous_drive_does_not_replace_bounded_trot_commands() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    gait_start = source.index("static RobotResult robot_trot_scaled(")
    gait_end = source.index("RobotResult robot_trot(", gait_start)
    gait_body = source[gait_start:gait_end]
    assert "continuous_drive || frame <= total_frames" in gait_body
    assert "continuous_drive ?" in gait_body
    assert "((frame + 1U) * period_ms) / frames_per_cycle" in gait_body

    console = (PROJECT / "Src/app_console.c").read_text()
    assert 'strcmp(command, "drive")' in console
    assert 'strcmp(command, "trot4")' in console
    assert 'strcmp(command, "turn")' in console



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


def test_forward_pose_runtime() -> None:
    """Run the real firmware routine against a bus/clock fake, without motion."""
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    sources = ["Src/forward_pose.c", "Src/robot_config.c", "Src/safety.c",
               "tests/test_forward_pose.c"]
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / "forward_pose"
        build = subprocess.run(
            [compiler, *CFLAGS, "-include", str(PROJECT / "tests/host_hal.h"),
             *[str(PROJECT / src) for src in sources], "-o", str(binary), "-lm"],
            capture_output=True, text=True,
        )
        assert build.returncode == 0, build.stderr
        run = subprocess.run([str(binary)], capture_output=True, text=True)
        assert run.returncode == 0, f"{run.stdout}\n{run.stderr}"


def test_mechanical_diagnostics_runtime() -> None:
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    sources = ["Src/mechanical_diagnostics.c", "Src/actuator_control.c",
               "Src/robot_config.c", "Src/feetech_protocol.c", "tests/test_mechanical_diagnostics.c"]
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / "mechanical_diagnostics"
        subprocess.run([compiler, *CFLAGS, "-include", str(PROJECT / "tests/host_hal.h"),
                        *[str(PROJECT / src) for src in sources], "-o", str(binary), "-lm"], check=True)
        subprocess.run([str(binary)], check=True)


def test_flight_log_power_loss_runtime() -> None:
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / "flight_log"
        subprocess.run([compiler, *CFLAGS, "-DFLIGHT_LOG_HOST_TEST", "-include",
                        str(PROJECT / "tests/flight_log_test_hal.h"),
                        str(PROJECT / "Src/flight_log.c"), str(PROJECT / "tests/test_flight_log.c"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


def test_shared_drive_diagnostics_do_not_add_motor_reads_or_flash_writes() -> None:
    source = (PROJECT / "Src/robot.c").read_text()
    start=source.index("static RobotResult robot_shared_drive(")
    body=source[start:source.index("void robot_control_idle(",start)]
    assert body.count("sample_next_joint(")==1
    assert "sts3215_read_state" not in body
    assert "flight_log" not in body and "mechanical_log" not in body
    assert body.index("gait_target_history_push(robot,positions)") < body.index("sample_next_joint(")
    assert body.index("robot->gait_support_mask=gait_policy_support_mask(nominal)") < body.index("sample_next_joint(")


def test_command_recovery_runtime() -> None:
    """Run the real firmware routine against a bus/clock fake, without motion."""
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")
    sources = ["Src/command_recovery.c", "Src/robot_config.c", "Src/safety.c",
               "tests/test_command_recovery.c"]
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / sources[-1].split("/")[-1].removesuffix(".c")
        build = subprocess.run(
            [compiler, *CFLAGS, "-include", str(PROJECT / "tests/host_hal.h"),
             *[str(PROJECT / src) for src in sources], "-o", str(binary), "-lm"],
            capture_output=True, text=True,
        )
        assert build.returncode == 0, build.stderr
        run = subprocess.run([str(binary)], capture_output=True, text=True)
        assert run.returncode == 0, f"{run.stdout}\n{run.stderr}"


def test_stow_motion_runtime():
    compiler = shutil.which("cc") or shutil.which("gcc")
    if not compiler:
        pytest.skip("no host C compiler")
    sources = ["Src/stow_motion.c", "Src/command_recovery.c", "Src/robot_config.c", "Src/safety.c", "Src/feetech_protocol.c", "tests/test_stow_motion.c"]
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / "stow"
        subprocess.run([compiler, *CFLAGS, "-include", str(PROJECT / "tests/host_hal.h"), *[str(PROJECT / s) for s in sources], "-o", str(binary), "-lm"], check=True)
        subprocess.run([str(binary)], check=True)


def test_stow_servo_coordinates_runtime():
    compiler = shutil.which("cc") or shutil.which("gcc")
    if not compiler:
        pytest.skip("no host C compiler")
    sources = ["Src/sts3215.c", "Src/robot_config.c", "Src/feetech_protocol.c", "tests/test_stow_servo_coordinates.c"]
    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / "coordinates"
        subprocess.run([compiler, *CFLAGS, "-include", str(PROJECT / "tests/host_hal.h"), *[str(PROJECT / s) for s in sources], "-o", str(binary), "-lm"], check=True)
        subprocess.run([str(binary)], check=True)
