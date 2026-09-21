"""Manual IMU recovery owns input until completion and a new gesture."""
import pytest

from spot_controller.protocol import Controller, IMU_RECOVERY_ACK, IMU_RECOVERY_REVISIONS


STATE = (b"$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v6-v92 "
         b"caps=stow,gaitprofiles,attitudepd_v6 profile=attitudepd_v6\r\n")


def drain(controller):
    packets = list(controller.outbox)
    controller.outbox.clear()
    return b"".join(data for _, data in packets)


def ready():
    c = Controller()
    c.opened(0)
    assert drain(c) == b"syncstate\n"
    c.feed(STATE + b"# ", .1)
    assert drain(c) == b"read 1\n"
    c.feed(b"ID 1 voltage=11100mV\r\n# ", .2)
    assert not drain(c)
    c.sample_input(None, .3)
    return c


def begin(c):
    c.request("imurecover", 1)
    assert drain(c) == b"imurecover\n"
    assert c.snapshot()["imu_recovery_status"] == "recovering"
    assert c.imu_recovery_pending and not c.controls_enabled


def complete(c, state=STATE):
    c.feed(IMU_RECOVERY_ACK.encode() + b"\r\n# ", 2)
    assert drain(c) == b"syncstate\n"
    assert c.imu_recovery_pending and c.imu_recovery_status == "refreshing"
    c.feed(state + b"# ", 2.1)
    assert not drain(c)


@pytest.mark.parametrize("revision", sorted(IMU_RECOVERY_REVISIONS))
def test_only_supported_real_revisions_allow_manual_recovery(revision):
    c = ready(); c.state["rev"] = revision
    assert c.snapshot()["supports_imu_recovery"] and c.snapshot()["can_recover_imu"]
    c.request("imurecover", 1)
    assert drain(c) == b"imurecover\n"


@pytest.mark.parametrize("simulator,revision", [(True, "attitudepd-v6-v92"),
                          (False, "attitudepd-v4-v82"), (False, "unknown")])
def test_raw_console_cannot_bypass_firmware_or_simulator_gate(simulator, revision):
    c = ready(); c.simulator = simulator; c.state["rev"] = revision
    with pytest.raises(ValueError): c.request("imurecover", 1)
    assert not drain(c) and not c.snapshot()["can_recover_imu"]


@pytest.mark.parametrize("line", ["imurecover force", "imurecover 0", "imurecover now now"])
def test_raw_console_rejects_additional_arguments(line):
    c = ready()
    with pytest.raises(ValueError): c.request(line, 1)
    assert not drain(c) and c.phase == "idle"


def test_recovery_is_not_queued_behind_a_drive_or_posture():
    c = ready(); c.update(0, 1, .5); drain(c)
    with pytest.raises(ValueError): c.request("imurecover", 1)
    assert c.phase == "drive" and c.pending is None and not drain(c)
    c = ready(); c.request("landing", .5); drain(c)
    with pytest.raises(ValueError): c.request("imurecover", 1)
    assert c.command == "landing" and c.pending is None and not drain(c)


@pytest.mark.parametrize("pose", ["stow", "stow-paused"])
def test_recovery_keeps_existing_stow_guard(pose):
    c = ready(); c.state["pose"] = pose
    assert not c.can_recover_imu
    with pytest.raises(ValueError): c.request("imurecover", 1)
    assert not drain(c)


def test_recovery_is_exclusive_through_fresh_state_readback():
    c = ready(); begin(c)
    for command in ("stand", "syncstate", "imurecover", "app_probe_start"):
        with pytest.raises(ValueError): c.request(command, 1.1)
    c.feed(IMU_RECOVERY_ACK.encode() + b"\r\n", 1.5)
    assert c.imu_recovery_pending and c.command == "imurecover" and not drain(c)
    c.feed(b"# ", 1.6)
    assert drain(c) == b"syncstate\n"
    assert c.imu_recovery_pending and not c.controls_enabled
    c.sample_input((0,1), 1.7)
    assert not drain(c)
    c.feed(STATE + b"# ", 1.8)
    assert c.imu_recovery_status == "succeeded" and not c.imu_recovery_pending
    assert c.controls_enabled and not c.can_drive


def test_held_gesture_and_crossing_center_cannot_restart_after_recovery():
    c = ready(); begin(c)
    # Neither a release during recovery nor a fresh press before the ACK counts.
    c.sample_input(None, 1.1); c.sample_input((0,1), 1.2)
    complete(c)
    for t, vector in [(2.2,(0,1)), (2.3,(0,0)), (2.4,(0,-1))]:
        c.sample_input(vector,t); c.tick(t)
    assert not drain(c) and c.phase == "idle" and c.imu_recovery_requires_release
    c.sample_input(None, 2.5)
    c.sample_input((0,1), 2.6)
    assert drain(c).startswith(b"drive 1000 0 ")


def test_direct_update_callers_also_need_a_real_release():
    c = ready(); begin(c); complete(c)
    c.update(0,0,2.2); c.update(0,1,2.3)
    assert not drain(c) and c.imu_recovery_requires_release
    c.release(2.4); c.update(0,1,2.5)
    assert drain(c).startswith(b"drive 1000 0 ")


def test_parameter_joystick_does_not_replay_after_recovery():
    c = ready(); c.parameter_walking = True
    c.state["profile"] = "s_native_v6_2_5"
    c.probe_config = (20,344,4000,"all",0,0)
    begin(c)
    c.update(0,1,1.2)
    complete(c, STATE.replace(b"profile=attitudepd_v6", b"profile=s_native_v6_2_5"))
    c.update(0,0,2.2); c.update(0,1,2.3)
    assert not drain(c) and not c.probe_running
    c.sample_input(None,2.4); c.sample_input((0,1),2.5)
    assert drain(c) == b"walkprobe\n"


@pytest.mark.parametrize("response", [b"OK\r\n", b"OK imurecover\r\n", b"",
                           IMU_RECOVERY_ACK.encode()+b"\r\nERROR: IMU read failed\r\n"])
def test_missing_exact_ack_or_error_cannot_report_success(response):
    c = ready(); begin(c)
    c.feed(response + b"# ",2)
    assert drain(c) == b"syncstate\n"
    c.feed(STATE + b"# ",2.1)
    assert c.imu_recovery_status == "failed" and c.imu_recovery_requires_release
    assert not c.can_drive and not drain(c)


def test_stale_state_during_recovery_does_not_satisfy_refresh():
    c = ready(); begin(c)
    c.feed(STATE + IMU_RECOVERY_ACK.encode()+b"\r\n# ",2)
    assert drain(c) == b"syncstate\n"
    c.feed(b"# ",2.1)
    assert c.imu_recovery_status == "failed" and c.fatal


def test_success_does_not_clear_firmware_faults_or_change_pose_torque():
    c = ready(); c.state["safety"] = "fault"; c.pause_reason = "imu"
    begin(c)
    complete(c, STATE.replace(b"safety=ok", b"safety=fault"))
    assert c.imu_recovery_status == "succeeded"
    assert c.state["safety"] == "fault" and c.pause_reason == "imu"
    assert c.state["pose"] == "stand" and c.state["torque"] == "on"


def test_timeout_disconnects_and_late_ack_cannot_rearm_input():
    c = ready(); begin(c)
    c.tick(6.1)
    assert drain(c) == b"\x03" and c.fatal
    assert c.imu_recovery_status == "timeout" and not c.imu_recovery_pending
    c.feed(IMU_RECOVERY_ACK.encode()+b"\r\n# ",6.2)
    c.sample_input((0,1),6.3)
    assert c.imu_recovery_status == "timeout" and not drain(c)


def test_stop_keeps_interrupt_priority_and_cancels_recovery_result():
    c = ready(); c.request("imurecover",1)
    c.stop(1.1)
    assert list(c.outbox) == [("interrupt", b"\x03")]
    assert c.imu_recovery_status == "cancelled" and not c.imu_recovery_pending
    assert c.command is None and c.imu_recovery_requires_release


def test_snapshot_exposes_result_and_input_boundary():
    c = ready(); begin(c); complete(c)
    state = c.snapshot()
    assert state["supports_imu_recovery"] and state["can_recover_imu"]
    assert state["imu_recovery_status"] == "succeeded"
    assert not state["imu_recovery_pending"] and state["imu_recovery_requires_release"]
    assert "새로 조작" in state["imu_recovery_message"]
