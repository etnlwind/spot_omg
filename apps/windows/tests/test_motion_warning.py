from spot_controller.protocol import Controller
from spot_controller.motion_warning import motion_warning

def test_protective_pause_survives_ok_telemetry_until_new_motion_ack():
    c=Controller();c.connected=True;c.synced=True;c.phase='drive'
    c.feed(b'$SPOTDRIVE stopped reason=tilt\r\n',1)
    assert c.requires_release
    reason=c.snapshot()['pause_reason']
    assert '기울기' in motion_warning(reason)[0]
    c.feed(b'$SPOTSTATE pose=custom torque=on safety=ok\r\n',2)
    assert c.pause_reason==reason
    c.feed(b'$SPOTDRIVE started seq=2 watchdog=800ms\r\n',3)
    assert not c.pause_reason

def test_pose_rejection_is_visible_and_read_commands_do_not_clear_it():
    c=Controller();c.command='landing';c.phase='busy'
    c.feed(b'ERROR: motion tilt safety limit reached; motion cancelled\r\n',1)
    assert '기울기' in motion_warning(c.pause_reason)[0]
    c.command='read 1'
    c.feed(b'OK\r\n',2)
    assert c.pause_reason

def test_imu_reason_is_not_presented_as_tilt():
    assert 'IMU' in motion_warning('ERROR: imu unavailable')[0]
