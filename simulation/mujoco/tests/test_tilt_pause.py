from simulation.mujoco.runtime.bno055_emulator import FirmwareAttitudeFilter

def reading(roll,pitch=0):
    return dict(roll_tenths=roll,pitch_tenths=pitch)

def test_explicit_retry_retains_real_attitude_and_restores_level_guard():
    f=FirmwareAttitudeFilter()
    assert f.update(reading(200)) is None
    assert f.update(reading(200))=='tilt'
    assert f.update(reading(200))=='tilt'  # no automatic restart
    f.retry_tilt(reading(200))
    for _ in range(5):assert f.update(reading(200)) is None
    assert f.filtered[0]>120  # do not falsify balance input
    assert f.update(reading(321)) is None
    assert f.update(reading(321))=='tilt'
    f.retry_tilt(reading(200))
    assert f.update(reading(0)) is None
    assert f.update(reading(200)) is None
    assert f.update(reading(200))=='tilt'

def test_retry_does_not_bypass_missing_imu():
    import pytest
    f=FirmwareAttitudeFilter()
    with pytest.raises(ValueError):f.retry_tilt(None)
    f.retry_tilt(reading(-200))
    assert f.update(None) is None
    assert f.update(None) is None
    assert f.update(None)=='imu'
