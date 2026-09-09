import math
import pytest
from bno055_emulator import BNO055Config, BNO055Emulator, FirmwareAttitudeFilter


def sensor(**kw):
    return BNO055Emulator(BNO055Config(noise_std_deg=0, fusion_tau_s=0, startup_s=0, **kw))


def test_register_mapping_quantization_and_c_signed_truncation():
    s=sensor()
    s.advance(0, -1.0625, 2.0625, 359.9375)
    r=s.read(.021)
    assert r['roll_tenths']==-10
    assert r['pitch_tenths']==20
    assert r['yaw_tenths']==3599
    assert r['euler_register_hex']=='7f162100efff'


def test_no_future_sample_and_100hz_hold():
    s=sensor()
    s.advance(0,0,0)
    assert s.read(.020) is None
    assert s.read(.021)['roll_tenths']==0
    s.advance(.01,10,0)
    assert s.read(.030)['roll_tenths']==0
    assert s.read(.031)['roll_tenths']==100
    s.advance(.015,20,0)
    assert s.samples==2


def test_stale_and_transport_failure():
    s=sensor();s.advance(0,0,0)
    assert s.read(.05)
    s.frozen=True;s.advance(.1,5,5)
    assert s.read(.101) is None
    s.online=False
    assert s.read(.05) is None


def test_firmware_filter_signed_rate_and_two_frame_tilt():
    f=FirmwareAttitudeFilter()
    def r(x): return dict(roll_tenths=x,pitch_tenths=-x)
    f.update(r(40)); f.update(r(20))
    assert f.filtered==[12,-12]
    assert f.rate==[-250,250]
    assert f.update(r(121)) is None
    assert f.update(r(121))=='tilt'
    assert f.update(None) is None
    assert f.update(None) is None
    assert f.update(None)=='imu'


def test_noise_repeatable_and_yaw_wrap_short_path():
    c=BNO055Config(startup_s=0)
    a,b=BNO055Emulator(c),BNO055Emulator(c)
    for i in range(20):
        for s in (a,b): s.advance(i*.01,0,0,359 if i<10 else 1)
    assert a.read(.21)==b.read(.21)
    assert a.read(.21)['yaw_tenths']<50 or a.read(.21)['yaw_tenths']>3550


@pytest.mark.parametrize('kw',[{'sample_hz':0},{'fusion_tau_s':-1},{'fusion_delay_s':math.nan}])
def test_invalid_config(kw):
    with pytest.raises(ValueError): BNO055Config(**kw)
