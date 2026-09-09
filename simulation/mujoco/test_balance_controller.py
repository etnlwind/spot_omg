import numpy as np
from balance_controller import BalanceController
from bno055_emulator import FirmwareAttitudeFilter
from servo import SharedGaitPolicy


def test_correction_is_bounded_slewed_and_does_not_accumulate_in_nominal():
    b=BalanceController(SharedGaitPolicy());f=FirmwareAttitudeFilter()
    f.filtered=[100,-50];nominal=np.array([0.,45,90]*4)
    previous=nominal.copy()
    for _ in range(100):
        target=b.apply(nominal,f,True,True)
        assert np.max(abs(target-previous))<=.30001
        assert np.max(abs(target-nominal))<=6.00001
        previous=target
    np.testing.assert_array_equal(nominal,[0,45,90]*4)
    assert max(abs(b.correction))>0
    assert b.integral[0]>0 and b.integral[1]<0


def test_sensor_loss_or_pose_gate_fades_correction_and_clears_integral():
    b=BalanceController(SharedGaitPolicy());f=FirmwareAttitudeFilter();f.filtered=[50,0]
    nominal=np.array([0.,45,90]*4)
    for _ in range(50):b.apply(nominal,f,True,True)
    assert b.applied
    for _ in range(30):out=b.apply(nominal,f,False,True)
    assert not b.applied
    np.testing.assert_allclose(out,nominal)
    np.testing.assert_array_equal(b.integral,[0,0])
    out=b.apply(nominal,f,True,False)
    np.testing.assert_allclose(out,nominal)
