import pytest
from servo.gait_alignment import align


def capture(origin=0, imu_offset=0, valid=1):
    tick = lambda t: (origin+t) & 0xffffffff
    joint = '\n'.join(['$JT,M,1,1,1,0,0,23,3400,50',
        *[f'$JT,J,{j},{j+1},2048,1' for j in range(12)],
        f'$JT,C,0,{tick(0)},{tick(1)},'+','.join(['2100']*12),
        f'$JT,S,{tick(2)},{tick(4)},0,0,0,2090,0,0,0,11800,30,0', '$JT,END'])
    imu = '\n'.join(['$IT,M,1,2,0,0',
        f'$IT,S,0,{tick(-10+imu_offset)},0,0,0,1000,1,0',
        f'$IT,S,1,{tick(10+imu_offset)},60,0,0,1000,{valid},0', '$IT,END'])
    return joint, imu


def test_clock_wrap_and_front_j1_conversion():
    report, rows, imu = align(*capture(0xfffffff8))
    assert [r['time_ms'] for r in imu] == [-10, 10]
    assert rows[0]['target_deg'] < 0
    assert rows[0]['error_deg'] > 0
    assert rows[0]['imu_time_ms'] == 10
    assert report['absolute_tilt_first_observation']['5']['time_ms'] == 10


def test_disjoint_capture_rejected():
    with pytest.raises(ValueError, match='overlapping'):
        align(*capture(imu_offset=10000))


def test_invalid_imu_not_used():
    joint, imu = capture()
    imu = imu.replace('$IT,M,1,2,', '$IT,M,1,3,').replace('$IT,END', '$IT,S,2,30,0,0,0,1000,1,0\n$IT,END')
    imu = imu.replace('60,0,0,1000,1,0', '60,0,0,1000,0,0')
    report, rows, _ = align(joint, imu)
    assert rows[0]['imu_time_ms'] is None
    assert report['absolute_tilt_first_observation']['5'] is None
