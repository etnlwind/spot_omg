import pytest
from spot_controller.battery import BatteryWarning, reading
from spot_controller.protocol import Controller
from test_protocol import ready, drain


def test_measured_low_pack_and_droop_latch_until_sustained_recharge():
    b = BatteryWarning()
    b.observe(10900, 0)
    assert b.level == 1
    b.observe(10300, 1)
    assert b.level == 2
    for now, mv in [(2, 10900), (3, 11000), (4, 11300), (5, 11400), (6, 11400)]:
        b.observe(mv, now)
        assert b.level == 2
    b.observe(11400, 10)
    assert b.level == 0
    b.observe(11000, 11)
    assert b.level == 1
    b.observe(10500, 12)
    assert b.level == 2


def test_invalid_duplicate_and_historical_samples_cannot_clear_warning():
    b = BatteryWarning()
    b.observe(10800, 0)
    for mv in [0, -1, 65535]:
        b.observe(mv, 1)
    b.observe(12000, 2, historical=True)
    assert b.level == 1
    b.observe(12000, 3)
    for _ in range(10): b.observe(12000, 3)
    b.observe(12000, 8)
    assert b.level == 1  # Only two distinct observations.
    b.observe(12000, 9)
    assert b.level == 0


@pytest.mark.parametrize('line', ['ID 1 voltage=0mV', '$BATTERY mv=65535', '$BATTERY mv=10500oops', 'ID 2 voltage=10000mV', 'old $BATTERY mv=10000'])
def test_invalid_or_unselected_source_ignored(line):
    assert reading(line) is None


def test_fragmented_telemetry_warns_during_drive_without_motor_command():
    record = b'$BATTERY mv=10300\r\n'
    for split in range(len(record)+1):
        c = ready(); c.update(0, 1, 1); drain(c)
        c.feed(record[:split], 2); c.feed(record[split:], 2)
        assert c.battery.level == 2
        assert c.voltage == 10.3
        assert c.phase == 'drive'
        assert drain(c) == b''
        c.tick(2.1)
        assert drain(c).startswith(b'@D ')  # No read interrupts realtime lane.


def test_idle_poll_only_and_previous_gait_minimum_does_not_replace_live_voltage():
    c = ready()
    c.tick(6)
    assert drain(c) == b'read 1\n'
    c.tick(6.1)
    assert not drain(c)
    c.feed(b'ID 1 voltage=11500mV\r\n# ', 6.2)
    c.feed(b'Gait diagnostics: samples=2828 min_voltage=10300mV lag=121\r\n', 7)
    assert c.voltage == 11.5 and c.voltage_at == 6.2
    assert c.battery.level == 2
    c.connected = False
    assert c.snapshot()['battery_warning'] == {}


def test_old_recovery_samples_do_not_count_after_telemetry_gap():
    b = BatteryWarning(); b.observe(10500, 0)
    b.observe(12000, 1); b.observe(12000, 3)
    b.observe(12000, 30)
    assert b.level == 2
    b.observe(12000, 32); b.observe(12000, 35)
    assert b.level == 0


def test_simulator_neither_polls_nor_warns_for_low_voltage():
    c=ready();c.simulator=True;c.voltage=None
    c.feed(b'$BATTERY mv=10300\r\n',1)
    c.feed(b'ID 1 voltage=10300mV\r\n',2)
    c.tick(6)
    assert c.battery.level==0 and c.voltage is None
    assert b'read 1' not in drain(c)
    c.battery.observe(10300,7)  # Even stale real-robot state cannot surface.
    assert c.snapshot()['battery_warning']=={}
