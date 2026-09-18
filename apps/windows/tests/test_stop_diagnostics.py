"""Replay real V80 diagnostic volume without moving a physical robot."""
from pathlib import Path

import pytest

from spot_controller.protocol import Controller

STATE = b'$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v4-v80 caps=gaitprofiles,attitudepd_v4 profile=attitudepd_v4\r\n'
STOP = b'$SPOTDRIVE stopped reason=ok elapsed=7568ms\r\n'


def drain(controller):
    data = b''.join(payload for _, payload in controller.outbox)
    controller.outbox.clear()
    return data


def released_drive():
    c = Controller()
    c.opened(0)
    assert drain(c) == b'syncstate\n'
    c.feed(STATE + b'# ', .1)
    assert drain(c) == b'read 1\n'
    c.feed(b'ID 1 voltage=12000mV\r\n# ', .2)
    c.release(.2)
    c.update(0, 1, 1)
    assert drain(c).startswith(b'drive ')
    c.release(1.1)
    assert drain(c).startswith(b'@S ')
    return c


@pytest.mark.parametrize('chunk_size,interval', [(20, .03), (180, .30)])
def test_recorded_v80_diagnostics_keep_connection_until_prompt(chunk_size, interval):
    c = released_drive()
    recorded = (Path(__file__).parent/'fixtures/v80-stop-diagnostics.txt').read_bytes()
    now = 1.2
    for start in range(0, len(recorded), chunk_size):
        now += interval
        c.feed(recorded[start:start+chunk_size], now)
        c.tick(now)
        assert not c.fatal, (now, c.error)
        assert not drain(c)  # Neither heartbeat, Ctrl+C, nor read may cut into diagnostics.
    assert now > 6.5 and c.phase == 'draining' and not c.can_drive
    c.update(0, 1, now)
    assert not drain(c)
    # The supplied log was cut mid-line. This synthetic suffix models the
    # remaining line ending and prompt, not an observed successful reconnection.
    c.feed(b'\r\n# ', now+.01)
    assert c.phase == 'idle' and not c.fatal
    c.update(0, 1, now+.02)
    assert not drain(c)  # A held input must not restart motion after completion.
    c.tick(now+.2)
    assert drain(c) == b'syncstate\n'
    c.feed(STATE+b'# ', now+.3)
    assert drain(c) == b'read 1\n'
    c.feed(b'ID 1 voltage=12000mV\r\n# ', now+.4)
    assert c.connected and c.can_drive
    c.release(now+.5)
    c.update(0, -1, now+.6)
    assert drain(c).startswith(b'drive -1000 0 ')


def test_partial_diagnostic_bytes_count_as_progress():
    c = released_drive()
    c.feed(STOP, 1.2)
    for now in (5., 9., 13.):
        c.feed(b'part of one unfinished diagnostic line ', now)
        c.tick(now)
        assert not c.fatal and not drain(c)
    c.feed(b'\r\n# ', 13.1)
    assert c.phase == 'idle' and not c.fatal


def test_telemetry_cannot_extend_unconfirmed_stop_deadline():
    c = released_drive()
    for now in (2., 4., 6., 6.2):
        c.feed(b'$BATTERY mv=12000\r\n', now)
        c.tick(now)
    assert c.fatal and drain(c) == b'\x03'


def test_silent_diagnostic_stream_probes_once_then_times_out():
    c = released_drive()
    c.feed(STOP, 1.2)
    c.feed(b'Gait diagnostics: ', 4.)
    c.tick(8.9)
    assert not c.fatal and not drain(c)
    c.tick(9.1)
    assert not c.fatal and drain(c) == b'\nsyncstate\n'
    assert c.phase == 'resync' and not c.can_drive
    c.tick(14.2)
    assert c.fatal and drain(c) == b'\x03'


def test_lost_tail_recovers_only_after_fresh_state_and_prompt():
    c = released_drive()
    c.feed(STOP + b'ID11 partial diagnostic', 1.2)
    c.pending = 'stand'
    c.tick(6.3)
    assert drain(c) == b'\nsyncstate\n'
    c.feed(b'\r\n# ', 6.4)  # Late or empty prompt is insufficient.
    assert c.phase == 'resync' and not c.can_drive
    c.feed(STATE, 6.5)
    assert not c.can_drive
    c.feed(b'# ', 6.6)
    assert c.phase == 'idle' and c.synced and not c.fatal
    assert c.pending is None and not drain(c)
    c.update(0, 1, 6.7)  # Still-held input must not restart after recovery.
    assert not drain(c)
    c.release(6.8)
    c.update(0, 1, 6.9)
    assert drain(c).startswith(b'drive ')


def test_error_without_stop_ack_is_not_recoverable():
    c = released_drive()
    c.feed(b'ERROR: incomplete motion\r\n', 1.2)
    c.tick(6.3)
    assert c.fatal and drain(c) == b'\x03'


@pytest.mark.parametrize('reply', [b'# ', b'$SPOTSTATE pose=stand\r\n# ', STATE,
                                 b'ERROR: failed\r\n' + STATE + b'# '])
def test_incomplete_or_failed_resync_does_not_enable_motion(reply):
    c = released_drive()
    c.feed(STOP, 1.2)
    c.tick(6.3)
    drain(c)
    c.feed(reply, 6.4)
    c.update(0, 1, 6.5)
    assert not c.can_drive and not drain(c)
    c.tick(11.4)
    assert c.fatal and drain(c) == b'\x03'


def test_continuous_output_without_prompt_has_absolute_limit():
    c = released_drive()
    c.feed(STOP, 1.2)
    for second in range(2, 32):
        # Duplicate stop notifications must not restart the absolute timer.
        c.feed(STOP if second % 3 == 0 else b'$BATTERY mv=12000\r\n', float(second))
        c.tick(float(second))
        assert not c.fatal and not drain(c)
    c.feed(b'$BATTERY mv=12000\r\n', 31.3)
    c.tick(31.3)
    assert c.fatal and drain(c) == b'\x03'
