import pytest
from spot_controller.protocol import Controller, ConsoleStream, drive_vector

STATE = b'$SPOTSTATE pose=stand torque=on safety=ok rev=test caps=stow,gaitprofiles,balancecontrol,headinghold,trot5 profile=legacy\r\n'


def test_native_profiles_require_simulator_and_explicit_capability():
    c = Controller(simulator=True)
    c.caps = {'gaitprofiles', 's_native_v5'}
    c.validate('gaitprofile s_native_v5')
    with pytest.raises(ValueError):
        c.validate('gaitprofile s_native_v4')
    c.simulator = False
    with pytest.raises(ValueError):
        c.validate('gaitprofile s_native_v5')


def test_v6_1_real_robot_requires_firmware_capability():
    c=Controller(simulator=False);c.caps={'gaitprofiles'}
    with pytest.raises(ValueError):c.validate('gaitprofile s_native_v6_1')
    c.caps.add('s_native_v6_1');c.validate('gaitprofile s_native_v6_1')
    c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_1')+b'# ',.1);drain(c)
    c.feed(b'ID 1 voltage=11100mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_1\n'


def test_select_newest_supported_native_once_after_initial_readback():
    c = Controller(simulator=True)
    c.opened(0)
    drain(c)
    state = STATE.replace(b'trot5', b'trot5,s_native_v2,s_native_v4')
    c.feed(state + b'# ', .1)
    assert drain(c) == b'read 1\n'
    c.feed(b'ID 1 voltage=11100mV\r\n# ', .2)
    assert drain(c) == b'gaitprofile s_native_v4\n'
    assert not c.default_profile_pending


@pytest.mark.parametrize('simulator,expected',[(True,'s_native_v6_2'),(False,'s_native_v6_1')])
def test_v62_default_keeps_v61_available_for_hardware(simulator,expected):
    c=Controller(simulator=simulator);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_1,s_native_v6_2')+b'# ',.1)
    drain(c);c.feed(b'ID 1 voltage=11100mV\r\n# ',.2)
    assert drain(c)==f'gaitprofile {expected}\n'.encode()
    if not simulator:
        with pytest.raises(ValueError):c.validate('gaitprofile s_native_v6_2')


@pytest.mark.parametrize('simulator,expected',[(True,'s_native_v6_2_1'),(False,'s_native_v6_2_1')])
def test_v621_is_first_supported_simulator_choice(simulator,expected):
    from spot_controller.protocol import NATIVE_PROFILES
    start=NATIVE_PROFILES.index('s_native_v6_2_2')
    assert NATIVE_PROFILES[start:start+4]==('s_native_v6_2_2','s_native_v6_2_1','s_native_v6_2','s_native_v6_1')
    c=Controller(simulator=simulator);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_1,s_native_v6_2,s_native_v6_2_1')+b'# ',.1)
    drain(c);c.feed(b'ID 1 voltage=11100mV\r\n# ',.2)
    assert drain(c)==f'gaitprofile {expected}\n'.encode()
    c.validate('gaitprofile s_native_v6_2_1')
    c.caps.remove('s_native_v6_2_1')
    with pytest.raises(ValueError):c.validate('gaitprofile s_native_v6_2_1')

def drain(c):
    packets = list(c.outbox)
    c.outbox.clear()
    return b''.join(d for _, d in packets)

def ready():
    c = Controller()
    c.opened(0)
    assert drain(c) == b'syncstate\n'
    c.feed(STATE + b'# ', .1)
    assert drain(c) == b'read 1\n'
    c.feed(b'ID 1 voltage=11100mV\r\n# ', .2)
    c.release(.2)
    assert c.phase == 'idle'
    return c

def test_parser_all_packet_boundaries_and_unicode():
    data = '# $SPOTSTATE pose=stand\r\n# $SPOTDRIVE stopped reason=requested\r\n한글\r\n# '.encode()
    expected = [('prompt',''), ('line','$SPOTSTATE pose=stand'), ('prompt',''), ('line','$SPOTDRIVE stopped reason=requested'), ('line','한글'), ('prompt','')]
    for split in range(len(data)+1):
        s = ConsoleStream()
        assert s.feed(data[:split]) + s.feed(data[split:]) == expected
    s = ConsoleStream()
    assert sum((s.feed(bytes([b])) for b in data), []) == expected

def test_parser_bounds_unterminated_input():
    with pytest.raises(ValueError): ConsoleStream().feed(b'x'*8193)

@pytest.mark.parametrize('x,y,expected', [(0,1,(1000,0)), (1,0,(0,1000)), (-1,0,(0,-1000)), (0,-1,(-1000,0)), (.01,.8,(835,0)), (0,0,(0,0)), (.1,0,(0,0))])
def test_ios_axis_mapping(x,y,expected):
    assert drive_vector(x,y) == expected

def test_invalid_vector_and_clamping():
    assert drive_vector(float('nan'), 0) is None
    assert max(abs(v) for v in drive_vector(2,2)) <= 1000

def test_drive_heartbeat_neutral_reverse_no_restarts():
    c=ready(); c.update(0,1,1)
    assert drain(c).startswith(b'drive 1000 0 ')
    c.tick(1.2); assert drain(c).startswith(b'@D ')
    c.update(0,0,1.3); c.tick(1.4)
    assert drain(c).endswith(b' 0 0\n')
    c.update(0,-1,1.5); c.tick(1.7)
    assert drain(c).endswith(b' -1000 0\n')
    c.request('syncstate',1.8)
    assert not drain(c) and c.phase=='drive'

def test_release_waits_for_stop_then_prompt_and_new_gesture():
    c=ready();c.update(0,1,1);drain(c);c.release(1.1)
    assert drain(c).startswith(b'@S ')
    c.update(0,1,1.2);c.tick(1.3);c.feed(b'# ',1.4)
    assert c.phase=='stopping' and not drain(c)
    c.feed(b'$SPOTDRIVE stopped reason=requested\r\n',1.5)
    assert c.phase=='draining'
    c.feed(b'# ',1.6);assert c.phase=='idle'
    c.update(0,1,1.7);assert not drain(c)
    c.release(1.8);c.update(0,-1,1.9)
    assert drain(c).startswith(b'drive -1000 0 ')


def test_normal_stop_is_idempotent_while_robot_returns_to_s():
    c = ready(); c.update(0, 1, 1); drain(c)
    c.stop(1.1)
    assert drain(c).startswith(b'@S ')
    c.stop(2); c.update(0, 1, 2.1); c.tick(3)
    assert c.phase == 'stopping' and not drain(c)
    c.feed(b'$SPOTDRIVE stopped reason=requested\r\n# ', 3.3)
    assert c.phase == 'idle'
    c.update(0, 1, 3.4)
    assert not drain(c)


def test_emergency_can_still_interrupt_return_to_s():
    c = ready(); c.update(0, 1, 1); drain(c)
    c.stop(1.1); drain(c); c.interrupt(1.2)
    assert drain(c) == b'\x03'

def test_watchdog_and_error_require_release_no_replay():
    c=ready();c.update(0,1,1);drain(c)
    c.feed(b'$SPOTDRIVE stopped reason=watchdog\r\n# ',1.1)
    c.update(0,1,1.2)
    assert not drain(c) and c.vector is None

def test_stop_timeout_clears_pending_motion_and_disconnects():
    c=ready();c.update(0,1,1);drain(c);c.request('stand',1.1);drain(c);c.tick(6.2)
    assert drain(c)==b'\x03' and c.fatal and c.pending is None

def test_command_after_drive_requires_stop_ack():
    c=ready();c.update(0,1,1);drain(c);c.request('landing',1.1)
    assert drain(c).startswith(b'@S')
    c.feed(b'$SPOTDRIVE stopped reason=requested\r\n# ',1.2)
    assert drain(c)==b'landing\n'

def test_relax_requires_fresh_landing_ack_and_readback_across_chunks():
    c=ready();c.state['pose']='landing';c.request('relax',1)
    assert drain(c)==b'landing\n'
    c.feed(b'OK\r\n# ',2); assert drain(c)==b'syncstate\n'
    c.feed(STATE.replace(b'pose=stand',b'pose=landing'),3);assert not drain(c)
    c.feed(b'# ',3.1);assert drain(c)==b'relax\n'

@pytest.mark.parametrize('reply', [b'ERROR: failed\r\n# ',b'# ',b'STOPPED: interrupt\r\n# '])
def test_relax_cancelled_on_failure_or_missing_ack(reply):
    c=ready();c.request('relax',1);drain(c);c.feed(reply,2)
    assert b'relax' not in drain(c) and not c.relax_pending

def test_relax_readback_mismatch_and_emergency_cancel():
    c=ready();c.request('relax',1);drain(c);c.feed(b'OK\r\n# ',2);drain(c);c.feed(STATE+b'# ',3)
    assert b'relax' not in drain(c) and not c.relax_pending
    c=ready();c.request('relax',1);drain(c);c.interrupt(2);c.feed(b'OK landing\r\n# ',3)
    assert b'relax' not in drain(c)

def test_stow_and_capabilities_apply_to_raw_commands():
    c=ready();c.state['pose']='stow'
    for cmd in ('stand','relax','recover','gaitprofile legacy','move 2 0'):
        with pytest.raises(ValueError):c.request(cmd,1)
    c.request('landing',1);assert drain(c)==b'landing\n'
    c=ready()
    for cmd in ('drive 1000 0 1','@D 1 1000 0','gaitprofile cushion_wbc','gaitprofile attitudepd','stand\nrelax'):
        with pytest.raises(ValueError):c.request(cmd,1)

def test_initial_sync_missing_and_sequence_wrap():
    c=Controller();c.opened(0);drain(c)
    with pytest.raises(ValueError):c.request('stand',.1)
    c.feed(b'OK\r\n# ',.2);assert c.fatal and not c.synced
    c=ready();c.sequence=0xffffffff;c.update(0,1,1)
    assert drain(c)==b'drive 1000 0 0\n'

def test_disconnect_during_read_does_not_send_motion():
    c=ready();c.request('syncstate',1);drain(c);c.disconnect(1.1)
    assert not drain(c)
    c.feed(STATE+b'# ',1.2);assert c.fatal and not drain(c)


def test_v622_is_latest_supported_hardware_model():
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=False);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_1,s_native_v6_2_2')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_2\n'
    c.validate('gaitprofile s_native_v6_2_2')
    assert NATIVE_PROFILES.index('s_native_v6_2_2') < NATIVE_PROFILES.index('s_native_v6_2_1')
    assert 's_native_v6_2_2' not in SIMULATOR_NATIVE_PROFILES


def test_v623_is_latest_supported_hardware_model():
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=False);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_1,s_native_v6_2_2,s_native_v6_2_3')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_3\n'
    c.validate('gaitprofile s_native_v6_2_3')
    assert NATIVE_PROFILES.index('s_native_v6_2_3') < NATIVE_PROFILES.index('s_native_v6_2_2')
    assert 's_native_v6_2_3' not in SIMULATOR_NATIVE_PROFILES


def test_v624_is_latest_supported_hardware_model():
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=False);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_1,s_native_v6_2_2,s_native_v6_2_3,s_native_v6_2_4')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_4\n'
    c.validate('gaitprofile s_native_v6_2_4')
    assert NATIVE_PROFILES.index('s_native_v6_2_4') < NATIVE_PROFILES.index('s_native_v6_2_3')
    assert 's_native_v6_2_4' not in SIMULATOR_NATIVE_PROFILES


def test_v625_is_latest_supported_hardware_model():
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=False);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_4,s_native_v6_2_5')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_5\n'
    assert NATIVE_PROFILES.index('s_native_v6_2_5') < NATIVE_PROFILES.index('s_native_v6_2_4')
    assert 's_native_v6_2_5' not in SIMULATOR_NATIVE_PROFILES


@pytest.mark.parametrize('simulator',[False,True])
def test_v626_is_first_only_when_firmware_advertises_support(simulator):
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=simulator);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_5,s_native_v6_2_6')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_6\n'
    assert NATIVE_PROFILES.index('s_native_v6_2_6') < NATIVE_PROFILES.index('s_native_v6_2_5')
    assert 's_native_v6_2_6' not in SIMULATOR_NATIVE_PROFILES
    c.validate('gaitprofile s_native_v6_2_6')
    c.caps.remove('s_native_v6_2_6')
    with pytest.raises(ValueError):c.validate('gaitprofile s_native_v6_2_6')


@pytest.mark.parametrize("simulator",[False, True])
def test_v627_is_first_only_when_firmware_advertises_support(simulator):
    from spot_controller.protocol import NATIVE_PROFILES, SIMULATOR_NATIVE_PROFILES
    c=Controller(simulator=simulator);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',b'trot5,s_native_v6_2_5,s_native_v6_2_7')+b'# ',.1)
    assert drain(c)==b'read 1\n'
    c.feed(b'ID 1 voltage=11400mV\r\n# ',.2)
    assert drain(c)==b'gaitprofile s_native_v6_2_7\n'
    assert NATIVE_PROFILES[0]=='s_native_v6_2_7'
    assert 's_native_v6_2_7' not in SIMULATOR_NATIVE_PROFILES
    c.validate('gaitprofile s_native_v6_2_7')
    c.caps.remove('s_native_v6_2_7')
    with pytest.raises(ValueError):c.validate('gaitprofile s_native_v6_2_7')


@pytest.mark.parametrize('supported,expected', [
    ('attitudepd_v2,attitudepd_v3,attitudepd_v4','attitudepd_v4'),
    ('attitudepd_v2,attitudepd_v3','attitudepd_v3'),
    ('attitudepd_v2','attitudepd_v2'),
])
def test_attitudepd_default_respects_firmware_capability(supported,expected):
    from spot_controller.protocol import PROFILES
    assert PROFILES[:3]==('attitudepd_v4','attitudepd_v3','attitudepd_v2')
    c=Controller(simulator=False);c.opened(0);drain(c)
    c.feed(STATE.replace(b'trot5',supported.encode())+b'# ',.1)
    drain(c);c.feed(b'ID 1 voltage=11100mV\r\n# ',.2)
    assert drain(c)==f'gaitprofile {expected}\n'.encode()
    c.validate('gaitprofile '+expected)
    if expected!='attitudepd_v4':
        with pytest.raises(ValueError):c.validate('gaitprofile attitudepd_v4')
    if expected=='attitudepd_v2':
        with pytest.raises(ValueError):c.validate('gaitprofile attitudepd_v3')
