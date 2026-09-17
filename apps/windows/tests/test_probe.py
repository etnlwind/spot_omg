import pytest
from spot_controller.protocol import Controller

def ready():
 c=Controller();c.connected=c.synced=True;c.phase='idle';c.default_profile_pending=False
 c.state=dict(rev='s-native-v6-2-7-v77-t1-param-j1',pose='stand',safety='ok',profile='s_native_v6_2_5')
 return c

def drain(c):
 result=b''.join(d for _,d in c.outbox);c.outbox.clear();return result

def apply(c):
 c.request('probeconfig set 28 344 4000 all',0);assert drain(c)==b'probeconfig set 28 344 4000 all\n'
 c.feed(b'$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all storage=ram\r\n# ',.1)
 assert c.probe_config is None and drain(c)==b'probeconfig show\n'
 c.feed(b'$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all storage=ram\r\n# ',.2)
 assert c.probe_config==(28,344,4000,'all')

def test_readback_start_heartbeat_joystick_and_stop():
 c=ready();apply(c);c.request('app_probe_start',1)
 assert drain(c)==b'walkprobe\n'
 c.update(1,1,1.1);assert c.vector==(344,0)
 c.tick(1.3);assert b'344 0' in drain(c)
 c.tick(8.1);assert b'@S ' in drain(c) and not c.probe_running
 c.feed(b'$SPOTDRIVE stopped reason=ok elapsed=6600ms\r\n# ',8.2)
 assert c.phase=='idle'

def test_mismatch_cancels_start():
 c=ready();c.request('probeconfig set 28 344 4000 all',0);drain(c)
 c.feed(b'# ',.1);drain(c)
 c.feed(b'$PROBECONFIG lift_mm=20 linear=344 duration_ms=4000 legs=all\r\n# ',.2)
 assert c.probe_config is None
 with pytest.raises(ValueError):c.request('app_probe_start',1)

def test_safety_stow_profile_and_unsupported_block_start():
 for key,value in [('pose','landing'),('safety','tilt'),('profile','s_native_v6_2_7'),('rev','v77')]:
  c=ready();apply(c);c.state[key]=value
  with pytest.raises(ValueError):c.start_probe(1)
  assert not c.probe_running

def test_explicit_stop_error_disconnect_and_bad_values():
 c=ready();apply(c);c.start_probe(1);drain(c);c.stop(1.1)
 assert b'@S' in drain(c) and not c.probe_running
 c=ready();apply(c);c.start_probe(1);drain(c)
 c.feed(b'ERROR: servo bus error\r\n# ',1.1)
 assert not c.probe_running and c.vector is None
 c=ready();apply(c);c.start_probe(1);drain(c);c.disconnect(1.1)
 assert drain(c)==b'\x03' and not c.probe_running
 for values in [('41','344','4000','all'),('28','0','4000','all'),('28','344','30001','all'),('28','344','4000','fl')]:
  with pytest.raises(ValueError):Controller.parse_probe(values)

@pytest.mark.parametrize('fr',[0,1])
def test_width_fr_option_readback(fr):
 c=ready();c.state['rev']='s-native-v6-2-7-v77-t1-width'
 c.request(f'probeconfig set 28 344 4000 all -20 {fr}',0);drain(c)
 c.feed(b'# ',.1);drain(c)
 c.feed(f'$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all width_mm=-20 fr_extra={fr}\r\n# '.encode(),.2)
 assert c.probe_config==(28,344,4000,'all',-20,fr)
 with pytest.raises(ValueError):c.parse_probe(['28','344','4000','all','0','2'])

def test_parameter_joystick_uses_probe_and_release_stops():
 c=ready();apply(c);c.request('app_parameter_mode 1',.3);c.release(.4);drain(c)
 c.update(0,1,1);assert drain(c)==b'walkprobe\n'
 assert c.vector==(344,0)
 c.update(0,1,1.1);assert drain(c)==b''
 c.release(1.2);assert b'@S ' in drain(c)


def test_default_joystick_stays_normal_and_mode_locked_in_motion():
 c=ready();apply(c);c.release(.4);c.update(0,1,1)
 assert drain(c).startswith(b'drive ')
 with pytest.raises(ValueError):c.request('app_parameter_mode 1',1.1)


def test_parameter_mode_never_falls_back_when_not_configured():
 c=ready();c.request('app_parameter_mode 1',.3);c.update(0,1,1)
 assert drain(c)==b''
