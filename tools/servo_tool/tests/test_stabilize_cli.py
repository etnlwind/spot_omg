"""Realtime attitude-PD control must bypass the occupied firmware text console."""
from unittest.mock import Mock, patch
import ctypes
import json
from pathlib import Path
import subprocess
import pytest
from servo.cli import CONSOLE_ONLY_COMMANDS, console_line_for, main, parse_args
from servo.console import ConsoleError, ConsoleResponse, Stm32Console

@pytest.mark.parametrize('words,line',[
    (['stabilize'],'@B 2'),(['stabilize','status'],'@B 2'),
    (['stabilize','on'],'@B 1'),(['stabilize','off'],'@B 0'),
])
def test_stabilizer_uses_realtime_command(words,line):
    assert 'stabilize' in CONSOLE_ONLY_COMMANDS
    assert console_line_for(parse_args(words))==line

@pytest.mark.parametrize('mode',['full','normal','1','invalid'])
def test_stabilizer_rejects_unsupported_modes(mode):
    with pytest.raises(SystemExit):parse_args(['stabilize',mode])

@pytest.mark.parametrize('transport_args,kind,endpoint',[
    (['--host','127.0.0.1'],'tcp','127.0.0.1'),
    (['--via','ble'],'ble','SpotOMG-Bridge'),
    (['--via','stm32','--stm32-port','/dev/fake-stm32'],'stm32','/dev/fake-stm32'),
])
def test_stabilizer_skips_blocking_sync_and_time_query(transport_args,kind,endpoint):
    console=Mock()
    console.send.return_value=ConsoleResponse('@B 1',('$STABILIZE enabled=1 status=active',),(),'info')
    context=Mock();context.__enter__=Mock(return_value=console);context.__exit__=Mock(return_value=False)
    with patch('servo.cli.open_console',return_value=context) as opened, patch('servo.cli._synchronize_console_clock') as clock:
        assert main(['--no-app-control',*transport_args,'stabilize','on'])==0
    assert opened.call_args.args[1:]==(kind,endpoint)
    console.sync.assert_not_called();clock.assert_not_called()
    assert [call.args[0] for call in console.send.call_args_list]==['@B 1']

class ChunkTransport:
    def __init__(self,response):
        self.is_open=True;self.response=list(response);self.chunks=[b'stale\r\n# '];self.writes=[]
    @property
    def in_waiting(self):return len(self.chunks[0]) if self.chunks else 0
    def reset_input_buffer(self):self.chunks=[]
    def write(self,data):self.writes.append(data);self.chunks.extend(self.response);return len(data)
    def flush(self):pass
    def read(self,size):
        if not self.chunks:return b''
        chunk=self.chunks.pop(0);result=chunk[:size]
        if len(chunk)>size:self.chunks.insert(0,chunk[size:])
        return result


def test_realtime_ack_waits_past_unrelated_prompt_and_fragmented_record():
    transport=ChunkTransport([b'# ',b'$STABI',b'LIZE enabled=1 status=active\r\n',b'# '])
    response=Stm32Console('fake',transport=transport).send('@B 1',timeout=.1)
    assert response.ok
    assert response.lines==('$STABILIZE enabled=1 status=active',)
    assert transport.writes==[b'@B 1\n']


def test_realtime_ack_requires_its_own_status_record():
    transport=ChunkTransport([b'OK\r\n# '])
    with pytest.raises(ConsoleError,match='timed out'):
        Stm32Console('fake',transport=transport).send('@B 2',timeout=.005)
    assert transport.writes==[b'@B 2\n']


def test_realtime_firmware_error_is_not_hidden_while_waiting_for_ack():
    transport=ChunkTransport([b'ERROR: unsupported realtime command\r\n# '])
    response=Stm32Console('fake',transport=transport).send('@B 0',timeout=.1)
    assert response.status=='error'


def test_attitude_profile_preserves_centerpivot_nominal_and_default(tmp_path):
    root=Path(__file__).resolve().parents[3]
    manifest=json.loads((root/'config/locomotion_profiles.json').read_text())
    profiles=manifest['profiles'];nominal=profiles['centerpivot'];pd=profiles['attitudepd']
    assert manifest['default']=='cruise'
    for key in ('family','params','turn_reverse_params','turn_input_limit'):
        assert pd[key]==nominal[key]
    assert pd['experimental']
    source=tmp_path/'nominal.c';library=tmp_path/'nominal.dylib'
    source.write_text('''#include "locomotion.h"
int id(const char *s){return locomotion_profile_id(s);}
int target(int profile,float phase,float scale,float linear,float yaw,float *q){
 GaitPolicyLegTarget out[4];if(!locomotion_targets(profile,phase,scale,linear,yaw,out))return 0;
 for(int i=0;i<4;i++){q[3*i]=out[i].j1_deg;q[3*i+1]=out[i].j2_deg;q[3*i+2]=out[i].j3_deg;}return 1;
}
float period(int profile,float linear,float yaw){return locomotion_period(profile,linear,yaw);}
''')
    subprocess.run(['clang','-shared','-fPIC','-O2','-I'+str(root/'firmware/stm32-learning/Inc'),str(source),'-o',str(library)],check=True)
    lib=ctypes.CDLL(str(library));f=ctypes.c_float
    lib.id.argtypes=(ctypes.c_char_p,);lib.id.restype=ctypes.c_int
    lib.target.argtypes=(ctypes.c_int,f,f,f,f,ctypes.POINTER(f));lib.target.restype=ctypes.c_int
    lib.period.argtypes=(ctypes.c_int,f,f);lib.period.restype=f
    assert lib.id(b'centerpivot')==14 and lib.id(b'attitudepd')==15
    for linear,yaw in ((0,0),(1,0),(-1,0),(0,.5),(0,-.5),(.5,.3),(-.5,-.3)):
        assert lib.period(14,linear,yaw)==lib.period(15,linear,yaw)
        for scale in (0.,.2,1.):
            for n in range(101):
                before=(f*12)();after=(f*12)()
                assert lib.target(14,n/100,scale,linear,yaw,before)
                assert lib.target(15,n/100,scale,linear,yaw,after)
                assert list(before)==list(after)
