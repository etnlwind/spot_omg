import asyncio
import time

import pytest
from PySide6.QtWidgets import QApplication
from spot_controller import connection
from spot_controller.control_channel import ControlStream, request


def frame(seq, payload=b'', kind=b'DATA'):
    return b'\x1e'+str(seq).encode()+b' '+kind+b' '+payload+b'\x1f'


def test_control_frames_survive_every_byte_boundary_and_resync():
    parser = ControlStream()
    wire = b'ignored # ERROR: console\n'+frame(10,b'OK\r\n')+frame(10,kind=b'DONE')
    result = []
    for byte in wire:
        result.extend(parser.feed(bytes([byte])))
    assert result == [(10,'DATA',b'OK\r\n'),(10,'DONE',b'')]
    assert parser.feed(b'\x1ebroken'+frame(11,b'next')) == [(11,'DATA',b'next')]


@pytest.mark.parametrize('data', [b'',b'\n',b'stand\nrelax',b'x'*96,b'\x03'])
def test_control_request_rejects_truncation_or_embedded_commands(data):
    with pytest.raises(ValueError):request(1,data)


def test_stop_ready_without_waiting_for_logs_and_old_reply_cannot_complete_new_command(monkeypatch):
    app = QApplication.instance() or QApplication([])
    owner = connection.Connection()
    snapshots, sent, logs = [], [], []
    owner.changed.connect(snapshots.append)
    owner.log.connect(logs.append)
    state = b'$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v4-v81 caps=controlv1,gaitprofiles,attitudepd_v4 profile=attitudepd_v4\r\n'
    radio = None

    class Radio:
        control_characteristic = True
        def __init__(self,*args):
            nonlocal radio
            radio=self
            self.console=asyncio.Queue();self.control=asyncio.Queue()
            self.drive_seq=None;self.next_seq=None
        async def open(self):pass
        async def close(self):pass
        async def read(self):return await self.console.get()
        async def read_control(self):return await self.control.get()
        async def write(self,data):
            assert data == b'syncstate\n'
            self.console.put_nowait(state+b'# ')
        async def write_control(self,data):
            sent.append(data)
            if data.startswith(b'@C '):
                _,seq,command=data.rstrip().split(b' ',2);seq=int(seq)
                if command==b'read 1':
                    self.control.put_nowait(frame(seq,b'ID 1 voltage=12000mV\r\n')+frame(seq,kind=b'DONE'))
                elif command.startswith(b'drive '):
                    self.drive_seq=seq
                    self.control.put_nowait(frame(seq,b'$SPOTDRIVE started seq=1\r\n'))
                elif command==b'targets':
                    self.next_seq=seq  # Wait for the test to provide its own DONE.
                else:raise AssertionError(command)
            elif data.startswith(b'@S '):
                seq=self.drive_seq
                self.control.put_nowait(frame(seq,b'$SPOTDRIVE stopped reason=ok\r\nOK\r\n')+frame(seq,state)+frame(seq,kind=b'DONE'))
                self.console.put_nowait(b'$SPOTDRIVE stopped reason=ok\r\nOK\r\nID11 incomplete')
            elif not data.startswith(b'@D '):raise AssertionError(data)

    monkeypatch.setattr(connection,'BleLink',Radio)

    async def scenario():
        task=asyncio.create_task(owner._session('robot','',0,''))
        async def until(predicate, timeout=2):
            deadline=time.monotonic()+timeout
            while not predicate():
                owner.pulse(owner.latest)
                assert not task.done(),logs
                assert time.monotonic()<deadline,logs
                app.processEvents()
                await asyncio.sleep(.01)
        try:
            await until(lambda:snapshots and snapshots[-1].get('phase')=='idle')
            owner.pulse((0,.5))
            await until(lambda:radio.drive_seq is not None)
            await until(lambda:snapshots[-1].get('phase')=='drive')
            owner.pulse(None)
            stopped=time.monotonic()
            await until(lambda:snapshots[-1].get('phase')=='idle')
            assert time.monotonic()-stopped < .5
            assert snapshots[-1]['connected'] and snapshots[-1]['can_drive']
            owner.command('targets')
            await until(lambda:radio.next_seq is not None)
            # Old console log prompts/errors, even after the new request,
            # cannot finish/cancel it. Neither can a stale control DONE.
            radio.console.put_nowait(b' ERROR: old fault\r\n# '+b'x'*9000)
            radio.control.put_nowait(frame(radio.drive_seq,kind=b'DONE'))
            radio.control.put_nowait(frame(radio.next_seq,b'# '))
            await asyncio.sleep(.15)
            assert snapshots[-1]['phase']=='busy'
            radio.control.put_nowait(frame(radio.next_seq,kind=b'DONE'))
            await until(lambda:snapshots[-1]['phase']=='idle')
            assert not any(b'syncstate' in d or d==b'\x03' for d in sent)
            assert any('ID11 incomplete' in s for s in logs)
        finally:
            owner.disconnect()
            await asyncio.wait_for(task,2)
    asyncio.run(scenario())


def test_console_overflow_does_not_disconnect_control_channel():
    async def scenario():
        radio=connection.BleLink()
        radio.control_mode=True
        for _ in range(300):radio.received_data(None,b'log bytes')
        assert radio.log_dropped>0
        assert b'BLE LOG overflow' in await radio.read()
        radio.received_control(None,frame(8,kind=b'DONE'))
        assert await radio.read_control() == frame(8,kind=b'DONE')
    asyncio.run(scenario())
