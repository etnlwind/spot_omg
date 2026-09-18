"""Read-only V81 control-channel verification; no motion or torque commands."""
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import secrets
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'apps/windows'))
from spot_controller.connection import BleLink
from spot_controller.control_channel import ControlStream, request
from spot_controller.protocol import ConsoleStream


async def verify(output):
    if output.exists():raise RuntimeError('Preserve previous verification evidence')
    output.parent.mkdir(parents=True,exist_ok=True)
    report=dict(started=datetime.now(timezone.utc).isoformat(),motion_sent=False,
                success=False,commands=[],console=[])
    radio=BleLink();reader=None
    def save():output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    save()
    try:
        await radio.open()
        assert radio.control_characteristic is not None,'New bridge control characteristics missing'
        await radio.write(b'syncstate\n')
        parser=ConsoleStream();state=None
        async def bootstrap():
            nonlocal state
            while True:
                for kind,line in parser.feed(await radio.read()):
                    if line.startswith('$SPOTSTATE '):state=line
                    if kind=='prompt':return
        await asyncio.wait_for(bootstrap(),10)
        assert state and {'rev=attitudepd-v4-v81','torque=off','safety=ok'}.issubset(state.split()),state
        assert 'controlv1' in state
        report['initial_state']=state
        radio.control_mode=True
        async def logs():
            while True:
                data=await radio.read()
                report['console'].append(dict(at=time.monotonic(),text=data.decode('utf-8','replace')))
        reader=asyncio.create_task(logs())
        stream=ControlStream()
        sequence=secrets.randbelow(0xffffff00)+1
        async def rpc(command):
            nonlocal sequence
            assert command in ('help','syncstate','gaitdiag')
            sequence+=1;seq=sequence;started=time.monotonic()
            item=dict(command=command,sequence=seq,started=started,data=[])
            report['commands'].append(item);save()
            await radio.write_control(request(seq,(command+'\n').encode()))
            async def receive():
                while True:
                    for number,kind,data in stream.feed(await radio.read_control()):
                        if number!=seq:continue
                        if kind=='DATA':item['data'].append(data.decode())
                        else:
                            item.update(done=time.monotonic(),elapsed=time.monotonic()-started,
                                        console_bytes_at_done=sum(len(x['text'].encode()) for x in report['console']))
                            return
            await asyncio.wait_for(receive(),5)
            save();print(json.dumps(item),flush=True)
            return item
        help_result=await rpc('help')
        status=await rpc('syncstate')
        assert any('rev=attitudepd-v4-v81' in x and 'torque=off' in x and 'safety=ok' in x for x in status['data']),status
        await asyncio.sleep(1.5)
        report['logs_after_help_done']=any(x['at']>help_result['done'] for x in report['console'])
        report['logs_after_state_done']=any(x['at']>status['done'] for x in report['console'])
        assert report['logs_after_help_done'],'No evidence of asynchronous diagnostic delivery'
        assert report['logs_after_state_done'],'State query did not complete before the diagnostic tail'
        await rpc('gaitdiag')
        report['success']=True
    except BaseException as exc:
        report['error']=repr(exc);raise
    finally:
        if reader:
            reader.cancel()
            try:await reader
            except (asyncio.CancelledError,Exception):pass
        await radio.close();save()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    asyncio.run(verify(parser.parse_args().output))
