"""Install V90-R2 via correlated BLE control + existing OTA; preserve foot settings."""
import argparse
import asyncio
import hashlib
import json
from pathlib import Path
import re
import secrets
import sys
import time
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'apps/windows'),str(ROOT/'tools/servo_tool')]
from spot_controller.connection import BleLink
from spot_controller.control_channel import ControlStream, request

async def run(args):
    args.output.mkdir(parents=True,exist_ok=True)
    file=args.output/(('prepare' if args.stage=='prepare-no-motion' else args.stage)+'.json')
    if file.exists():raise RuntimeError('Use a new output directory; preserve previous evidence')
    report=dict(stage=args.stage,success=False,events=[],sha256=hashlib.sha256(args.image.read_bytes()).hexdigest())
    def save():file.write_text(json.dumps(report,indent=2)+'\n')
    radio=BleLink();drainer=None;sequence=secrets.randbelow(0xffffff00)+1;stream=ControlStream()
    async def rpc(command):
        nonlocal sequence
        sequence+=1;seq=sequence
        event=dict(command=command,data=[]);report['events'].append(event);save()
        console_start=len(report.get('console',[]))
        await radio.write_control(request(seq,command.encode()))
        async def receive():
            while True:
                for number,kind,payload in stream.feed(await radio.read_control()):
                    if number!=seq:continue
                    if kind=='DATA':event['data'].append(payload.decode(errors='replace'))
                    else:return
        await asyncio.wait_for(receive(),75 if command=='landing' else 15)
        text=''.join(event['data'])
        if command=='landing' or command.startswith('servoconfig '):
            for _ in range(40):
                logs=''.join(report.get('console',[])[console_start:])
                complete = bool(re.search(r'POSE landing reason=[^\r\n]+\r?\n',logs)) if command=='landing' else bool(re.search(r'SERVO_CONFIG id='+command.split()[1]+r' addr=0: (?:[0-9A-F]{2} ?){48,}\r?\n',logs))
                if complete or 'ERROR:' in text:break
                await asyncio.sleep(.25)
            event['diagnostic_reply']=logs
            text+='\n'+logs
        save();print(command+'\n'+text,flush=True)
        if 'ERROR:' in text:raise RuntimeError(text)
        return text
    async def state():
        text=await rpc('syncstate')
        line=next(l for l in text.splitlines() if l.startswith('$SPOTSTATE '))
        return dict(w.split('=',1) for w in line.split()[1:] if '=' in w)
    async def healthy():
        text=await rpc('status')
        voltage=[int(v) for v in re.findall(r'voltage=(\d+)mV',text)]
        assert len(voltage)==12 and min(voltage)>=10500,text
        assert len(re.findall('hw=0x00',text))==12,text
        return text
    try:
        await radio.open()
        assert radio.control_characteristic,'Dedicated control channel required'
        radio.control_mode=True
        async def drain():
            while True:
                data=await radio.read()
                report.setdefault('console',[]).append(data.decode(errors='replace'))
                save()
        drainer=asyncio.create_task(drain())
        initial=await state();report['initial']=initial;save()
        if args.stage=='forward-diagnosis':
            for command in ('footlift show','safety','gaitdiag','log show 320'):
                await rpc(command)
            # Control DONE may precede delivery of the separate console stream.
            for _ in range(90):
                if '$SPOTLOG END' in ''.join(report.get('console',[])):break
                await asyncio.sleep(0.5)
            report['success']=True;return
        if args.stage=='imu-recover':
            await rpc('imurecover')
            report['final']=await state()
            report['success']=True;return
        if args.stage=='motion-failure':
            for command in ('imu off','servoconfig 1','profile','gaitdiag','syncstate'):
                await rpc(command)
            await asyncio.sleep(0.5)
            report['success']=True;return
        if args.stage=='imu-sample':
            previous=await rpc('imu status')
            try:
                await rpc('imu on')
                await asyncio.sleep(2)
            finally:
                if 'IMU log: on' not in previous:await rpc('imu off')
            report['success']=True;return
        if args.stage=='attitude':
            for command in ('imudiag','baldiag','status'):
                await rpc(command)
            await asyncio.sleep(0.5)
            report['success']=True;return
        if args.stage=='diagnose':
            for command in ('ping 1','ping 2','ping 7','busprobe 1','busprobe 7','read 1'):
                await rpc(command)
            await asyncio.sleep(0.5)
            report['success']=True;return
        if args.stage=='inspect':
            await healthy();report['success']=True;return
        if args.stage=='landing':
            # A normal fresh Landing command invokes the firmware's own
            # commandretry preflight. Do not clear/bypass guards on the host.
            assert initial.get('rev')==args.source_revision,initial
            assert initial.get('pose') in ('stand','landing'),initial
            assert (initial.get('safety')=='ok' and initial.get('fault_code')=='0') or (
                initial.get('fault_code')=='11' and 'commandretry' in initial.get('caps','').split(',')),initial
            await healthy()
            reply=await rpc('landing')
            assert re.search(r'POSE landing reason=complete(?:-residual)?(?: |$)',reply),reply
            final=await state()
            assert final['pose']=='landing' and final['torque']=='on' and final['safety']=='ok' and int(final['error'])<=40,final
            report['landing_confirmed']=True;report['final']=final
            report['success']=True;return
        expected=args.source_revision if args.stage in ('prepare','prepare-no-motion') else args.target_revision
        assert initial.get('rev')==expected,initial
        assert initial.get('safety')=='ok' and initial.get('fault_code')=='0',initial
        if args.stage=='prepare-no-motion':
            # Explicit user override: install firmware first without another Landing move.
            await healthy()
            report['foot_lift_mm']=[int(initial['lift_'+l]) for l in ('fl','fr','rl','rr')]
            report['user_requested_firmware_first']=True
            report['landing_confirmed']=False
            await rpc('relax')
            final=await state()
            assert final['torque']=='off' and final['safety']=='ok',final
            report['torque_off']=True;report['final']=final
        elif args.stage=='prepare':
            await healthy()
            report['foot_lift_mm']=[int(initial['lift_'+l]) for l in ('fl','fr','rl','rr')]
            landing=await rpc('landing')
            assert re.search(r'POSE landing reason=complete(?:-residual)?(?: |$)',landing),landing
            landed=await state()
            assert landed['pose']=='landing' and landed['torque']=='on' and landed['safety']=='ok' and int(landed['error'])<=40,landed
            report['landing_confirmed']=True;save()
            await rpc('relax')
            final=await state();assert final['torque']=='off' and final['safety']=='ok',final
            for i in range(1,13):
                text=await rpc(f'servoconfig {i}')
                from scripts.hardware.measure_rr_j3_response import registers
                assert registers(text,i)[40]==0
            report['torque_off']=True
        elif args.stage=='verify':
            # Verify reboot pose before issuing any new posture motion.
            prior=json.loads((args.output/'prepare.json').read_text())
            assert initial['torque']=='off',initial
            if not prior.get('user_requested_firmware_first'):assert initial['pose']=='landing',initial
            assert 'footliftpersist' in initial['caps'].split(','),initial
            prior=json.loads((args.output/'prepare.json').read_text())
            assert prior['success'] and prior['sha256']==report['sha256']
            values=prior['foot_lift_mm']
            reply=await rpc('footlift save '+' '.join(map(str,values)))
            assert 'OK footlift saved' in reply,reply
            fresh=await state()
            assert [int(fresh['lift_'+l]) for l in ('fl','fr','rl','rr')]==values,fresh
            report['foot_lift_mm']=values
            report['final']=fresh
        report['success']=True
    except BaseException as e:
        report['error']=repr(e);raise
    finally:
        report['finished_epoch']=time.time();save()
        if drainer:
            drainer.cancel()
            try:await drainer
            except (asyncio.CancelledError,Exception):pass
        await radio.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['imu-recover','inspect','landing','diagnose','attitude','imu-sample','motion-failure','forward-diagnosis','prepare-no-motion','prepare','ota','verify'])
    p.add_argument('--output',type=Path,required=True);p.add_argument('--image',type=Path,required=True)
    p.add_argument('--source-revision',default='attitudepd-v4-v90-r1')
    p.add_argument('--target-revision',default='attitudepd-v4-v90-r2')
    args=p.parse_args()
    if args.stage=='ota':
        prior=json.loads((args.output/'prepare.json').read_text())
        assert prior['success'] and prior['torque_off']
        assert prior['landing_confirmed'] or prior.get('user_requested_firmware_first')
        assert time.time()-prior['finished_epoch']<600
        assert prior['sha256']==hashlib.sha256(args.image.read_bytes()).hexdigest()
        from servo.cli import update_stm32_firmware
        result=update_stm32_firmware(SimpleNamespace(image=args.image,chunk_size=120,ble_name='SpotOMG-Bridge',skip_landing=True))
        assert result==0
        (args.output/'ota.json').write_text(json.dumps(dict(success=True,sha256=prior['sha256']))+'\n')
    else:asyncio.run(run(args))
