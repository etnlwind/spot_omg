"""V77 installation stages: measured Landing, torque-off, OTA, readback.

No walking, Stand, calibration/EEPROM writes or automatic motion retries.
The prepare stage is required in this session before the separate OTA stage.
"""
import argparse
import hashlib
import json
import time
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
from types import SimpleNamespace
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.cli import update_stm32_firmware
from scripts.hardware.measure_rr_j3_response import registers

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/v77-t1/width-install'
OUT.mkdir(parents=True,exist_ok=True)
IMAGE=ROOT/'artifacts/v77-t1/width-firmware/s-native-v6-2-7-v77-t1-width.bin'
HASH='b4bdc90506b966784d20764382ea3563efa68801055c2690f629bd7fb76148b5'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['prepare','ota','verify'])
    args=parser.parse_args()
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==HASH,'Firmware changed; review first'
    if args.stage=='ota':
        evidence=json.loads((OUT/'prepare.json').read_text(encoding='utf-8'))
        assert evidence['completed_landing'] and evidence['all_torque_off']
        assert evidence['firmware_sha256']==HASH
        result=update_stm32_firmware(SimpleNamespace(image=IMAGE,chunk_size=120,
                    ble_name='SpotOMG-Bridge',skip_landing=True))
        assert result==0
        return
    started=time.monotonic()
    def now():return datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec="milliseconds")
    events=[]
    report=dict(started_at=now(),verification_level="installation_readback",stage=args.stage,firmware_sha256=HASH,completed_landing=False,all_torque_off=False)
    def save():
        (OUT/(args.stage+'.json')).write_text(json.dumps(dict(**report,events=events),indent=2),encoding='utf-8')
    try:
        with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as console:
            def send(command):
                response=console.send(command,timeout=75 if command=='landing' else 15,
                                      on_line=lambda line:print(line,flush=True))
                events.append(dict(timestamp=now(),elapsed_ms=round((time.monotonic()-started)*1000,3),command=command,status=response.status,text=response.text));save()
                if not response.ok:raise RuntimeError(response.text)
                return response
            initial=send('syncstate')
            required_revision='rev=s-native-v6-2-7-v77-t1-param-j1' if args.stage=='prepare' else 'rev=s-native-v6-2-7-v77-t1-width'
            assert required_revision in initial.text.split(),initial.text
            assert {'safety=ok','fault_code=0'}.issubset(initial.text.split()),initial.text
            if args.stage=='verify':
                assert 'profile=s_native_v6_2_7' in initial.text.split()
                assert 's_native_v6_2_7' in initial.text
            landed=send('landing')
            assert any(line in ('OK','OK landing') for line in landed.lines)
            state=send('syncstate')
            assert {'pose=landing','torque=on','safety=ok','fault_code=0'}.issubset(state.text.split()),state.text
            report['completed_landing']=True;save()
            send('status')
            if args.stage=='prepare':send('relax')
            config={}
            for servo_id in range(1,13):
                raw=registers(send(f'servoconfig {servo_id}').text,servo_id)
                assert raw[40]==(0 if args.stage=='prepare' else 1),(servo_id,raw[40])
                config[servo_id]=raw
            report['servo_registers']=config
            if args.stage=='prepare':
                state=send('syncstate')
                assert 'torque=off' in state.text.split()
                report['all_torque_off']=True
            else:
                prior=json.loads((OUT/'prepare.json').read_text(encoding='utf-8'))['servo_registers']
                assert all(config[i][:40]==prior[str(i)][:40] for i in config),'Persistent servo settings changed'
                report['persistent_servo_settings_unchanged']=True
                send('targets');send('probeconfig show');send('probeconfig set 20 344 4000 all 0 0');
                configured=send('probeconfig show')
                assert {'lift_mm=20','linear=344','duration_ms=4000','legs=all','width_mm=0','fr_extra=0'}.issubset(configured.text.split())
                for width,fr in [(-20,0),(-20,1),(10,0),(0,0)]:
                    send(f'probeconfig set 28 344 4000 all {width} {fr}')
                    checked=send('probeconfig show')
                    assert {f'width_mm={width}',f'fr_extra={fr}'}.issubset(checked.text.split())
                report['parameter_readback_verified']=True
                send('gaitprofile s_native_v6_2_5');send('profile');send('imutrace status');send('imutrace dump 0 12')
                send('relax');final=send('syncstate')
                assert 'torque=off' in final.text.split()
            report['success']=True;report['ended_at']=now();report['elapsed_ms']=round((time.monotonic()-started)*1000,3);save()
    except Exception as exc:
        report['error']=str(exc);save();raise


if __name__=='__main__':main()
