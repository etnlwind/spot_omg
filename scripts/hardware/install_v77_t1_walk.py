"""V77 installation stages: measured Landing, torque-off, OTA, readback.

No walking, Stand, calibration/EEPROM writes or automatic motion retries.
The prepare stage is required in this session before the separate OTA stage.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.cli import update_stm32_firmware
from scripts.hardware.measure_rr_j3_response import registers

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/v77-t1/walk-install'
OUT.mkdir(parents=True,exist_ok=True)
IMAGE=ROOT/'artifacts/v77-t1/walk-firmware/s-native-v6-2-7-v77-t1-walk.bin'
HASH='076f87720bec92b6b9946a267d5ccc7759923555183f02bfa53fbd50b103843b'


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
    events=[]
    report=dict(stage=args.stage,firmware_sha256=HASH,completed_landing=False,all_torque_off=False)
    def save():
        (OUT/(args.stage+'.json')).write_text(json.dumps(dict(**report,events=events),indent=2),encoding='utf-8')
    try:
        with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as console:
            def send(command):
                response=console.send(command,timeout=75 if command=='landing' else 15,
                                      on_line=lambda line:print(line,flush=True))
                events.append(dict(command=command,status=response.status,text=response.text));save()
                if not response.ok:raise RuntimeError(response.text)
                return response
            initial=send('syncstate')
            required_revision='rev=s-native-v6-2-7-v77-t1-lift' if args.stage=='prepare' else 'rev=s-native-v6-2-7-v77-t1-walk'
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
                send('gaitprofile s_native_v6_2_5');send('profile');send('imutrace status');send('imutrace dump 0 12')
                send('relax');final=send('syncstate')
                assert 'torque=off' in final.text.split()
            report['success']=True;save()
    except Exception as exc:
        report['error']=str(exc);save();raise


if __name__=='__main__':main()
