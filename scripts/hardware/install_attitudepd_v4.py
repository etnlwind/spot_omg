"""V80 installation: verified Landing, torque off, OTA, Landing readback.

No Stand, walking, EEPROM writes, or automatic motion retries.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time
from types import SimpleNamespace
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.cli import update_stm32_firmware
from scripts.hardware.measure_rr_j3_response import registers

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/attitudepd-v4/hardware-install'
IMAGE=ROOT/'artifacts/attitudepd-v4/firmware/attitudepd-v4-v80.bin'
HASH='79f465ff2238ba7f762adede957036d6da7c8aa35c6e5d962e1b1905d9c02f15'


def main():
    global OUT, IMAGE, HASH
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['prepare','ota','verify','finish-verify'])
    parser.add_argument('--output',type=Path,default=OUT,
        help='New evidence directory for a separately reviewed installation attempt')
    parser.add_argument('--image',type=Path,default=IMAGE)
    parser.add_argument('--sha256',default=HASH)
    parser.add_argument('--from-revision',default='attitudepd-v3-v79')
    parser.add_argument('--to-revision',default='attitudepd-v4-v80')
    args=parser.parse_args()
    OUT=args.output
    IMAGE=args.image;HASH=args.sha256
    if hashlib.sha256(IMAGE.read_bytes()).hexdigest()!=HASH:
        raise RuntimeError('Firmware image differs from reviewed build')
    OUT.mkdir(parents=True,exist_ok=True)
    if args.stage=='ota':
        prior=json.loads((OUT/'prepare.json').read_text())
        assert prior['success'] and prior['completed_landing'] and prior['all_torque_off']
        assert prior['firmware_sha256']==HASH and 0<=time.time()-prior['finished_epoch']<600
        result=update_stm32_firmware(SimpleNamespace(image=IMAGE,chunk_size=120,
            ble_name='SpotOMG-Bridge',skip_landing=True))
        assert result==0
        (OUT/'ota-result.json').write_text(json.dumps(dict(success=True,firmware_sha256=HASH,
            completed_at=datetime.now(timezone.utc).isoformat()),indent=2)+'\n')
        return
    path=OUT/(args.stage+'.json')
    if path.exists():raise RuntimeError('Preserve previous attempt; use a new evidence directory before retrying')
    events=[]
    report=dict(stage=args.stage,firmware_sha256=HASH,success=False,
        completed_landing=False,all_torque_off=False,physical_walk_test=False)
    def save():path.write_text(json.dumps(dict(**report,events=events),indent=2)+'\n')
    save()
    try:
        with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as c:
            def send(command):
                event=dict(command=command,started_at=datetime.now(timezone.utc).isoformat(),lines=[])
                events.append(event);save()
                def line(value):event['lines'].append(value);print(value,flush=True);save()
                response=c.send(command,timeout=75 if command=='landing' else 15,on_line=line)
                event.update(status=response.status,text=response.text);save()
                if not response.ok:raise RuntimeError(response.text)
                return response.text
            state=send('syncstate')
            revision=args.from_revision if args.stage=='prepare' else args.to_revision
            assert {f'rev={revision}','safety=ok','fault_code=0'}.issubset(state.split()),state
            if args.stage in ('verify','finish-verify'):
                assert 'profile=attitudepd_v4' in state.split(),state
                caps=state.split('caps=')[1].split()[0].split(',')
                assert 'attitudepd_v4' in caps
            status=send('status')
            voltages=[int(v) for v in re.findall(r'voltage=(\d+)mV',status)]
            temperatures=[int(v) for v in re.findall(r'temp=(\d+)C',status)]
            assert len(voltages)==len(temperatures)==12 and min(voltages)>=10500 and max(temperatures)<70,status
            assert len(re.findall(r'hw=0x00',status))==12,status
            if args.stage=='finish-verify':
                # Resume readback only after inspecting a successful residual
                # completion. Never repeat a posture move to hide a failed one.
                previous=json.loads((OUT/'verify.json').read_text())
                assert previous['firmware_sha256']==HASH
                landings=[e for e in previous['events'] if e['command']=='landing']
                assert len(landings)==1 and landings[0].get('status')=='ok'
                landed=landings[0]['text']
                assert 'POSE landing reason=complete-residual ' in landed
                report['resumed_from']='verify.json'
            else:
                landed=send('landing')
            # The supervisor accepts a stationary, low-effort residual only
            # inside its existing 40-tick envelope; confirm measured error below.
            assert re.search(r'POSE landing reason=complete(?:-residual)?(?: |$)',landed),landed
            state=send('syncstate')
            assert {'pose=landing','torque=on','safety=ok','fault_code=0'}.issubset(state.split()),state
            assert int(re.search(r'\berror=(\d+)',state)[1])<=40,state
            report['completed_landing']=True;report['landing_state']=state;save()
            config={}
            if args.stage=='prepare':send('relax')
            for i in range(1,13):
                config[str(i)]=registers(send(f'servoconfig {i}'),i)
                assert config[str(i)][40]==(0 if args.stage=='prepare' else 1),(i,config[str(i)][40])
            report['servo_registers']=config;save()
            if args.stage in ('verify','finish-verify'):
                prior=json.loads((OUT/'prepare.json').read_text())['servo_registers']
                assert all(config[i][:40]==prior[i][:40] for i in config),'Persistent servo registers changed'
                report['persistent_servo_settings_unchanged']=True
                send('profile');send('probeconfig show');send('gaitprofiles')
                send('relax')
                for i in range(1,13):
                    assert registers(send(f'servoconfig {i}'),i)[40]==0,f'Servo {i} torque still on'
            # Landing arrival was measured above, before releasing torque.
            # Once torque is OFF the joints can settle beyond the UI's 80-tick
            # pose label threshold. Do not repeat the motion just to restore
            # that label; require stationary healthy servos and torque OFF.
            settled=send('status')
            assert len(re.findall(r'moving=0(?: |$)',settled,re.M))==12,settled
            assert len(re.findall(r'hw=0x00',settled))==12,settled
            final_temperatures=[int(v) for v in re.findall(r'temp=(\d+)C',settled)]
            final_voltages=[int(v) for v in re.findall(r'voltage=(\d+)mV',settled)]
            assert len(final_temperatures)==len(final_voltages)==12,settled
            assert max(final_temperatures)<70 and min(final_voltages)>=10500,settled
            final=send('syncstate')
            assert {'torque=off','safety=ok','fault_code=0'}.issubset(final.split()),final
            assert report['completed_landing'] and all(config[i][40]==(0 if args.stage=='prepare' else 1) for i in config)
            report['torque_off_pose_label']=re.search(r'\bpose=(\S+)',final)[1]
            report['torque_off_settled_error_ticks']=int(re.search(r'\berror=(\d+)',final)[1])
            report.update(all_torque_off=True,final_state=final,success=True,finished_epoch=time.time())
            save()
    except BaseException as error:
        report['error']=str(error);save();raise


if __name__=='__main__':main()
