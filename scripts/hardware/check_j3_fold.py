"""User-supervised, single FL J3 110->130 degree test using bounded firmware moves."""
import argparse
import json
import re
import time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--body-secured', action='store_true', required=True)
    ap.add_argument('--output', type=Path, required=True)
    args=ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    events=[]
    start=time.monotonic()
    result={'servo_id':3,'leg':'FL','preparation_deg':110,'target_deg':130,
            'speed_register':3400,'acceleration_register':254,
            'timing_limit':'Host BLE command/read intervals, not servo peak speed or exact settling time.'}
    def persist():
        (args.output/'events.json').write_text(json.dumps(events,indent=2)+'\n')
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as console:
        def send(command):
            before=time.monotonic()-start
            response=console.send(command,timeout=6)
            event={'command':command,'begin_s':before,'end_s':time.monotonic()-start,'text':response.text,'status':response.status}
            events.append(event);persist()
            print(command, response.text,flush=True)
            if not response.ok:raise RuntimeError(response.text)
            return event
        def read():
            e=send('read 3')
            match=re.search(r'ID 3 pos=(\d+) speed=(-?\d+) load=(-?\d+) voltage=(\d+)mV temp=(\d+)C current=(-?\d+) moving=(\d+) hw=0x([0-9A-Fa-f]+)',e['text'])
            if not match:raise RuntimeError('Missing complete J3 readback')
            vals=list(map(int,match.groups()[:-1]))+[int(match.group(8),16)]
            s=dict(zip(('position','speed','load','voltage_mv','temperature_c','current_raw','moving','hardware_error'),vals))
            s['angle_deg']=(s['position']-1965)*360/4096
            e['sample']=s;persist()
            if s['hardware_error'] or s['temperature_c']>=55 or s['voltage_mv']<10000:
                raise RuntimeError('Health guard failed')
            return e
        saved_profile=None
        torque_may_be_on=False
        try:
            state=send('syncstate')['text']
            if not all(x in state for x in ('torque=off','safety=ok','fault_code=0','rev=s-native-v6-2-1-v63')):
                raise RuntimeError('Expected idle v63 with torque off and no fault')
            config=send('profile')['text']
            match=re.search(r'speed=(\d+) acceleration=(\d+)',config)
            if not match:raise RuntimeError('Missing original profile')
            saved_profile=tuple(map(int,match.groups()))
            first=read(); pos=first['sample']['position']
            if not 100<=first['sample']['angle_deg']<=140:
                raise RuntimeError('Current angle outside planned preparation envelope')
            send('profile 200 20')
            prep=1965+round(110*4096/360)
            for step in range(4):
                delta=prep-pos
                if abs(delta)<=8:break
                target=pos+max(-180,min(180,delta))
                torque_may_be_on=True
                send(f'move 3 {target}')
                deadline=time.monotonic()+2.5
                while True:
                    sample=read()['sample'];pos=sample['position']
                    if abs(pos-target)<=8:break
                    if time.monotonic()>=deadline:raise RuntimeError('Preparation did not settle')
                    time.sleep(.06)
            if abs(pos-prep)>8:raise RuntimeError('Preparation incomplete')
            result['prepared_position']=pos
            send('profile 3400 254')
            final=1965+round(130*4096/360)
            if not 0<final-pos<=256:raise RuntimeError('Fast move exceeds bounded delta')
            motion=send(f'move 3 {final}')
            result['fast_move_command']=motion
            observations=[]
            deadline=time.monotonic()+1.2
            while True:
                sample=read();observations.append(sample)
                if time.monotonic()>=deadline:break
            result['fast_samples']=observations
            reached=[e for e in observations if abs(e['sample']['position']-final)<=12]
            result['reached_within_12_ticks']=bool(reached)
            if reached:
                result['first_observed_reached_host_upper_bound_ms']=(reached[0]['end_s']-motion['begin_s'])*1000
            result['final_error_deg']=(observations[-1]['sample']['position']-final)*360/4096
            result['result']='completed' if reached else 'target_not_reached'
        except BaseException as exc:
            result['result']='aborted';result['error']=str(exc)
            raise
        finally:
            if torque_may_be_on:
                try:
                    send('relax 3');result['j3_torque_off_command_ok']=True
                except Exception as exc:
                    result['j3_torque_off_command_ok']=False;result['cleanup_error']=str(exc)
            if saved_profile:
                try:send(f'profile {saved_profile[0]} {saved_profile[1]}')
                except Exception as exc:result['profile_restore_error']=str(exc)
            try:result['final_state']=send('syncstate')['text']
            except Exception as exc:result['final_state_error']=str(exc)
            persist()
    print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':main()
