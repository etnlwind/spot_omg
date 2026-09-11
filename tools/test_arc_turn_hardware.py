"""Explicit bounded real-robot arc test. Requires V41 and a released app BLE link.

Uses STM32 drive watchdog, sequence-controlled Stop, IMU and servo protections.
Does not replay raw motor positions or relax torque. Stops after any failure.
"""
import argparse,json,re,threading,time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport

def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--execute',action='store_true',help='Actually move the physical robot')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.execute:raise SystemExit('No motion: explicitly pass --execute')
    records=[];transport=BleTransport('SpotOMG-Bridge');console=Stm32Console('SpotOMG-Bridge',transport=transport)
    def save():args.output.write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    def send(command,timeout=15):
        print('>',command,flush=True)
        response=console.send(command,timeout=timeout,on_line=lambda line:print(line,flush=True))
        records.append(dict(command=command,text=response.text,ok=response.ok,time=time.time()));save()
        if not response.ok:raise RuntimeError(response.text)
        return response.text
    try:
        console.open();console.sync()
        state=send('syncstate')
        if not re.search(r'rev=shared-locomotion-v41\b',state):raise RuntimeError('Expected arc firmware V41')
        if 'safety=ok' not in state:raise RuntimeError('Initial safety state is not OK')
        timing=send('arctiming',30)
        peak=re.search(r'max_ms=(\d+)',timing)
        failures=re.search(r'failures=(\d+)',timing)
        if not peak or not failures or int(peak[1])>=10 or int(failures[1])!=0:
            raise RuntimeError('Arc computation must leave at least half the 20ms frame for I/O before motion')
        voltage=send('read 1');match=re.search(r'voltage=(\d+)mV',voltage)
        if not match or int(match[1])<11400:raise RuntimeError('Voltage is below test preparation threshold or missing')
        if 'pose=stand ' not in state or 'torque=on ' not in state:send('stand',30)
        send('gaitprofile arcturn');send('locomotiondiag')
        for direction in (-1,1):
            before=send('locomotiondiag')
            stop=threading.Event();errors=[]
            def feed():
                try:
                    for index in range(40):
                        if stop.wait(.1):return
                        transport.write(f'@D {index+2} 0 {direction*1000}\n'.encode())
                    transport.write(b'@S 1000\n')
                except Exception as exc:errors.append(str(exc))
            thread=threading.Thread(target=feed);thread.start()
            try:send(f'drive 0 {direction*1000} 1',12)
            finally:
                stop.set();thread.join(timeout=2)
                if thread.is_alive():raise RuntimeError('Drive sender did not terminate')
            if errors:raise RuntimeError(errors)
            state=send('syncstate');diag=send('locomotiondiag');send('gaitdiag');send('baldiag')
            start_yaw=re.search(r'yaw10=(-?\d+)',before);end_yaw=re.search(r'yaw10=(-?\d+)',diag)
            if start_yaw and end_yaw and 'valid=1' in before and 'valid=1' in diag:
                delta=((int(end_yaw[1])-int(start_yaw[1])+1800)%3600-1800)/10
                records.append(dict(direction=direction,imu_heading_change_deg=delta,
                                    note='Heading only; not a measurement of sideways translation'))
                save();print('Measured IMU heading change:',delta,'deg',flush=True)
            if 'safety=ok' not in state or 'fault=0' not in diag:raise RuntimeError('Post-turn fault')
            late=re.search(r'late=(\d+)',diag)
            if not late or int(late[1])>0:raise RuntimeError('Embedded loop missed its deadline; do not run the other direction')
        send('landing',30)
    except BaseException:
        try:console.abort()
        except Exception:pass
        raise
    finally:
        save();console.close()
if __name__=='__main__':main()
