"""One bounded real-robot gait capture; no motion without --execute.

Uses existing firmware protections, sequence Stop and watchdog. Failure data is
saved before returning an error. Does not change policy, gains or servo limits.
"""
import argparse,json,re,threading,time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import write_report,download

def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--direction',choices=('forward','left','right'),required=True)
    p.add_argument('--prepare-stand',action='store_true',help='Explicitly command Stand before this bounded test')
    p.add_argument('--seconds',type=float,default=5);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not args.execute:raise SystemExit('No motion: --execute is required')
    if not 1<=args.seconds<=6:raise SystemExit('Capture duration must be 1..6 seconds')
    if args.output.exists():raise SystemExit('Use a new output directory to preserve earlier captures')
    args.output.mkdir(parents=True);records=[]
    transport=BleTransport('SpotOMG-Bridge');console=Stm32Console('SpotOMG-Bridge',transport=transport)
    def save():
        (args.output/'commands.json').write_text(json.dumps(records,indent=2,ensure_ascii=False))
    def send(command,timeout=20,required=True):
        print('>',command,flush=True)
        r=console.send(command,timeout=timeout,on_line=lambda line:print(line,flush=True))
        records.append(dict(command=command,text=r.text,status=r.status,host_time=time.time()));save()
        if required and not r.ok:raise RuntimeError(r.text)
        return r
    motion_started=False
    try:
        console.open();console.sync();state=send('syncstate').text
        if not re.search(r'\brev=shared-locomotion-v47\b',state) or 'jointtrace' not in state:raise RuntimeError('V47 jointtracepage firmware required')
        if 'safety=ok' not in state or 'fault_code=0' not in state:raise RuntimeError('Initial safety fault')
        if args.prepare_stand:
            send('stand',timeout=75)
            state=send('syncstate').text
            if 'torque=on ' not in state or 'safety=ok' not in state or 'fault_code=0' not in state:
                raise RuntimeError('Stand preparation did not reach an enabled, fault-free state')
        elif 'pose=stand ' not in state or 'torque=on ' not in state:
            raise RuntimeError('Prepare and verify Stand before executing this capture')
        voltage=send('read 1').text;match=re.search(r'voltage=(\d+)mV',voltage)
        if not match or int(match[1])<11000:raise RuntimeError('Preflight voltage absent or below 11.0V')
        send('profile');send('locomotiondiag');send('jointtrace arm')
        linear,yaw={'forward':(1000,0),'left':(0,-1000),'right':(0,1000)}[args.direction]
        done=threading.Event();errors=[]
        def feed():
            try:
                for n in range(int(args.seconds*10)):
                    if done.wait(.1):return
                    transport.write(f'@D {n+2} {linear} {yaw}\n'.encode())
                transport.write(b'@S 10000\n')
            except Exception as exc:errors.append(str(exc))
        sender=threading.Thread(target=feed);motion_started=True;sender.start()
        try:response=send(f'drive {linear} {yaw} 1',timeout=args.seconds+15,required=False)
        finally:
            done.set();sender.join(timeout=2)
            transport.write(b'@S 10001\n')
        if sender.is_alive():raise RuntimeError('Sender did not stop')
        if errors:raise RuntimeError(errors)
        state=send('syncstate',required=False).text
        send('locomotiondiag',required=False)
        raw=download(console,args.output/'trace.log')
        send('gaitdiag',timeout=60,required=False)
        try:write_report(raw,args.output/'analysis')
        except ValueError as exc:(args.output/'analysis-error.txt').write_text(str(exc))
        if not response.ok or 'safety=ok' not in state or 'fault_code=0' not in state:raise RuntimeError('Motion failed; partial trace retained, do not continue other directions')
    except BaseException:
        if motion_started:
            try:transport.write(b'@S 10002\n')
            except Exception:pass
        raise
    finally:save();console.close()
if __name__=='__main__':main()
