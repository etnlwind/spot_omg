"""Explicitly authorized bounded V76 floor start; preserve trace and never retry motion."""
import argparse,json,re,threading,time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download,write_report
from servo.imu_trace import download as download_imu

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute-authorized',action='store_true',required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--revision',default='s-native-v6-2-7-v76')
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic();done=threading.Event();begun=threading.Event();lock=threading.Lock()
    result=dict(command=[344,0],host_drive_window_s=4,automatic_retry=False);worker=None;motion=False
    def record(kind,text):
        with lock:
            with (a.output/'events.jsonl').open('a') as f:f.write(json.dumps(dict(host_s=time.monotonic()-start,kind=kind,text=text))+'\n')
    transport=BleTransport('SpotOMG-Bridge',timeout=.1)
    with Stm32Console('SpotOMG-Bridge',transport=transport) as c:
        def line(s):
            record('rx',s)
            if s.startswith('$SPOTDRIVE started'):begun.set()
            if s.startswith('$SPOTDRIVE stopped'):result['stop_result']=s;done.set()
            if s.startswith(('$SPOTDRIVE','ERROR','POSE_RESULT')):print(s,flush=True)
        def send(cmd,timeout=12):
            record('tx',cmd)
            with (a.output/(cmd.replace(' ','-')+'.txt')).open('w') as f:
                def receive(s):f.write(s+'\n');f.flush();line(s)
                r=c.send(cmd,timeout=timeout,on_line=receive)
            if not r.ok:raise RuntimeError(r.text)
            return r.text
        def heartbeat():
            if not begun.wait(8):return
            t=time.monotonic();seq=2
            while not done.is_set():
                elapsed=time.monotonic()-t
                cmd=f'@D {seq} 344 0' if elapsed<4 else f'@S {seq}'
                record('tx',cmd);transport.write((cmd+'\n').encode());seq+=1
                if elapsed>10:c.abort();record('abort','stop deadline exceeded');return
                done.wait(.15)
        try:
            state=send('syncstate')
            if not all(s in state for s in ('rev='+a.revision,'profile=s_native_v6_2_7','safety=ok','fault_code=0')):raise RuntimeError('Unexpected preflight state')
            status=send('status');voltages=[int(x) for x in re.findall(r'voltage=(\d+)mV',status)]
            if len(voltages)!=12 or min(voltages)<11000:raise RuntimeError('Preflight voltage insufficient or incomplete')
            if len(re.findall(r'hw=0x00',status))!=12:raise RuntimeError('Servo health not clear')
            has_imu='imutrace' in state.split('caps=')[-1].split()[0].split(',')
            if has_imu:download_imu(c,a.output/'previous-imutrace.txt')
            trace=send('jointtrace status');m=re.search(r'\$JT,M,1,(\d+),(\d+),',trace)
            if not m:raise RuntimeError('Trace metadata missing')
            if int(m[1]):download(c,a.output/'previous-jointtrace.txt')
            send('balance status')
            motion=True
            send('stand',timeout=30)
            ready=send('syncstate')
            if 'pose=stand' not in ready or 'safety=ok' not in ready:raise RuntimeError('Stand not confirmed')
            send('imu on');send('jointtrace arm')
            worker=threading.Thread(target=heartbeat,daemon=True);worker.start()
            motion=True;record('tx','drive 344 0 1');transport.write(b'drive 344 0 1\n')
            deadline=time.monotonic()+20;pending='';last=time.monotonic()
            while time.monotonic()<deadline:
                chunk=transport.read(max(1,transport.in_waiting))
                if chunk:
                    last=time.monotonic();pending+=chunk.decode('utf-8','replace')
                    while '\n' in pending:
                        s,pending=pending.split('\n',1);s=s.strip('\r')
                        if s.startswith('# '):s=s[2:]
                        line(s)
                    if done.is_set() and pending.endswith('# '):break
                elif done.is_set() and time.monotonic()-last>1:break
            if not done.is_set():raise RuntimeError('No stopped response')
            worker.join(timeout=2);motion=False
            send('imu off');result['final_state']=send('syncstate')
            download(c,a.output/'jointtrace.txt');write_report(a.output/'jointtrace.txt',a.output/'joint-analysis')
            result['trace_validated']=True
            if has_imu:
                download_imu(c,a.output/'imutrace.txt');result['imu_trace_validated']=True
            for cmd in ('baldiag','gaitdiag','trackingdiag','status','balance status'):
                try:send(cmd,timeout=8)
                except Exception as exc:result[cmd+'_error']=str(exc)
        except BaseException as exc:
            result['error']=str(exc);print('CAPTURE ERROR',str(exc),flush=True)
        finally:
            done.set()
            if worker:worker.join(timeout=2)
            if motion:
                try:c.abort();record('abort','capture failed; interrupt motion')
                except Exception as exc:result['abort_error']=str(exc)
            try:send('imu off')
            except Exception as exc:result['imu_off_error']=str(exc)
            try:result['final_state']=send('syncstate')
            except Exception as exc:result['final_state_error']=str(exc)
            (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
