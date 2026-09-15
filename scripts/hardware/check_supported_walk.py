"""Short, user-supported v63 walking capture with heartbeat deadline and preserved diagnostics."""
import argparse
import json
import threading
import time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download, write_report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--body-secured',action='store_true',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic();events=[];summary={'linear':500,'yaw':0,'host_drive_window_s':4,'body_supported':True}
    done=threading.Event();begun=threading.Event();writer=None;motion_possible=False
    transport=BleTransport('SpotOMG-Bridge')
    def log(kind,text):
        events.append({'time_s':time.monotonic()-start,'kind':kind,'text':text})
        with (args.output/'transcript.jsonl').open('a') as f:f.write(json.dumps(events[-1])+'\n')
    with Stm32Console('SpotOMG-Bridge',transport=transport) as console:
        def on_line(line):
            log('rx',line)
            if line.startswith('$SPOTDRIVE started'):begun.set()
            if line.startswith('$SPOTDRIVE stopped'):done.set();summary['stop_result']=line
            if line.startswith(('$SPOTDRIVE','ERROR','POSE_','STOPPED','Gait diagnostics')):print(line,flush=True)
        def send(command,timeout=15,strict=True):
            log('tx',command);print('COMMAND',command,flush=True)
            response=console.send(command,timeout=timeout,on_line=on_line)
            (args.output/(command.split()[0]+'-response.txt')).write_text(response.text+'\n')
            if strict and not response.ok:raise RuntimeError(response.text)
            return response
        def heartbeat():
            if not begun.wait(5):return
            began=time.monotonic();seq=2
            try:
                while not done.is_set():
                    command=f'@D {seq} 500 0' if time.monotonic()-began<4 else f'@S {seq}'
                    log('realtime_tx',command);transport.write((command+'\n').encode());seq+=1
                    if done.wait(.18):break
                    if time.monotonic()-began>10:
                        log('abort','No stopped result after deadline');console.abort();return
            except Exception as exc:log('heartbeat_error',str(exc))
        try:
            state=send('syncstate').text
            if not all(s in state for s in ('torque=off','safety=ok','fault_code=0','profile=s_native_v6_2_1','rev=s-native-v6-2-1-v63')):
                raise RuntimeError('Expected v63 native profile, torque off, safety OK')
            profile=send('profile').text
            if 'speed=3400 acceleration=254' not in profile:raise RuntimeError('Unexpected profile')
            trace=send('jointtrace status').text
            if '$JT,M,1,0,0,' not in trace:raise RuntimeError('Existing trace must be preserved first')
            send('imu on')
            motion_possible=True
            send('stand',timeout=40)
            send('jointtrace arm')
            writer=threading.Thread(target=heartbeat,daemon=True);writer.start()
            drive=send('drive 500 0 1',timeout=25,strict=False)
            summary['drive_status']=drive.status
            done.set();writer.join(timeout=2)
            send('relax',strict=False)
            send('baldiag',timeout=25,strict=False)
            send('gaitdiag',timeout=25,strict=False)
            send('jointtrace status')
            print('Downloading paged joint trace',flush=True)
            download(console,args.output/'jointtrace.txt')
            write_report(args.output/'jointtrace.txt',args.output/'joint-analysis')
            summary['trace_validated']=True
        except BaseException as exc:
            summary['error']=str(exc);print('TEST ERROR',str(exc),flush=True)
            done.set()
            if writer:writer.join(timeout=2)
            if motion_possible:
                try:console.abort();time.sleep(.5)
                except Exception as cleanup:summary['abort_error']=str(cleanup)
        finally:
            done.set()
            if writer:writer.join(timeout=2)
            if motion_possible:
                try:summary['relax_result']=send('relax',timeout=20,strict=False).text
                except Exception as exc:summary['relax_error']=str(exc)
            try:send('imu off',strict=False)
            except Exception as exc:summary['imu_cleanup_error']=str(exc)
            try:summary['final_state']=send('syncstate',strict=False).text
            except Exception as exc:summary['state_error']=str(exc)
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':main()
