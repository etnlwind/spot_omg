"""User-supported full-input walking for nominal 30 cycles; preserves startup trace."""
import argparse,json,threading,time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download,write_report

def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--body-secured',action='store_true',required=True)
 ap.add_argument('--output',type=Path,required=True)
 ap.add_argument('--profile',default='s_native_v6_2_1',choices=['s_native_v6_2_1','s_native_v6_2_2','s_native_v6_2_3','s_native_v6_2_4','s_native_v6_2_5'])
 ap.add_argument('--revision',default='s-native-v6-2-1-v63')
 ap.add_argument('--trace-phase',choices=['walk','stop'],default='walk')
 ap.add_argument('--seconds',type=float,default=60.)
 args=ap.parse_args()
 if not 2<=args.seconds<=60:ap.error('seconds must be 2..60')
 p=args.output;p.mkdir(parents=True,exist_ok=False)
 origin=time.monotonic();stop=threading.Event();started=threading.Event();worker=None;motion=False
 period=json.loads((Path(__file__).resolve().parents[2]/'config/locomotion_profiles.json').read_text())['profiles'][args.profile]['params'][0]
 result={'linear':1000,'yaw':0,'nominal_cycles':args.seconds/period,'nominal_duration_s':args.seconds,'profile':args.profile,'revision':args.revision,'support':'user-secured body; feet off floor','full_cycle_count_measured':False}
 lock=threading.Lock()
 def log(kind,text):
  with lock:
   with (p/'transcript.jsonl').open('a') as f:f.write(json.dumps({'host_s':time.monotonic()-origin,'kind':kind,'text':text})+'\n')
 transport=BleTransport('SpotOMG-Bridge',timeout=.1)
 with Stm32Console('SpotOMG-Bridge',transport=transport) as c:
  def line(text):
   log('rx',text)
   if text.startswith('$SPOTDRIVE started'):started.set();print(text,flush=True)
   if text.startswith('$SPOTDRIVE stopped'):result['stop_result']=text;stop.set();print(text,flush=True)
   if text.startswith(('ERROR','STOPPED','Gait diagnostics','POSE_RESULT')):print(text,flush=True)
  def send(cmd,timeout=15):
   log('tx',cmd);print('COMMAND',cmd,flush=True)
   r=c.send(cmd,timeout=timeout,on_line=line)
   if not r.ok:raise RuntimeError(r.text)
   return r.text
  def heartbeat():
   if not started.wait(5):return
   began=time.monotonic();seq=2;next_report=10
   try:
    while not stop.is_set():
     elapsed=time.monotonic()-began
     cmd=f'@D {seq} 1000 0' if elapsed<args.seconds else f'@S {seq}'
     log('realtime_tx',cmd);transport.write((cmd+'\n').encode());seq+=1
     if elapsed>=next_report:print(f'Full-input run {elapsed:.1f}s / {args.seconds:g}s',flush=True);next_report+=10
     if elapsed>args.seconds+8:c.abort();log('abort','Stop response deadline exceeded');return
     stop.wait(.18)
   except Exception as exc:result['heartbeat_error']=str(exc);log('heartbeat_error',str(exc))
  try:
   state=send('syncstate')
   if not all(v in state for v in ['torque=off','safety=ok','fault_code=0','rev='+args.revision,'profile='+args.profile]):raise RuntimeError('Unexpected preflight state')
   if 'speed=3400 acceleration=254' not in send('profile'):raise RuntimeError('Unexpected servo profile')
   send('imu off')
   # drive performs its own supervised S entry; do not duplicate stand.
   motion=True
   # The previous complete trace was downloaded and validated before this reset.
   send('jointtrace stop' if args.trace_phase=='stop' else 'jointtrace arm')
   worker=threading.Thread(target=heartbeat,daemon=True);worker.start()
   cmd='drive 1000 0 1';log('tx',cmd);transport.write((cmd+'\n').encode())
   deadline=time.monotonic()+args.seconds+20;pending='';last_rx=time.monotonic();rows=[]
   while time.monotonic()<deadline:
    chunk=transport.read(max(1,transport.in_waiting))
    if chunk:
     last_rx=time.monotonic();pending+=chunk.decode('utf-8','replace')
     while '\n' in pending:
      text,pending=pending.split('\n',1);text=text.strip('\r')
      if text.startswith('# '):text=text[2:]
      if text:rows.append(text);line(text)
     if stop.is_set() and pending.endswith('# '):result['drive_prompt_received']=True;break
    elif stop.is_set() and time.monotonic()-last_rx>2:
     result['drive_prompt_received']=False;break
   (p/'drive-response.txt').write_text('\n'.join(rows)+'\n'+pending)
   if 'stop_result' not in result:raise RuntimeError('No stopped response before deadline')
   stop.set();worker.join(timeout=2)
   send('relax')
   result['post_relax_state']=send('syncstate')
   # Small validated pages avoid the long diagnostic response truncation seen over BLE.
   print('Downloading startup joint trace',flush=True)
   download(c,p/'jointtrace.txt');write_report(p/'jointtrace.txt',p/'joint-analysis');result['trace_validated']=True
   # Preserve every received balance line, even if the unpaged response is incomplete.
   try:(p/'baldiag.txt').write_text(send('baldiag',timeout=12)+'\n')
   except Exception as exc:result['balance_download_error']=str(exc)
  except BaseException as exc:
   result['error']=str(exc);print('RUN ERROR',str(exc),flush=True)
   stop.set()
   if worker:worker.join(timeout=2)
   if motion:
    try:c.abort();time.sleep(.5)
    except Exception as err:result['abort_error']=str(err)
  finally:
   stop.set()
   if worker:worker.join(timeout=2)
   if motion:
    try:result['relax_result']=send('relax',timeout=20)
    except Exception as exc:result['relax_error']=str(exc)
   try:result['final_state']=send('syncstate')
   except Exception as exc:result['final_state_error']=str(exc)
   (p/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
