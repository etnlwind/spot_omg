"""Read-only preflight and host BLE sampling timing for a response experiment.

No torque, position, profile-write, fault-reset or firmware-update commands.
Host request/reply bounds are NOT MCU sample times or motor response delays.
"""
import argparse
import json
import re
import statistics
import time
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=30)
    a=p.parse_args()
    if not 3<=a.samples<=100:p.error('samples must be 3..100')
    a.output.mkdir(parents=True,exist_ok=False)
    events=[];start=time.perf_counter()
    def persist():
        (a.output/'read-only-events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
    try:
        with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as c:
            def read(command):
                assert command in ('syncstate','profile','status','servoconfig 11','servoconfig 12','read 12')
                before=time.perf_counter()-start
                response=c.send(command,timeout=8)
                row=dict(command=command,host_begin_s=before,host_end_s=time.perf_counter()-start,
                         status=response.status,text=response.text)
                events.append(row);persist()
                if not response.ok:raise RuntimeError(response.text)
                return row
            for command in ('syncstate','profile','status','servoconfig 11','servoconfig 12'):
                row=read(command);print(command, row['text'],flush=True)
                if command=='syncstate' and ('backend=sim' in row['text'] or '-sim ' in row['text']):
                    raise RuntimeError('A physical robot is required; simulator response rejected')
            samples=[]
            for _ in range(a.samples):
                row=read('read 12')
                m=re.search(r'ID 12 pos=(\d+)',row['text'])
                if not m:raise RuntimeError('Missing RR J3 position response')
                row['position_ticks']=int(m[1]);samples.append(row)
            durations=[(r['host_end_s']-r['host_begin_s'])*1000 for r in samples]
            gaps=[(b['host_end_s']-a['host_end_s'])*1000 for a,b in zip(samples,samples[1:])]
            report=dict(read_only=True,samples=len(samples),
                request_reply_ms=dict(min=min(durations),median=statistics.median(durations),max=max(durations)),
                successive_reply_ms=dict(min=min(gaps),median=statistics.median(gaps),max=max(gaps)),
                position_range_ticks=max(r['position_ticks'] for r in samples)-min(r['position_ticks'] for r in samples),
                limitation='These are host BLE intervals, not MCU/encoder timestamps. No acceleration or motion delay measured.')
            (a.output/'sampling-summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report,indent=2),flush=True)
    except Exception as exc:
        (a.output/'failure.json').write_text(json.dumps(dict(read_only=True,error=str(exc),
            successful_responses=len(events)),indent=2),encoding='utf-8')
        raise
    finally:persist()


if __name__=='__main__':main()
