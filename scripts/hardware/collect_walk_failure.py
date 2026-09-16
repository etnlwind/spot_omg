"""Read existing failure evidence; never move, recover, arm or clear the robot."""
import argparse
import json
import re
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    events=[]
    def save():
        (a.output/'events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
    try:
        with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as c:
            for command in ('syncstate','jointtrace status','log show 512','gaitdiag',
                            'trackingdiag','baldiag','locomotiondiag','status','targets','profile','imudiag'):
                partial=a.output/(command.replace(' ','-')+'-stream.txt')
                with partial.open('w',encoding='utf-8') as stream:
                    def record(line):
                        stream.write(line+'\n');stream.flush()
                    r=c.send(command,timeout=40 if command.startswith('log ') else 20,on_line=record)
                events.append(dict(command=command,status=r.status,text=r.text));save()
                (a.output/(command.replace(' ','-')+'.txt')).write_text(r.text,encoding='utf-8')
                print(command,r.text,flush=True)
            trace=next(e['text'] for e in events if e['command']=='jointtrace status')
            meta=re.search(r'\$JT,M,1,(\d+),(\d+),',trace)
            if meta and int(meta[1])>0:
                download(c,a.output/'jointtrace.txt')
                print('Existing joint trace saved; no new capture armed',flush=True)
            for i in range(1,13):
                r=c.send(f'servoconfig {i}',timeout=15)
                events.append(dict(command=f'servoconfig {i}',status=r.status,text=r.text));save()
            print('Read-only collection finished',flush=True)
    except Exception as exc:
        (a.output/'failure.json').write_text(json.dumps(dict(error=str(exc),events=len(events))),encoding='utf-8')
        raise


if __name__=='__main__':main()
