"""Download and validate V77 timed IMU pages; no motion commands."""
from pathlib import Path
import csv

def parse(text):
    rows=[];meta=None;ended=False
    for line in text.splitlines():
        if not line.startswith('$IT,'):continue
        fields=line.split(',')[1:];kind=fields.pop(0)
        if kind=='END':ended=True;continue
        values=list(map(int,fields))
        if kind=='M':
            if meta is not None or len(values)!=4 or values[0]!=1 or not 0<=values[1]<=512:
                raise ValueError('Invalid IMU metadata')
            meta=values
        elif kind=='P':
            if meta is None or len(values)!=3 or values[0]!=len(rows) or not 0<=values[1]<=12 or values[2]!=meta[1] or values[0]+values[1]>meta[1]:
                raise ValueError('Invalid IMU page')
        elif kind=='S':
            if len(values)!=8 or values[0]!=len(rows) or values[6] not in (0,1) or values[7] not in (0,1,2):
                raise ValueError('Invalid IMU sample')
            if not 0<=values[1]<=0xffffffff:raise ValueError('Invalid clock')
            if rows and not 0<((values[1]-rows[-1][1])&0xffffffff)<60000:raise ValueError('Invalid IMU time order')
            rows.append(values)
        else:raise ValueError('Unknown IMU record')
    if meta is None or not ended or len(rows)!=meta[1]:raise ValueError('Incomplete IMU capture')
    return meta,rows

def download(console,path):
    path=Path(path);parts=[];offset=0
    partial=path.with_suffix(path.suffix+'.partial')
    while True:
        response=console.send(f'imutrace dump {offset} 12',timeout=8)
        # Preserve replies even if a status error or transport anomaly is present.
        parts.append(response.text);partial.write_text('\n'.join(parts)+'\n')
        if not response.ok:raise RuntimeError(response.text)
        records=[s for s in response.text.splitlines() if s.startswith('$IT,')]
        page=[s for s in records if s.startswith('$IT,P,')]
        if len(page)!=1:raise ValueError('Missing IMU page boundary')
        begin,count,total=map(int,page[0].split(',')[2:])
        if begin!=offset or not 0<=count<=12 or total>512:raise ValueError('Unexpected IMU page')
        offset+=count
        if '$IT,END' in records:break
        if count==0 or offset>=total:raise ValueError('Missing IMU END')
    text='\n'.join(parts)+'\n';meta,rows=parse(text);path.write_text(text);partial.unlink()
    with path.with_suffix('.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(('index','mcu_ms','roll10','pitch10','phase1000','rate1000','valid','fault'))
        writer.writerows(rows)
    return meta,rows
