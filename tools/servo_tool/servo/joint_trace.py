"""Offline analysis of timed STM32 feedback. Never connects to or moves a robot."""
from __future__ import annotations
import bisect
import csv
import json
import math
import statistics as stats
from pathlib import Path

DEG=360/4096

def signed_target(value):
    return -(value & 0x7fff) if value & 0x8000 else value

def wrapped_error(raw,target):
    return (raw-target+2048)%4096-2048

def parse(text):
    commands=[];samples=[];joints={};meta=None;ended=False
    for line in text.splitlines():
        if '$JT,' not in line:continue
        part=line[line.index('$JT,')+4:].strip().split(',')
        kind=part.pop(0)
        if kind=='END':ended=True;continue
        if kind=='R':
            if meta is None or len(part)!=1:raise ValueError('Invalid revision record')
            meta['revision']=part[0];continue
        try:v=list(map(int,part))
        except ValueError as exc:raise ValueError('Malformed jointtrace record') from exc
        if kind=='P':
            if len(v)!=3 or v[0]!=len(commands)+len(samples) or not 0<=v[1]<=12 or v[0]+v[1]>v[2]:raise ValueError('Invalid page boundary')
            continue
        if kind=='M':
            if len(v)!=8 or v[0]!=1:raise ValueError('Unsupported jointtrace schema')
            # Status responses preceding a dump carry no records. A new dump
            # starts a new capture; never silently concatenate two captures.
            if commands or samples:raise ValueError('Use one jointtrace dump per input file')
            meta=dict(zip(('version','commands','samples','armed','full','profile','speed','acceleration'),v))
        elif kind=='J':
            if len(v)!=4 or not 0<=v[0]<12 or v[3] not in (-1,1) or v[0] in joints:raise ValueError('Invalid joint mapping')
            joints[v[0]]=dict(id=v[1],center=v[2],direction=v[3])
        elif kind=='C':
            if len(v)!=15 or v[0]!=len(commands):raise ValueError('Missing or duplicated command record')
            commands.append(dict(begin=v[1],end=v[2],target=v[3:]))
        elif kind=='S':
            if len(v)!=12:raise ValueError('Invalid sample record')
            samples.append(dict(zip(('begin','end','command','joint','status','raw','speed_raw','load_raw','current_raw','voltage_mv','temperature_c','hardware_error'),v)))
        else:raise ValueError('Unknown jointtrace record')
    if not ended or meta is None or len(joints)!=12:raise ValueError('Incomplete dump: need metadata, 12 mappings and END')
    if len(commands)!=meta['commands'] or len(samples)!=meta['samples']:raise ValueError('Truncated dump: record counts do not match')
    if not commands:raise ValueError('No gait commands recorded; arm before a gait')
    origin=commands[0]['begin']
    def time(t):return (t-origin)&0xffffffff
    last=-1
    for c in commands:
        c['begin']=time(c['begin']);c['end']=time(c['end'])
        if not last<=c['begin']<=c['end']<60000:raise ValueError('Invalid command timing')
        if any(not 0<=p<=65535 for p in c['target']):raise ValueError('Invalid encoded target')
        last=c['end']
    for s in samples:
        s['begin']=time(s['begin']);s['end']=time(s['end'])
        if s['joint'] not in joints or not 0<=s['command']<len(commands):raise ValueError('Sample references missing command/joint')
        c=commands[s['command']]
        if not c['end']<=s['begin']<=s['end']<60000:raise ValueError('Sample predates its command')
        if s['command']+1<len(commands) and s['end']>commands[s['command']+1]['begin']:raise ValueError('Ambiguous command during sample')
    return meta,joints,commands,samples

def analyze(text,start_ms=0,end_ms=None):
    if start_ms<0 or (end_ms is not None and end_ms<=start_ms):raise ValueError("Invalid analysis window")
    meta,joints,commands,samples=parse(text);rows=[];results={}
    times=[c['end'] for c in commands]
    window=[i for i,c in enumerate(commands) if c['end']>=start_ms and (end_ms is None or c['end']<end_ms)]
    if not window:raise ValueError('No commands in analysis window')
    for j,config in joints.items():
        sign=config['direction'];targets=[signed_target(c['target'][j]) for c in commands]
        valid=[];bad=0
        for s in samples:
            if s['joint']!=j or s['end']<start_ms or (end_ms is not None and s['begin']>=end_ms):continue
            if s['status']!=0 or not 0<=s['raw']<4096:bad+=1;continue
            target=targets[s['command']];error=wrapped_error(s['raw'],target)*sign*DEG
            row={**s,'servo_id':config['id'],'joint_name':f"{('FL','FR','RL','RR')[j//3]}-J{j%3+1}",
                 'time_ms':(s['begin']+s['end'])/2,'target_deg':(target-config['center'])*sign*DEG,
                 'actual_deg':(target-config['center'])*sign*DEG+error,'error_deg':error}
            rows.append(row);valid.append(row)
        name=f"{('FL','FR','RL','RR')[j//3]}-J{j%3+1}"
        gaps=[b['time_ms']-a['time_ms'] for a,b in zip(valid,valid[1:])]
        if any(g<=0 for g in gaps):raise ValueError('Duplicate or backwards sample timestamps')
        speeds=[abs(wrapped_error(b['raw'],a['raw'])*DEG/g*1000) for a,b,g in zip(valid,valid[1:],gaps)]
        errors=[r['error_deg'] for r in valid]
        result=dict(samples=len(valid),failed_reads=bad,servo_id=config['id'],
            target_excursion_deg=(max(targets[i] for i in window)-min(targets[i] for i in window))*DEG,
            observed_excursion_deg=max(r['actual_deg'] for r in valid)-min(r['actual_deg'] for r in valid) if len(valid)>=2 else None,
            rms_error_deg=math.sqrt(stats.mean(e*e for e in errors)) if errors else None,
            max_error_deg=max(map(abs,errors),default=None),
            median_sample_gap_ms=stats.median(gaps) if gaps else None,
            max_sample_gap_ms=max(gaps,default=None),
            max_interval_average_speed_deg_s=max(speeds,default=None),
            min_voltage_mv=min((r['voltage_mv'] for r in valid),default=None),
            max_abs_load_raw=max((abs(r['load_raw']) for r in valid),default=None),
            max_abs_current_raw=max((abs(r['current_raw']) for r in valid),default=None),
            hardware_error_samples=sum(bool(r['hardware_error']) for r in valid),
            delay_estimate_ms=None,delay_reason='insufficient excitation or samples')
        # Delay fit uses the dense sent-command history and sparse sensor data.
        # It is an aggregate fit, not a per-step measured latency or true torque.
        fit=[r for r in valid if r['time_ms']>=times[0]+400 and not r['hardware_error']]
        if len(fit)>=12 and result['target_excursion_deg']>=5 and fit[-1]['time_ms']-fit[0]['time_ms']>=2500:
            costs=[]
            for lag in range(0,401,10):
                residual=[wrapped_error(r['raw'],targets[bisect.bisect_right(times,r['time_ms']-lag)-1])*DEG*sign for r in fit]
                bias=stats.mean(residual)
                costs.append((math.sqrt(stats.mean((v-bias)**2 for v in residual)),lag))
            cost,lag=min(costs)
            plausible=[d for c,d in costs if c<=cost+.2]
            uncertainty=max(gaps)/2+max(r['end']-r['begin'] for r in fit)/2
            result.update(delay_candidate_ms=lag,delay_fit_rms_deg=cost,
                delay_fit_range_ms=[min(plausible),max(plausible)],timing_resolution_bound_ms=uncertainty,
                delay_reason='aggregate candidate only; sparse sampling/offset/dynamics can confound delay')
            if 0<lag<400 and max(plausible)-min(plausible)<=40 and uncertainty<=40:
                result['delay_estimate_ms']=lag
                result['delay_reason']='aggregate fit; validate with repeated captures'
        results[name]=result
    return dict(schema=1,metadata=meta,joints=results,source='jointtrace-input',analysis_window_ms=[start_ms,end_ms],
                torque_units='load/current raw; not calibrated Nm or amperes',
                limitation='No contact sensor; branch reconstructed near commanded canonical target; interval speed is not peak speed'),rows

def write_report(path:Path,output:Path,compare:Path|None=None,start_ms=0,end_ms=None):
    data,rows=analyze(path.read_text(),start_ms,end_ms);data['input']=str(path.resolve())
    other=None
    if compare:other,_=analyze(compare.read_text(),start_ms,end_ms);data['comparison']={'input':str(compare.resolve()),'result':other}
    output.mkdir(parents=True,exist_ok=True)
    _,mapping,commands,_=parse(path.read_text())
    with (output/'commands.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['send_begin_ms','send_end_ms',*[f'{leg}_J{j}_deg' for leg in ('FL','FR','RL','RR') for j in (1,2,3)]])
        for c in commands:w.writerow([c['begin'],c['end'],*[(signed_target(c['target'][j])-mapping[j]['center'])*mapping[j]['direction']*DEG for j in range(12)]])
    (output/'joint-capability.json').write_text(json.dumps(data,indent=2,ensure_ascii=False))
    if rows:
        with (output/'samples.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(sorted(rows,key=lambda r:r['time_ms']))
    def fmt(value):return '측정 부족' if value is None else f'{value:.2f}'
    lines=['# 관절 실측 진단','',f'입력: `{path}`','',
        '목표·실측은 각 서보 조회 시점에 연결했습니다. 관측 속도는 샘플 사이 평균이며 모터 최대 속도가 아닙니다.',
        '부하/전류는 서보 원시값이며 Nm/A로 환산하지 않습니다. 발 접촉과 토크 부족을 이 자료만으로 확정하지 않습니다.','',
        '| 관절 | 샘플 | 목표 진폭 ° | 관측 진폭 ° | RMS 오차 ° | 최대 오차 ° | 조회 간격 ms | 최저 전압 V |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for k,v in data['joints'].items():
        lines.append(f"|{k}|{v['samples']}|{fmt(v['target_excursion_deg'])}|{fmt(v['observed_excursion_deg'])}|{fmt(v['rms_error_deg'])}|{fmt(v['max_error_deg'])}|{fmt(v['median_sample_gap_ms'])}|{fmt(v['min_voltage_mv']/1000 if v['min_voltage_mv'] is not None else None)}|")
    lines+=['','## 지연 판정','', '12축 순차 조회에서는 축당 약240ms 간격입니다. 10~20ms 지연을 실측 확정값으로 보고하지 않습니다. JSON의 delay_candidate_ms는 탐색 후보이고 delay_estimate_ms가 null이면 식별 미완료입니다.']
    if other:
        lines+=['','## 비교 입력 대비 RMS 오차 차이','',f'비교 입력: `{compare}`. 양수면 첫 번째 입력의 오차가 더 큽니다. 정책·전압·바닥·조작 크기를 맞춰 해석해야 합니다.','', '| 관절 | 첫 입력 − 비교 입력 ° |','|---|---:|']
        for k,v in data['joints'].items():
            a=v['rms_error_deg'];b=other['joints'][k]['rms_error_deg'];lines.append(f'|{k}|{fmt(a-b if a is not None and b is not None else None)}|')
    lines+=['','이 보고서는 실측한 조건에서의 추종 특성입니다. 정격 토크, 최대 부하, 모든 자세의 동작 가능성을 인증하지 않습니다.']
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    return output/'report.md'


def download(console,path:Path):
    """Read bounded pages, verify each response, then atomically publish a dump.

    Does not start motion or clear the RAM trace. First page freezes recording.
    No retries are silently combined; incomplete pages remain diagnostic files.
    """
    if path.exists():raise ValueError('Use a new trace output path')
    path.parent.mkdir(parents=True,exist_ok=True)
    partial=path.with_suffix(path.suffix+'.partial')
    offset=0;expected_total=None;lines=[]
    while True:
        response=console.send(f'jointtrace dump {offset} 12',timeout=20)
        if not response.ok:raise ValueError(response.text)
        page=[line for line in response.lines if line.startswith('$JT,')]
        markers=[line for line in page if line.startswith('$JT,P,')]
        if len(markers)!=1:raise ValueError('Paged transfer requires V47 jointtracepage firmware')
        first,count,total=map(int,markers[0].split(',')[2:])
        data=[line for line in page if line.startswith(('$JT,C,','$JT,S,'))]
        if first!=offset or len(data)!=count or not 0<=count<=12 or not first+count<=total<=512:
            partial.write_text('\n'.join(lines+page)+'\n');raise ValueError('Incomplete or misnumbered trace page')
        if expected_total is not None and total!=expected_total:raise ValueError('Capture changed during transfer')
        expected_total=total;lines.extend(page);partial.write_text('\n'.join(lines)+'\n')
        offset+=count
        if offset==total:
            if '$JT,END' not in page:raise ValueError('Missing final trace marker')
            parse('\n'.join(lines))
            partial.replace(path);return path
        if count==0:raise ValueError('Trace transfer made no progress')
