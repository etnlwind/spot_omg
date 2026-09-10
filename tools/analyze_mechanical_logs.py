"""Analyze exported STM32 MECH records. Never connect to or configure a robot."""
import argparse
import re
from pathlib import Path
from statistics import mean

LEGS=('FL','FR','RL','RR')


def parse(text):
    groups=[];active=None;seen=set()
    for line in text.splitlines():
        identity=re.search(r'\$SPOTLOG seq=(\d+) boot=(\d+)',line)
        if identity:
            key=identity.groups()
            if key in seen:continue
            seen.add(key)
        match=re.search(r'\b(MECH_[A-Z]+)\s+(.*)',line)
        if not match:continue
        kind,body=match.groups()
        values=dict(re.findall(r'([a-zA-Z0-9_]+)=([^\s]+)',body))
        if kind=='MECH_BEGIN':
            if active:groups.append(active)
            active=dict(meta=values,motors={},complete=False)
        elif active and kind=='MECH_END':
            active['complete']=values.get('stored')=='1' and values.get('missing','0')=='0'
            groups.append(active);active=None
        elif active and 'id' in values:
            active['motors'].setdefault(values['id'],{})[kind]=values
    if active:groups.append(active)
    return groups


def report(groups, fixture_labels=()):
    lines=['# 모터 피드백 / Stand11 기구 검증','',
           '모터 목표값 일치는 실제 링크의 일직선을 증명하지 않습니다. 지그 또는 외부 각도 측정이 기준입니다.',
           '각도는 저장된 중심값·회전 방향으로 환산합니다. 전류와 부하는 원시값이며 토크/전류의 물리 단위로 해석하지 않습니다.','']
    reference={}
    for number,g in enumerate(groups,1):
        label=g['meta'].get('label',g['meta'].get('mode','?'))
        lines += [f'## 기록 {number}: {label}', '',f"완전한 기록: {'예' if g['complete'] else '아니요 — 중심값 제안에서 제외'}",'']
        if g['meta'].get('mode')=='gait':
            lines += ['| ID | 샘플 | 평균 부호 오차(°) | 지지 평균(°) | 스윙 평균(°) | 최저 전압(mV) |','|---|---:|---:|---:|---:|---:|']
            for sid,m in sorted(g['motors'].items(),key=lambda x:int(x[0])):
                t=m.get('MECH_TRACK');p=m.get('MECH_PHASE',{})
                if not t:lines.append(f'| {sid} | 미측정 | — | — | — | — |');continue
                def phase_mean(key,count):return f"{int(p[key])/10:.1f}" if int(p.get(count,0)) else '—'
                lines.append(f"| {sid} | {t['n']} | {int(t['mean10'])/10:.1f} | {phase_mean('se10','stance_n')} | {phase_mean('we10','swing_n')} | {t['mv']} |")
            lines += ['', '지지/스윙은 보행 명령상의 구간이며 실제 접지 센서 측정은 아닙니다.',''];continue
        lines += ['| 다리/관절 | ID | 위치 tick | 중심 tick | 추정각(°) | 목표-실제(°) | 전압(mV) | 토크 |','|---|---:|---:|---:|---:|---:|---:|---|']
        for sid,m in sorted(g['motors'].items(),key=lambda x:int(x[0])):
            p=m.get('MECH_POS');c=m.get('MECH_CAL');goal=m.get('MECH_GOAL');power=m.get('MECH_PWR')
            if not all((p,c,goal,power)):
                lines.append(f'| 읽기 누락 | {sid} | — | — | — | — | — | — |');continue
            position,center,direction=int(p['pos']),int(c['ctr']),int(c['dir'])
            angle=(position-center)*direction*360/4096
            error=(int(goal['goal'])-position)*direction*360/4096
            lines.append(f"| {LEGS[int(p['leg'])]} J{p['j']} | {sid} | {position} | {center} | {angle:.3f} | {error:.3f} | {power['mv']} | {goal['torque']} |")
            if label in fixture_labels and g['complete']:
                healthy=(goal['torque']=='0' and p['moving']=='0' and power['hw']=='0'
                         and int(power['mv'])>=10800 and int(c['min'])<=position<=int(c['max'])<=4095)
                if healthy:reference.setdefault(sid,[]).append((position,center,direction,label))
        lines += ['']
    lines += ['## 외부 지그 기준 중심값 검토','',
              'fixture-label은 사용자가 모든 관절의 실제 0° 정렬을 외부에서 검증했다는 선언입니다. Stand11이라는 명령 이름만으로 지그 기준으로 취급하지 않습니다.',
              '토크 OFF·정지·하드웨어 오류 없음·10.8V 이상·완전한 기록을 사용합니다. 최소 두 번 측정하고 양쪽 접근 방향을 비교하십시오. 아래 값은 검토용이며 자동 적용하지 않습니다.','',
              '| ID | 횟수 | 기존 중심 | 측정 평균 중심 | 변경 tick | 방향 환산 변화(°) | 반복 범위(°) |','|---|---:|---:|---:|---:|---:|---:|']
    for sid,rows in sorted(reference.items(),key=lambda x:int(x[0])):
        if len(rows)<2 or len({(r[1],r[2]) for r in rows})!=1:continue
        center=rows[0][1];direction=rows[0][2];candidate=round(mean(r[0] for r in rows));spread=(max(r[0] for r in rows)-min(r[0] for r in rows))*360/4096
        lines.append(f'| {sid} | {len(rows)} | {center} | {candidate} | {candidate-center:+d} | {(candidate-center)*direction*360/4096:+.3f} | {spread:.3f} |')
    lines += ['', '반복 범위가 크면 유격·지그 정렬·마찰을 먼저 점검하십시오. 평균값 변경만으로 유격이나 링크 휨을 해결할 수 없습니다.','']
    return '\n'.join(lines)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture-label',action='append',default=[])
    args=parser.parse_args()
    groups=parse(args.log.read_text())
    if not groups:parser.error('No MECH log groups found')
    args.output.write_text(report(groups,args.fixture_label))
    print(args.output)
