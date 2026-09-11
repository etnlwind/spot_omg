# 원형 쿠션: 어깨 앞 착지 전진 정책

프로필 `cushion_reach`, `simulation/mujoco/upright_profiles.json`에 저장.
기존 upright/upright_reach와 실기 정책은 변경하지 않았다.
쿠션: 최대 지름37.3mm, 길이27mm, 삼각형 구멍 바닥과 평행. 물성은 추정이다.
정책: 주기1.8초, 지지비0.60, 보폭50mm, 목표 들림30mm, 다리 수직 길이240mm,
전후 중심−10mm, 벌림0.75°. IMU 수평/직진 보정은 기존 level 기반이다.

## 검증

24초 후보12개 비교 후, 앞발 중심이 J1보다 조금 앞에 착지하면서 기울기가 작은
50mm 보폭/−10mm 중심 조합을 선택했다. 쿠션 앞쪽 가장자리 대신 중심을 측정했다.
60초 직진: safety=ok, 평면 변위 기준0.046m/s, 최대 roll/pitch 약2.0°.
접촉 시작 시점 중심의 중앙값: FL +5.5mm, FR +5.6mm, RL +10.6mm, RR +11.0mm.
접촉 재발생도 포함한 통계이므로 모든 스텝이 이 위치에서 착지한다는 보장은 아니다.
중간 스윙 접촉은 앞발 약9%, 뒷발7~8% 남는다.

24초 후진/좌우 회전은 안전 정지 없었지만, 후진 앞발 중간 스윙 접촉66~67%,
회전 시 일부 발 약36%가 남아 전방 직진에 한정한 실험 정책으로 취급한다.
무게중심 이동+40ms명령 지연30초 조건은 safety=ok,최대3.5°지만 FL 착지는−4.7mm로
후퇴했다. 추정 질량/지면이 달라져도 전방 착지를 보장하는 정책은 아니다.
호스트 기하/기존 정책 보존 검사8개 통과. 실기 업로드/동작 검증은 수행하지 않았다.

결과: `artifacts/upright/2026-09-11/cushion/reach373-validation.json`
후보: 같은 폴더의 `reach373-search.json`, `reach373-fine.json`
영상: 같은 폴더의 `cushion-reach.mp4` (16초, 종료 시 정지 포함).
재검증: `python simulation/mujoco/validate_cushion_reach.py`

## 앱 연결 가능한 실행

```bash
cd /Users/etnlwind/project/spot_omg
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py \
  --viewer --host 0.0.0.0 --port 8765 --video-host 0.0.0.0 --video-port 8766 \
  --experimental-profiles simulation/mujoco/upright_profiles.json \
  --profile cushion_reach --foot-cushion simulation/mujoco/foot_cushion_10mm.json
```

Bluetooth도 기본 활성화된다. 앱의 기존 정책 버튼을 누르면 선택이 바뀔 수 있다.
가상 로봇 터미널에서 `gaitprofile cushion_reach`로 다시 선택할 수 있다.

## J2 들림 강화 후보: cushion_j2lift

기존 cushion_reach를 보존하고 `cushion_j2lift`를 추가했다.
목표 들림30→40mm, 주기1.8→2.2초로 변경. 지지 구간 목표 각도는 그대로다.
명목 J2 최대 목표는 약50→53°로 증가하고 J3도 IK로 함께 조정된다.
J2 단독 오프셋을 넣어 발 경로를 왜곡하지 않고 높은 발 경로를 관절각으로 변환한다.

60초 직진: safety=ok,0.035m/s,최대기울기3.3°,앞발 착지 중심 중앙값+5.0/+5.1mm.
24초 후보에서 앞발 최대 여유37mm,뒷발23mm.60초에서 앞발 중간 스윙 접촉은
거의0%,뒷발약6%.기존보다 느리지만 발 들림은 더 분명하다.
후진/회전은 safety=ok여도 접촉 끌림이 남는다. 무게중심+지연 조건은7.4°까지
기울고 앞발 착지가 뒤로 이동하므로 전진용 실험이라는 제약은 유지한다.
기존 정책 보존 검사 및 스윙 J2 증가/지지 구간 목표 유지 검사 통과.
실제 로봇은 변경하지 않았다.
결과 `artifacts/upright/2026-09-11/cushion/j2lift-validation.json`,
영상 같은 폴더 `cushion-j2lift.mp4`.
실행 명령에서 `--profile cushion_j2lift`를 사용한다.

## 큰 전진 스텝 실험: cushion_forward

사용자가5mm 추가는 작다고 정정하여 큰 차이를 보이는 별도 실험으로 변경했다.
보폭140mm(기존50mm),주기4.8초,지지비0.70,들림40mm,높이240mm,중심−10mm.
이전60mm 후보는 `cushion_forward_small`로 보존했다.
60초 전진에서 앞발 착지 중심 중앙값 FL+42.5mm/FR+40.1mm로 기존약+5mm보다
약35~38mm 더 앞에 착지했다. 뒷발은+26.7/+25.4mm다.
전진속도약0.035m/s,최대기울기8.5°,safety=ok.
뒷발 중간 스윙 접촉53~55%가 남아 보행 품질 검증에 실패했다.
후진/회전에도 큰 끌림이 있어 안정화된 정책으로 취급하지 않는다.
앱 이름은 **큰 전진 스텝 · 실험 (검증실패)**. 가상 로봇에서만 선택 가능하다.
물리 모델/서보 제한/안전 정지 조건은 유지했다. 실기 적용은 하지 않았다.

- 후보: `artifacts/upright/2026-09-11/cushion/big-step-search.json`
- 검증: 같은 폴더 `forward-reach-validation.json` (`quality_passed:false`)
- 영상: 같은 폴더 `cushion-bigstep.mp4` (20초)
- 재실행: 기존 앱 연결용 명령에서 `--profile cushion_forward` 사용.
