# S 출발 · 대각선 동기 V1 (2026-09-14)

시뮬레이터 전용 절차식 보행 정책 `s_native_v1`. 이전 비교 모델로 유지한다. 현재 기본 선택은 S 연속 교대 V2다. 학습된 신경망 모델은 아니다. S를 기준으로 출발하며 RL·FR / RR·FL 대각선 쌍이 반 주기 간격으로 교대한다. 발끝 쿠션은 D37.3 × 27mm, 총질량은 2.754kg이다.

## 발 교대 시 멈칫거림 수정

이전 코드의 한 주기 2초 중 양쪽 교대마다 0.6초를 정지하는 설정이 원인이었다. 발을 옮기는 시간 0.4초는 유지하고, 교대 대기 0.3초 / 전체 주기 1.4초로 줄였다(최대 입력 기준). 보폭 40mm와 명목 발 들기 25mm를 유지했다. 각 발의 실제 지지 비율도 0.7142857로 맞췄다. 입력 60%에서는 공통 주기 배율 1.14가 적용되어 목표 대기 시간이 0.684초 → 0.342초다.

두 설정을 각각 32초 동안 자유 몸체·중력·접촉·기존 서보 응답이 포함된 동일 모델에서 재생했다. 전진 60%, 분석 구간 6~28초, 측정 간격 20ms다.

| 실제 물리 측정 | 변경 전 | 변경 후 |
|---|---:|---:|
| 네 발 동시 접지 구간 중앙값 | 680ms | 360ms |
| 네 발 동시 접지 구간 최대값 | 720ms | 380ms |
| 대각선 주 들림 시점 차이 최대 | 20ms | 20ms |
| 대각선 주 착지 시점 차이 최대 | 60ms | 60ms |
| 최대 몸체 기울기 | 3.17° | 2.89° |
| 평균 X 이동 속도 | 0.0206m/s | 0.0216m/s |

동시 접지는 각 발의 실제 쿠션 최저점 높이가 모두 1mm 이하인 구간이다. 주 들림 구간은 각 교대에서 높이 적분이 가장 큰 연속 공중 구간으로 정한다. 작은 추가 튀어오름도 JSON의 `extra_airborne`에 그대로 기록한다. 초기·마지막 불완전 주기는 대각선 시점 비교에서 제외한다. `missed`는 한쪽 주 들림이 없거나 최고 높이가 5mm 미만인 교대를 뜻한다.

입력 30% / 60% / 100% 각각 32초 추가 시험에서 주 들림 누락은 없었다. 최대 착지 차이는 각각 120 / 60 / 100ms로, 모든 입력에서 완전히 같은 접지 시점을 달성한 것은 아니다. 더 짧은 1.2초 주기는 일부 시험에서 낮은 발 들림과 동기 악화가 있어 채택하지 않았다. 이 수정은 교대 대기를 줄였으며 전체 보행 품질이나 실기 동작을 검증 완료한 것은 아니다. 47개 관련 회귀 검사가 통과했다. 기존 NumPy 행렬곱 경고가 있으며, 이 시험의 유한값·안전 상태·정지 완료 검사는 통과했다.

- 변경 전/후 물리 기록: `transition-before.json`, `transition-after.json` 및 같은 이름의 CSV
- 4초 측면 재생: `transition-before.gif`, `transition-after.gif`
- 입력별 장시간 기록: `acceptance-transition-{300,600,1000}.csv`와 `transition-contact-analysis.json`
- 제약 감사: `constraints-audit.md` (초기 모델 기준)
- 기존 `validation.json`과 `trajectory.csv`는 최초 1.8초 모델의 실패 기록이며 현재 결과가 아니다.

재현:

```bash
PYTHONPATH=tools/servo_tool:simulation/mujoco python simulation/mujoco/scripts/validation/validate_s_native.py --profile s_native_v1 --command 600 --period 2 --hold .6 --gif --output artifacts/s-native-v1/transition-before
PYTHONPATH=tools/servo_tool:simulation/mujoco python simulation/mujoco/scripts/validation/validate_s_native.py --profile s_native_v1 --command 600 --gif --output artifacts/s-native-v1/transition-after
```

## S 좌표와 기본 화면

S의 쿠션 접지점 X를 J2/R2 축 X에 맞춘다. 보행 중 X/Y는 S에서 정한 동일한 발바닥 재질점을 추적하고, Z는 실제 최저 표면으로 계산하여 메시 꼭짓점 교체에 따른 수평 목표의 점프를 피한다. 몸체 지지 이동은 CAD 기반 예상 동역학으로 계산한다. 기존 두 링크 자세 보정은 새 CAD IK 결과에 중복 적용하지 않는다. IMU 기울기 감시와 방향 보정은 유지한다.

기본 화면은 앞쪽이 오른쪽을 향하는 측면(방위 90°, 고도 0°, 거리 1.05m)이며 무게중심 아래 0.1m를 따라간다. 사용자 지정 모니터는 LG FULL HD다.

## 사진의 투영각
`reference-projected-angles.png`에 수동 기준점을 표시했다. 사용자 RL/RR 지정을 따른다. 화면 오른쪽 수평에서 시계방향으로 몸체 축 약12.5도, RL 무릎→발끝 연결선102.1도, RR 연결선156.1도. 몸체 축 대비 약89.6/143.6도. 굽은 링크의 접선이나 실제 관절각이 아니며 가림/기준점 오차 약5~10도인 화면상 근사다. 한 장으로 카메라의 실제 방위/고도, 3D 관절각, 보폭 길이를 확정할 수 없다. 사진은 대각선 극점 교대 조건의 참고로만 사용한다.
