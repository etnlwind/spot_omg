# 대각선 연속 교대와 몸체 높이 하중 보상 — 2026-09-11

## 목표와 적용 범위

전진하면서 몸체 높이·roll·pitch를 유지하는 것이 목표다. J1 목표는 고정하고 J2/J3를 CAD 발끝 역기구학으로 계산한다. 실물 펌웨어는 변경하지 않았다. 신규 실험을 앱/라이브 기본 정책으로 채택하지 않았다. 전체 품질 기준 미달이다.

## 이번 변경

- 지지율 0.5, 주기 1.44초: FL+RR / FR+RL을 0.72초마다 교대한다. 계획상 네 발 대기는 없다. 실제 접촉은 센서 지연·추종·쿠션 때문에 계획과 다를 수 있다.
- 이번 실험은 보폭 **60mm**, 명목 높이 매개변수 201.75mm, 스윙 높이 20mm다. 기존 140mm 보폭 요구를 통과시킨 것이 아니다. 기존 V2의 240mm 높이 매개변수보다 낮다.
- `load_preload_gain`은 기본 0인 선택 기능이다. CAD 질량과 중력, 지연된 관절 피드백, 예정 지지 쌍으로 하중 토크를 추정한다.
- 정적 토크는 `bias - J.T @ force`, 위치 보상은 `torque / estimated_servo_kp`다. stiffness 35 Nm/rad는 현재 서보 모델의 **추정치**다. 실제 서보 식별값이 아니다.
- 위치 보상은 ±4도, 20ms당 ±0.25도로 제한하고 J1 보상은 0이다. 센서 실패/누락이나 제어 해제 시 초기화한다. 실제 관절/토크/속도 제한과 안전정지는 변경하지 않는다.
- 힘 계산은 쿠션 최저 꼭짓점의 실제 3D Jacobian을 사용한다. IK용 XY 중심/Z 최저점 혼합 Jacobian을 힘 계산에 쓰지 않는다.
- 기본은 균등 지지하중, `load_share=com_projection`은 무게중심의 지지선 투영으로 두 하중을 나눈다. 전체 몸체 힘·모멘트를 만족하는 최적화가 아니며, 미해결 root wrench를 진단에 남긴다. MPC/WBIC 구현이 아니다.
- 시뮬레이터 실제 접촉력/몸체 높이는 평가와 영상에만 사용한다. 몸체를 고정하거나 매 틱 qpos를 덮어쓰지 않는다.

## 같은 20초 실행의 5~20초 비교

| 후보 | 높이 최대−최소 mm | 높이 표준편차 mm | 최대 roll ° | 최대 pitch ° |
|---|---:|---:|---:|---:|
| diagonal-lift20-lead40 | 15.34 | 4.87 | 2.25 | 1.29 |
| diagonal-preload-1 | 7.56 | 1.54 | 4.06 | 2.17 |
| diagonal-load-attitude | 5.77 | 1.20 | 4.66 | 2.04 |

하중 보상은 높이 변동을 줄였으나 roll을 악화시켰다. 높이 개선만으로 전체 안정화 성공이라 판정하지 않는다.

## 60초 평가

`diagonal-preload-final-60s.json`: 65.02초 실행, 5~65초 평가. 안전정지 없이 진행. 높이 변동 8.43mm, 표준편차 1.52mm, 목표 높이 RMS 오차 3.90mm. 최대 roll 5.36°, pitch 2.74°, RMS roll 1.34°, pitch 0.63°. 오른앞/오른뒤 중간 스윙 접촉률 19.96%/27.38%로 기준 미달. 앞발 착지 중심도 어깨보다 30mm 앞이라는 기존 기준 미달이다. 평균 목표 높이보다 약 3.59mm 낮다. 짐벌 같은 안정성 달성으로 표현하지 않는다.

## 검증 수준

호스트: 중력 토크 부호를 위치별 중력 위치에너지 유한차분과 비교, 고정 J1·보상 한도·센서 누락 초기화·두 발 교대 일정을 검사한다. 실기 설정 readback, 실제 위치 유지 시험, 실기 전체 동작 시험은 수행하지 않았다.

하중 보상 후보에서 14초에 Stop을 보내고 18초에 motion 없음/safety OK를 확인했다. 추가 하중·마찰·센서 지연 변형 조건은 이번 후보에서 전체 재검증하지 않았다.

## 영상

`artifacts/upright/2026-09-11/cushion/diagonal-preload-comparison.mp4`: 실제 자유 몸체 물리 시뮬레이션 20초, 25fps. 위는 보상 전, 아래는 균등 하중 보상 후. 좌측 측면/우측 정면. 카메라 높이를 고정하고 몸체 높이/각도/발 여유/접촉을 표시한다. 보폭 표시는 실제 프로필 값을 사용하도록 수정했다. 추가 기울기 보정 후보의 영상은 아니다.

재생 비교 명령:

```sh
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/preview_support_shift.py --profiles artifacts/upright/2026-09-11/cushion/diagonal-preload-profiles.json --baseline diagonal-lift20-lead40 --candidate diagonal-preload-1 --prefix diagonal-preload --height-overlay
```

추가 기울기·각속도 보정 60초 결과 (`diagonal-load-attitude-final-60s.json`): safety=ok, 높이 변동 8.31mm, 표준편차 1.30mm, 목표 높이 RMS 오차 3.76mm, 최대 roll/pitch 4.66°/2.04°. 최대 roll 3° 기준 미달이다. 호스트 회귀 총 22개 통과.
