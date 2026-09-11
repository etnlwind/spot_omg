# 지지·스윙 작업 분리 구현 및 실패 기록 — 2026-09-11

## 상태

코드 구현은 진행했지만 물리 품질 개선은 검증하지 못했다. 기존 앱·라이브 기본 정책과 실물 펌웨어는 변경하지 않았다. 초기 고정 J1 시험 기록이다. 이후 사용자가 J1 해제를 승인하여 추가 시험을 수행했다. 최종 상태와 추가 결과는 DIAGONAL-SPLIT-J1-2026-09-11.md를 참고한다.

## 코드 변경

`support_shift.py`의 `separate_support_swing` 선택 분기:

- 지지 발은 몸체 높이·자세 보정 작업을 수행한다.
- 스윙 발은 지면 기준 높이를 목표로 한다. 지지 높이 보정을 스윙 발에 그대로 더하지 않는다.
- J1 고정 상태에서 불가능한 독립 XYZ 작업을 강제하지 않고, 몸체 X와 지면 Z의 두 Jacobian 행으로 J2/J3를 계산한다.
- 스윙 구간 경계는 기존 smooth 가중치로 이어지며 네 발 대기 구간을 추가하지 않았다.
- 하중 보상은 스윙 가중치에 따라 감소한다.
- 관절 보정 한도 ±6도, 20ms당 ±0.5도, 기존 실제 관절·토크·안전정지 한도 유지. 기존 정책 분기의 한도는 그대로다.
- 선택적인 `support_foreaft_shift`는 최대 10mm 앞뒤 몸체 기준점 이동을 시험한다. 기존 60mm 명목 보폭과 별개의 기준점 이동이며 실제 발끝 상대 이동량이 달라질 수 있다.
- 선택적인 `predict_swing_attitude`는 이미 도착한 지연 센서 표본과 각속도로 40ms를 예측하며 ±0.15rad로 제한한다. 시뮬레이터 자세 정답을 사용하지 않는다.
- IK 해 자체의 잔여 오차와 한도·하중 보상을 적용한 최종 명령의 잔여 오차를 별도로 기록한다.

## 같은 20.02초 자유 몸체 시험

5초 이후 평가. 모든 수치는 추정 물리 모델의 결과다.

| 후보 | 최초 안전정지 s | 최대 roll ° | 높이 최대−최소 mm |
|---|---:|---:|---:|
| diagonal-preload-1 | None | 4.06 | 7.56 |
| diagonal-separated | 12.46 | 17.20 | 32.05 |
| diagonal-separated-support-1 | None | 11.05 | 39.26 |
| diagonal-separated-support-0.4 | None | 7.77 | 26.71 |
| diagonal-separated-transfer-0 | None | 5.87 | 8.69 |
| diagonal-separated-transfer-0.4 | None | 8.40 | 19.15 |
| diagonal-separated-predict-1 | 17.28 | 13.25 | 36.20 |
| diagonal-separated-predict-0.4 | None | 10.01 | 31.41 |
| diagonal-separated-noheight-1 | None | 6.72 | 13.82 |
| diagonal-separated-noheight-0.4 | None | 5.82 | 13.16 |

대표 `diagonal-separated-noheight-1` 후보의 FL–RR 중간 스윙 높이 차이 RMS는 16.86mm, 접촉 불일치 비율은 45.69%로 기존 10.55mm/25%보다 나쁘다. 수정된 발 궤적을 실제로 따라가더라도 몸체를 두 지지 발 연결선 부근에서 안정화하는 문제가 남는다. J1을 반드시 풀어야 한다는 수학적 불가능성 증명은 아니지만 현재 고정 J1 해법으로 해결했다고 말할 수 없다.

## 검증과 배포

호스트 회귀는 지지·스윙 작업, J1 고정, 보정 한도, 센서 누락 초기화 등을 검사했다. 호스트 성공과 물리 보행 성공은 구분한다. 실패 후보를 앱 기본 정책으로 배포하지 않았다. 60초 합격 검증이나 실기 설정 읽기/위치 유지/전체 보행 시험을 수행했다고 주장하지 않는다.

비교 영상용 설정은 `artifacts/upright/2026-09-11/cushion/separated-review-profiles.json`, 결과 영상은 `separated-review-comparison.mp4`다. 위는 이전, 아래는 실패한 작업 분리 후보이며 영상에도 FAILED로 표시한다.
