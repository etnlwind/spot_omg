# 추진 후 J2 / J3 회수 — 시험 녹화

2026-09-16. 사용자 요청 순서(추진 완료 → J2 추가 후굴로 이지 → J3 굴곡과 전방
스윙 → 이동 중 펴서 착지)를 검토하기 위한 **MuJoCo 후보 영상**이다.
기존 V6.2.6 및 앱 기본값과 실기 펌웨어는 변경하지 않았다.

## 파일

- `SpotOMG_J2_J3_4views_10s.mp4`: 2초 정지 + 8초 전진, 25fps, 1280×1060.
- `SpotOMG_J2_J3_4views_half_speed.mp4`: 같은 상태를 0.5배속으로 재생, 총20초.
  화면의 시간 수치는 원래 시뮬레이션 시간이다.
- `capture/four-views.mp4`: 정지·S 복귀 관찰 4초를 추가한 전체14초.
- `capture/front.mp4`, `rear.mp4`, `top.mp4`, `side.mp4`: 각 시점의 전체14초.
- `capture/telemetry.csv`, `trajectory.json`: 20ms 간격 수치.
- `capture/summary.json`: 시뮬레이션 조건과 요약.
- `verification.json`: 재생·디코딩 확인, 원본 해시, 지지 궤적 보존 및 재현 검사.

네 화면은 **동일한 한 번의 물리 시뮬레이션**을 렌더링했다.
J2/J3 목표·실제 CAD 관절각, 발 하중과 바닥 여유, S 대비 발 X,
몸체 기울기, 위상, 추종 속도 비율·오차, 전압과 보호 상태가 포함된다.
J3 CAD 각도는 링크 사이의 기하학적 무릎 안쪽 각도와 다르다.

## 후보와 검증 범위

설정은 `config/experiments/post_push_j2_j3_recovery.json`에 있다.
지지 구간 끝까지 기존 +20/−125mm 밀기 궤적을 유지한 뒤,
회수 구간에서 J2 추가6° 펄스를 먼저 시작하고 J3 추가10° 펄스를 뒤이어 시작한다.
파형은 정지 구간 없이 이어지며 발 이동 중 추가 굴곡이 풀린다.

**위상에 따른 목표 순서이며 실제 추진력 소진/접촉 해제 감지에 의한 전환은 아니다.**
영상의 `planned` 표시는 목표 단계이고, 접촉력·발 높이 및 실제 관절각은 물리 상태다.
첫걸음의 대각선 J1 차이에도 상대 X/Z가 같도록 쌍별 변위를 재투영한다.

추정 물리 조건: 질량2.754kg, 쿠션D37.3×27mm, 전압 설정11.1V,
실기 관측 기반 가속도 제한[50,254,50]×4와 속도3400 적용.
충돌·추종·기울기 보호를 유지했다. 실제 로봇 검증이나 설치는 하지 않았다.

- 8초 보행 및 Stop→S 복귀 시 보호 중단 없음.
- 보행 중 최대 roll/pitch 절댓값8.28°.
- 영상12.6초에 S 복귀 완료, 마지막 최대 실제 관절 오차0.99°.
- 일부 회수 구간 접촉/끌림은 남아 있다. 해결 완료 또는 장시간 안정성 통과가 아니다.
- 401개 목표 위상 비교에서 기존 지지/끝점과 차이 최대0.00572°,
  발 위치0.00960mm로 IK 수렴 오차 범위였다.
- 영상의 J2/J3 실제 각도는 별도 사전 시험과 700개 제어 샘플에서 일치했다.

## 재현

저장소 루트에서 다음을 실행한다. `ffmpeg`가 PATH에 없다면 `--ffmpeg`로 경로를 준다.

```powershell
python -X utf8 simulation/mujoco/scripts/analysis/capture_s_native_balance.py --profile s_native_v6_2_6 --command 1000 --walk-seconds 8 --settle-seconds 4 --tuck-config config/experiments/post_push_j2_j3_recovery.json --output artifacts/s-native-v6-2-7/post-push-video/capture
```
