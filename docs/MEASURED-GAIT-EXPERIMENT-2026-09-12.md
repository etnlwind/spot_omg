# 실측 비교 기반 보행정책 실험 — 2026-09-12

## 결과와 적용 범위

새 시뮬레이터 전용 정책 `measured_lift` — **실측 기반 · 선행 발 들기 · 실험 (검증실패)**를 추가했다. 기존 Cruise와 실물 펌웨어는 이 실험으로 변경하지 않았다. 전진·회전의 발 끌림은 줄었지만, 수평 유지와 반복 스윙 여유 기준에 미달했다. 전진속도·몸체 흔들림의 손익도 함께 보고한다.

## 실측 데이터와 모델 보정의 구분

V47의 전진 252명령/252표본, 좌·우회전 각각 241명령/241표본을 사용했다. 각 관절은 약 240ms 간격이므로 최대 속도·정확한 지연·모터 내부 제어 게인을 식별할 수 없다. 부하/전류의 원시값을 토크 Nm로 간주하지 않았다.

실제 송신한 signed 목표를 기존 검증된 중심/방향으로 복원하고, MuJoCo에서 재생하여 실제 응답 시각에 비교했다. 앞 1초는 초기 몸체 상태가 미측정이므로 비교 점수에서 제외했다. 물리 적분 1ms, 명령 재생 20ms 격자로 비교했고, 0.5ms 적분의 긴 보행 검증을 별도로 수행했다.

11개 유효 PD 응답 후보를 전진 기록으로 비교하고 상위 후보를 좌회전으로 검증했다. 최상 전진 RMS는 2.539°→2.335°였으나 좌회전은 1.719°→1.726°로 악화되어 채택하지 않았다. 원래 모델과 동일한 오른쪽 회전 보류 데이터 RMS는 1.601°다. **개별 모터의 물성 보정을 완료했다는 의미가 아니며, 원래 추정 PD/20ms 지연/토크 한계를 유지했다.** 근접 응답 후보는 민감도 시험에만 사용했다. 무게·마찰·쿠션 물성도 추정값이다.

## 정책

- 전진: 주기 1.35초, 지지 비율 0.52, 보폭 80mm, 계획 들림 24mm, 기존 IK 높이 파라미터 220mm. 이 높이는 바닥부터 몸체까지의 실측 높이가 아니다.
- 스윙: 앞 40%에 매끄럽게 들어 올리고, 중간 30% 유지, 마지막 30%에 내딛는다. 지지 중 직선 궤적과 수평 이동 궤적은 보존하며 J2·J3를 발 위치에서 계산한다.
- 회전: 주기 1.05초, 지지 비율 0.52, 보폭 70mm, 계획 들림 22mm, 높이 파라미터 201.75mm. 회전에는 기존 아치 궤적을 사용한다. 입력 크기나 yaw 명령을 몰래 줄이지 않는다.
- 전진 입력에 따라 두 파라미터 집합과 들림 형태를 연속적으로 섞는다. J1은 기존 회전 기구학과 IMU 보정에 사용하고, 임의의 추가 관절각/토크는 넣지 않았다.
- 27mm 길이·37.3mm 원형 바닥의 기존 쿠션, 관절 제한, 서보 양자화, 안전 정지 조건을 보존했다.
- 계획 단계에서 MuJoCo의 정답 접촉력은 사용하지 않는다. 접촉력은 아래 평가용이다.

## 동일 조건 60초 구간 비교

총 67초 시뮬레이션: 준비 및 가속 후 5~65초를 측정하고 65초에 Stop, 2초 더 관찰했다. 적분 간격 0.5ms, 제어 간격 20ms. 기존·후보 각각 전진/좌/우 모두 안전 정지 없이 완료하고 Stop 처리까지 확인했다.

접촉률은 각 다리의 중간 스윙 접촉 비율 중 최댓값이다. 발 중심 이동은 쿠션 굴림을 포함하는 대용 지표이며 원본 CSV에 남긴다.

| 정책 | 명령 | 전진속도 m/s | 회전속도 °/s | 중간 스윙 접촉률 | 최대 기울기 | Roll RMS | 높이 변동 폭 |
|---|---|---:|---:|---:|---:|---:|---:|
| 기존 | forward | 0.155 | -0.01 | 58.8% | 5.36° | 1.21° | 17.2mm |
| 기존 | left | -0.000 | 18.78 | 86.9% | 2.53° | 0.85° | 15.8mm |
| 기존 | right | 0.000 | -18.69 | 86.2% | 2.52° | 0.84° | 16.2mm |
| 후보 | forward | 0.109 | 0.03 | 13.3% | 5.17° | 2.06° | 18.3mm |
| 후보 | left | -0.000 | 24.86 | 40.1% | 3.57° | 1.50° | 17.8mm |
| 후보 | right | 0.000 | -25.04 | 40.2% | 3.47° | 1.49° | 17.8mm |

전진 접촉률은 약 59%→13%, 회전은 약 86~87%→40%로 감소했다. 회전속도는 약 19→25°/s로 증가했다. 그러나 전진속도는 약 0.155→0.109m/s로 감소했고 Roll RMS와 몸체 높이 변동은 악화됐다. 따라서 완성/합격으로 표시하지 않는다.

한 번의 최대 발 들림을 모든 스윙의 성공으로 간주하지 않았다. 완결된 스윙별 발 여유 중앙값은 후보 전진 FL/FR/RL/RR 약 16.5/16.7/13.8/14.7mm이며, 하위 10%는 약 4.5/4.7/2.3/2.4mm에 그쳤다. 좌회전 앞발 중앙값도 약 8.2/9.3mm로 15mm 기준에 미달한다. 전체 스윙 기록과 대각선 동시성은 `robust-validation.json`의 `cycle_metrics`에 있다.

## 민감도 검증

각 조건에서 전진·좌·우를 각각 17초 실행하고 Stop을 확인했다. nominal 모델의 60초 시험과 길이가 다르므로 단기 조건 간 비교로만 사용한다.

| 조건 | 전진속도 | 전진 접촉률 | 전진 최대 기울기 |
|---|---:|---:|---:|
| delay60 | 0.110m/s | 18.6% | 4.78° |
| motor85 | 0.055m/s | 44.9% | 7.94° |
| heavy | 0.004m/s | 71.4% | 8.70° |
| soft | 0.096m/s | 52.9% | 5.08° |
| fit_candidate | 0.097m/s | 27.6% | 7.25° |

조건: delay60=가정한 명령 지연 60ms, motor85=모터 토크/속도 한계 85%, heavy=질량 전체 20% 증가, soft=쿠션 접촉 시정수 1.5배, fit_candidate=전진 기록에 더 잘 맞았으나 좌회전 검증에서 거부된 PD 후보. 이 값들은 실측 확정치가 아니다. 모든 조건에서 tilt fault는 없었지만 heavy에서 전진속도가 약 0.004m/s로 무너졌다.

## 재실행

저장소 루트에서 실행한다. 휴대폰 에뮬레이터나 실물 모터를 실행하지 않는다.

```sh
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py \
  --parameters simulation/mujoco/config/measured_response_plant.json \
  --experimental-profiles simulation/mujoco/config/measured_response_profiles.json \
  --profile measured_lift --host 127.0.0.1 --port 8875 --no-ble --no-video
```

MuJoCo 창에서 W는 8초 전진, Space는 Stop이다. 시뮬레이터 TCP 127.0.0.1:8875에서 `gaitprofile measured_lift`, `drive 0 -1000 1`(좌회전), `drive 0 1000 1`(우회전)을 사용할 수 있다. drive는 기존 watchdog 규약에 따라 0.8초 이내 간격의 @D 갱신이 필요하다. 동작 중 정책 변경은 거부하고 Stop 후 변경한다. 실제 iPhone/Bluetooth 설치·연결 검증은 이번 작업의 완료 항목에 포함하지 않는다.

```sh
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/analysis/calibrate_joint_response.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/tuning/search_measured_gait.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/tuning/refine_measured_gait.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/tuning/search_measured_cad.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/tuning/search_measured_turn.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/validation/validate_measured_gait.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/scripts/visualization/preview_measured_gait.py
```

탐색 JSON에는 실패·넘어짐을 포함한 모든 후보가 저장된다. 재탐색은 artifacts의 결과를 갱신하며, 배포용 실험 JSON을 자동 승격하지 않는다.

## 영상 및 남은 검증

[36초 비교 영상](../artifacts/audits/measured-gait-2026-09-12/measured-gait-before-after.mp4): 전진/좌회전/우회전 각 12초, 왼쪽 기존·오른쪽 후보, 위 측면·아래 정면, roll/pitch·네 발 여유 표시. 영상은 계산한 궤적 그대로이며 기울기나 접촉을 보정하여 그리지 않았다.

[원본 실측 비교](./JOINT-CAPABILITY-VALIDATION-2026-09-12.md) · [전체 민감도 결과](../artifacts/audits/measured-gait-2026-09-12/robust-validation.json)

실제 위치 유지 시험/새 정책 실기 전체 동작 시험은 이번 작업에서 수행하지 않았다. 다음 단계는 관절별 고속 응답 측정으로 추정 응답 모델의 모호성을 줄이고, 지지 전환과 하중 변화에 대한 몸체 높이/수평 제어를 보완하는 것이다. 현재 정책을 실물 검증 완료로 간주하지 않는다.

## 구현 검증

- 기록 분석·펌웨어 호스트 회귀 40개 통과.
- 새 궤적 경계 연속성·지지 궤적 보존·관절 제한·공용 보행·Stop 전환·서보 경로 검사 27개 통과.
- 로컬 TCP에서 새 정책 선택, 동작 중 정책 변경 거부, Stop, 정지 완료 후 기존 정책 전환 확인: `artifacts/audits/measured-gait-2026-09-12/tcp-smoke.json`.
- 비교 영상 1280×920, 25fps, 36초. 전진 7초 프레임을 직접 확인했다.
- `git diff --check` 및 새 Python 모듈 문법 검사 통과.
