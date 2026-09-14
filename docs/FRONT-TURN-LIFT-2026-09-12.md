# 회전 앞발 높이 보강 실험 — 2026-09-12

## 저장한 결과

시뮬레이터 정책 `measured_front_lift`를 **실측 기반 · 앞발 높이 보강 · 실험 (검증실패)**로 추가했다. 기존 `measured_lift`를 보존했다. 최종 후보는 `refine_6_20`이다. 앞발 높이는 증가했으나 수평 유지·매 스텝 충분한 여유·뒷발 끌림까지 해결한 정책은 아니다.

실물 펌웨어/서보 설정·위치 원점·방향은 이번 변경에 포함하지 않는다. 결과는 기존 실측 명령과 비교한 추정 물리 모델에서의 시뮬레이션이다. 실기 전체 동작 검증은 수행하지 않았다.

## 구현

- 앞발의 스윙 구간에 **쿠션 바닥 기준 수직 6mm 추가 이동**을 계획한다. CAD 쿠션 바닥·관절 Jacobian을 쓰는 기존 `ArcSwingShape`의 제약 IK로 J1/J2/J3를 함께 계산한다. 수평 발 위치는 유지한다.
- 스윙 시작/끝에서 추가 위치·속도·가속도가 0인 아치다. 지지 중 앞발 목표와 뒷발 추가 높이는 바꾸지 않는다.
- 회전용 기존 IK 높이 파라미터는 201.75→210mm, 앞뒤 위치는 −35→+5mm로 조정했다. 이것은 실제 몸체 지면 높이와 동일한 수치가 아니다.
- 회전 보폭 70mm·주기 1.05초·지지 비율 0.52·yaw 입력 크기를 유지한다. 속도를 몰래 줄이는 방식으로 통과시키지 않는다.
- 기존 `PositionWBC`의 자세 보정을 gain 0.2·관절 보정 한도 6°로 적용한다. 새 `turn_only` 설정은 조향 입력과 전진 입력으로 보정을 부드럽게 혼합한다. 순수 전진의 계획 궤적은 기존 것을 사용한다.
- 지연 IMU 및 관절 피드백 모델을 사용한다. **이 PositionWBC의 encoder 모델은 40ms 지연 벡터이며 실제 12관절 순환 조회의 약 240ms 간격을 재현하지 않는다.** 따라서 실물에 그대로 적용할 수 있다고 판정하지 않았다. 실제 모델로의 승격에는 관측기·순환 읽기 모델의 추가 검증이 필요하다.
- 기존 쿠션 형상, 기구/서보 범위, 모터 토크·속도 한계, 안전 정지 조건을 유지했다.

처음에는 앞발만 10~30mm 추가로 들게 했지만 tilt 정지가 발생했다. 앞뒤 들림 재배분, CAD 수직 이동, 회전 자세, 지지 비율, 자세 보정 강도를 비교했다. 짧은 시험에서 좋아 보인 7mm 후보도 60초에는 최대 약 11°까지 흔들렸고 모터 성능 85% 조건에서 정지했다. 더 작은 6mm 후보를 긴 시험으로 다시 비교하여 선택했다. 실패 기록도 artifacts에 남겼다.

## 60초 비교

67초 실행 중 5~65초를 평가하고 65초에 Stop, 2초 더 관찰했다. 적분 0.5ms, 제어 20ms. 비교 대상은 이전 `measured_lift`다.

| 방향 | 앞발 FL/FR 스윙 최고높이 중앙값: 이전 → 후보 | 회전속도: 이전 → 후보 | 최대 몸체 기울기: 이전 → 후보 |
|---|---|---|---|
| 좌회전 | 8.2 / 9.3 → **19.9 / 13.2mm** | 24.9 → **24.2°/s** | 3.6 → **7.3°** |
| 우회전 | 9.3 / 8.2 → **13.1 / 21.3mm** | 25.0 → **24.0°/s** | 3.5 → **7.9°** |

양쪽 모두 안전 정지 없이 완료했고 Stop 후 보행 종료를 확인했다. 중간 스윙 접촉률의 네 다리 최댓값은 이전 약 40%에서 후보 약 30%/32%다.

**앞발이 매번 15mm 이상 들리는 것은 아니다.** 앞발 스윙 최고높이의 하위 10%는 좌회전 FL/FR 약 2.3/6.4mm, 우회전 약 8.8/3.7mm다. 뒷발 중앙값도 좌회전 RL/RR 약 9.0/13.8mm, 우회전 약 11.3/9.0mm로 낮아졌다. 한 발을 높이 들었다는 최대값만으로 성공 처리하지 않는다. Roll RMS는 좌/우 약 1.93/1.98°로 이전보다 악화됐다.

## 파일·재실행

- 정책: `simulation/mujoco/config/measured_response_profiles.json`의 `measured_front_lift`
- 긴 후보 비교: `artifacts/audits/front-turn-lift-2026-09-12/long-selection.json`
- 최종 추가 조건 시험: `artifacts/audits/front-turn-lift-2026-09-12/stress-validation.json`
- 영상의 실제 설정: `artifacts/audits/front-turn-lift-2026-09-12/video-profile.json`
- 단기 후보/실패: 같은 디렉터리의 `front-only-failed.json`, `balanced-search.json`, `cad-search.json`, `pose-search.json`, `level-search.json`, `transfer-search.json`, `refined-search.json`, `load-search.json`

저장소 루트에서 실행:

```sh
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py \
  --parameters simulation/mujoco/config/measured_response_plant.json \
  --experimental-profiles simulation/mujoco/config/measured_response_profiles.json \
  --profile measured_front_lift --host 127.0.0.1 --port 8875 --no-ble --no-video
```

시뮬레이터 TCP에서 `gaitprofile measured_front_lift`로 선택할 수 있다. 기존 정책으로 돌아갈 때에는 Stop 후 `gaitprofile measured_lift`를 쓴다. 앱 설치·Bluetooth 실기 연결 검증은 이번 작업에 포함하지 않았다.

[좌/우 비교 영상](../artifacts/audits/front-turn-lift-2026-09-12/measured-gait-before-after.mp4): 왼쪽 이전·오른쪽 후보, 위 측면·아래 정면, 실제 MuJoCo 궤적 및 네 발 지면 여유 표시.

## 구현 검사

새 발끝 수직 이동이 실제 MuJoCo 쿠션 바닥에 반영되는지 독립 FK로 확인했다. 지지·뒷발·순수 전진 보존, 시작/끝 연속성, 입력 제한, 공용 보행·서보 경로·Stop 회귀를 포함한 44개 검사를 통과했다. 이는 실물 위치 유지/보행 시험을 대체하지 않는다.

최종 후보의 추가 17초 시험에서 명령 지연 60ms 조건은 좌/우 모두 안전 정지 없이 끝났고 회전속도는 약 22.9/23.0°/s였다. 모터 토크·속도 한계를 85%로 둔 조건은 양쪽 모두 tilt 정지가 발생했다. 따라서 모터 성능 변화에 강건하다고 판정할 수 없다. 영상은 1280×920, 25fps, 24초이며 좌회전 7초 프레임을 직접 확인했다. 저장된 두 실험 정책의 로드와 선택도 확인했다.
