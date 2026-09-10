# Stow 수납 자세 설계 검토 — 2026-09-10

## 결과

MuJoCo의 중력·추정 질량·서보 토크/속도·접촉 계산으로 15초 동작을 기록했다. 기존 가상 로봇과 물리 로봇에는 명령을 전송하지 않았다.

- 기존 범위(J2 −45~100°): 앞·뒷발 충돌, 최대 기울기 35.47°, 목표 추종 오차 최대 49.38°. 요청 동작에 부적합하다.
- 확장 범위(J2 −95~105°): **설계 검토 전용 가정**. 앞다리 J2 목표 −85→95°(180°), 뒷다리 −85°, 모든 J3 최종 0°. 실제 마지막 자세는 앞 J2 약90°, 뒤 J2 약−89°로 목표와 다르다.
- 확장 모델 최종 CAD 외곽: 약513×242×118mm. Landing 약578×241×190mm 대비 AABB 부피 44.7% 감소. 전역 최적화 결과가 아니라 첫 수납 후보다.
- 확장 모델의 전환 중 최대 기울기는 35.48°, 최종 기울기는 0.00°. 몸체를 의도적으로 뒤로 낮추는 구간이 포함된다.
- 확장 모델에서 바닥 이외 충돌 프록시의 1mm 초과 침투는 검출되지 않았다. **상세 CAD 메쉬 충돌, 배선, 모터 케이스, 실제 회전 한계는 검증하지 못했으므로 실로봇 적용 승인 아님.** 서보 인코딩은 기존 것을 유지했다.

## 단계

Landing 2초 → 뒷다리를 앞으로 뻗기 3초 → 앞다리를 앞으로 뻗기 3초 → 앞다리 J2 180° 회전 5초 → Stow 유지 2초.

움직임은 원래 물리 계산에서 기록했고, 뷰어는 기록된 상태를 재생한다. 균형 보정은 의도한 자세 전환과 충돌하지 않도록 이 별도 실험에서 껐다. 기존 가상 로봇의 제어기·관절 제한·앱·펌웨어는 변경하지 않았다.

## 재생

저장소 루트에서:

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/stow_preview.py --case extended --viewer
```

기존 범위 비교는 `--case nominal --viewer`. 다시 물리 계산 및 영상 생성은 Python으로 `--case extended --run --video`를 사용한다.

영상과 정량 결과: `simulation/mujoco/diagnostics/stow/`의 `nominal.mp4`, `extended.mp4`, 각 JSON/NPZ. 이 별도 검토 뷰어는 앱의 일반 가상 로봇 영상 서버와 연결하지 않는다.

## 다음 설계 조건

J2가 최소 −85~95°를 실제로 통과할 수 있는지 확인하고, 전환 전 구간에서 상세 메쉬/배선 여유를 검증해야 한다. 검증 후에만 실제 관절 제한과 Stow 진입·탈출·중단 절차를 공유 제어기에 추가한다. 현재는 별도 실험으로 유지한다.

## 위쪽 회전 후보 — overhead

사용자 요청에 따라 앞다리 J2를 −85→−265°로 회전하여 아래쪽 대신 위쪽을 통과하도록 변경했다. −265°는 최종 기하학적 방향에서 기존 +95°와 같지만 회전 경로가 반대다. 기존 nominal/extended 기록은 유지했다.

- 앞다리 J2 범위: 설계 검토 모델에서만 −275~105°.
- 이 경로는 기존 단회전 서보 인코딩 범위를 넘으므로 `overhead` 실험에서만 `embedded_servo_quantization=False`로 설정했다. 중력, 토크/속도 한계, 전원, 접촉 동역학은 유지한다. 실제 펌웨어 적용 가능한 경로라고 해석하면 안 된다.
- 전체 동작 중 최대 기울기 13.05° (아래쪽 후보 35.48°), 최대 추종 오차 6.91°. 최종 외곽 약513×241×118mm.
- 바닥 이외 충돌 프록시의 1mm 초과 침투는 검출되지 않았다. 상세 메쉬·배선 충돌 검증을 대신하지 않는다.
- 테스트 4개 통과: 중간 경로에서 발이 고관절보다 20cm 이상 위를 통과하고, 최종 기하학적 방향은 기존 후보와 같으며, 일반 모델의 서보 인코딩은 유지됨을 확인했다.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/stow_preview.py --case overhead --viewer
```

영상: `simulation/mujoco/diagnostics/stow/overhead.mp4`.

## 동시 저속 접기 — simultaneous

순차적인 뒷다리→앞다리 단계를 없애고 Landing 후 모든 J2/J3가 하나의 smootherstep 시간축으로 동시에 출발·도착하도록 변경했다. 접기 시간은 12초이며 Landing 2초, 최종 유지 2초를 포함한 기록은 16초다. 앞다리는 위쪽 회전 방향을 유지한다. 개별 관절의 각속도는 이동 각도에 비례하고 공통 진행률을 사용한다.

- 최종 크기 약513×241×118mm, 최대 기울기15.61°, 최대 추종 오차6.70°.
- 바닥 이외 충돌 프록시의 1mm 초과 침투 미검출. 상세 CAD·배선 및 실제 서보 회전 한계 검증은 미완료다.
- 이전과 동일한 설계 검토용 확장 범위/단회전 인코딩 해제 조건이며, 실제 로봇·앱의 일반 제어에는 추가하지 않았다.
- 영상 `simulation/mujoco/diagnostics/stow/simultaneous.mp4`.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/stow_preview.py --case simultaneous --viewer
```

## 채택한 기준 및 역방향 Landing 복귀 — stow-cycle

사용자가 `simultaneous` 후보를 기준 버전으로 채택했다. 기준 접기 단계와 목표는 그대로 유지하고, Stow 유지 뒤 12초 동안 동일한 공통 smootherstep 경로를 반대로 명령하여 Landing으로 펼친다. 네 다리가 동시에 출발·도착한다. 펼치기는 동역학을 새로 계산한 것이며 영상 역재생이 아니다.

- 기본 `--case`는 `stow-cycle`로 변경했다. Landing 2초 → 동시 접기12초 → Stow 유지2초 → 동시 펼치기12초 → Landing 유지2초.
- 전체 최대 기울기15.61°, 최대 추종 오차8.49°. 최종 Landing 기울기0.107°, 최종 관절 목표 오차 최대 약0.843°.
- 바닥 이외 충돌 프록시의 1mm 초과 침투 미검출. 같은 확장 관절/인코딩 가정을 사용하는 설계 검토 기준이며 물리 로봇 적용 승인은 아니다.
- 테스트6개 통과. 접기 기준 보존, 전·후방 공통 시간축, 역경로의 대칭성을 검증했다(공유 C float32 반올림 허용치0.001°).
- 전체 영상 `simulation/mujoco/diagnostics/stow/stow-cycle.mp4`, 펼치기 영상 `simulation/mujoco/diagnostics/stow/unfold-to-landing.mp4`.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/stow_preview.py --viewer
```

## 가상 로봇 명령 및 앱 V0.5.0 (28)

`virtual_robot.py --stow`로 시작한 실험 가상 로봇은 `simstow` 기능을 알린다. 앱은 해당 기능을 확인한 가상 로봇에만 Stow 버튼을 표시하며, 전송 함수에서도 실제 로봇으로 Stow를 보내지 못하게 검사한다.

- `stow`: Landing이 아니면 2초 Landing 준비 후 12초 동시 접기. Landing에서 시작하면 바로 12초 접기.
- `landing`: Stow 및 Stow 중단 상태에서 12초 역경로 펼치기.
- `hold`, Ctrl-C, 연결 해제: Stow 전환을 중단하고 측정 관절 위치를 유지. 다시 연결하여 `landing` 또는 `stow`로 진행할 수 있다.
- Stow에서 보행/Stand 등은 거부하며 Landing 복귀가 먼저 필요하다.
- 정상 보행의 서보 인코딩은 유지하고 Stow 경로 동안에만 연속 각도를 사용한다. Landing 복귀 완료 후 기존 인코딩으로 복원한다. 확장 관절 범위는 `--stow` 모델에만 적용한다. 실물 모터가 기구적으로 불가능하다는 의미가 아니라 현재 펌웨어의 각도/인코딩 체계를 넘어서는 실험이다.
- 앱의 기존 Landing 버튼으로 펼칠 수 있고, 자세 상세 영역에도 Stow가 추가되었다.
- 실제 제어기를 거친 물리 왕복, 인코딩 복구, 중단/재연결, 일반 모델 차단 및 기존 가상 로봇 회귀 테스트 총39개 통과. iPhone용 Debug 빌드 성공 및 설치 완료.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --viewer --stow
```

## 실제 실행 사전검사 추가

`validate_stow_hardware.py`는 시뮬레이션의 인코딩 우회를 사용하지 않고, 펌웨어와 공유하는 C `spot_servo_encode`로 전체 접기 경로를 검사한다. 현재 결과는 hardware_ready=false이며 종료코드2를 반환한다.

- FL J2 ID2: 현재 영점 기반 표현 범위 −177.363~182.549°, 접기7.42초에서 실패.
- FR J2 ID5: −168.926~190.986°, 접기7.22초에서 실패.
- Stow 목표 −265°는 현재 인코딩으로 표현할 수 없다. 이것은 기구적인 회전 불가능의 증명이 아니다.
- 영점/엔코더 경계 재설계 또는 지원 운전 모드 확인이 필요하다. 최종 각도를 단순히360°로 나눈 나머지로 바꾸면 원하는 위쪽 경로를 보장하지 못하므로 해결로 간주하지 않는다.
- STEP 및 영점/모터 모델 자료는 존재하지만 물리 파라미터 파일 자체에 추정 질량·마찰·축·무게중심 및 단순 충돌 형상이라는 제한이 기록되어 있다. 이를 실제 측정/사양 기반 값과 혼동하지 않는다.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/validate_stow_hardware.py
```

결과: `simulation/mujoco/diagnostics/stow/hardware-preflight.json`.

## 앱 경로에서 앞다리가 멈추던 문제 수정

사용자 화면에서 앞다리가 앞으로 뻗은 상태로 멈추는 현상을 확인했다. 원인은 공유 balance 함수가 enabled=false인 경우에도 J2의 −45~100° 제한을 적용하기 때문이다. 미리보기는 이 보행 보정 단계를 거치지 않았고, 기존 통합 테스트는 Stow의 최종 관절 위치를 확인하지 않아 이를 놓쳤다.

- 실험 Stow 경로에서만 보행용 balance/clamp를 통과하지 않고 공통 Stow 목표를 전달한다. 일반 보행과 실제 펌웨어 제한은 유지한다.
- 새 회귀 검사는 목표 배열, 실제 앞다리 J2가 −250°를 넘었는지, 최종 목표 오차, Landing 복귀와 인코딩 복원을 확인한다.
- 완료 시 실제 관절 오차가12°를 넘으면 OK 대신 ERROR를 응답한다. 기존처럼 시간 경과만으로 성공 처리하지 않는다.
- 가상 로봇 회귀33개 및 완료 검사 반영 후 Stow 검사3개 통과.

### 실행 중인 TCP 가상 로봇 최종 확인

실행 중인 가상 로봇에 TCP로 `stow` → `targets` → `landing`을 전송해 왕복을 확인했다. Stow의 실제 앞다리 J2는 FL −263.39°, FR −263.46°였으며 `OK stow` 응답 후 Landing 복귀도 `OK landing`으로 완료했다. 이후 사용자도 앱에서 동작 성공을 확인했다.
