# Spot OMG 전체 업데이트 — 2026-09-09

## 현재 사용 상태

| 대상 | 상태 |
|---|---|
| iPhone 앱 | **V0.3.0 (6) Debug**, 설치 완료 |
| 실제 로봇 펌웨어 | 마지막 설치 V15 `stride-drive-v15` |
| 펌웨어 소스 | V16 `walk-stance-v16` 및 공통 drive 입력 계산 리팩토링 반영 |
| V16 실제 설치 | 미실시. 정상 교체 배터리가 아직 준비되지 않음 |
| MuJoCo | 새 STEP 모델 기반 물리 가상 로봇, GUI 표시 및 명령 수신 |
| 앱↔가상 로봇 | BLE 연동 확인. TCP 연결도 유지 |
| 실물과 물리적 동일성 | 아직 미검증. 추정 물성과 미공유 제어 계층이 남아 있음 |

## 앱 및 통신

- 실제 BLE, 가상 BLE, 가상 TCP를 명시적으로 선택한다.
- 실제 `SpotOMG-Bridge`와 가상 `SpotOMG-Sim`은 서비스/특성 UUID를 분리한다.
- 가상 장치는 `backend=sim protocol=1` 식별을 통과해야 조작 가능하다.
- 대상 변경은 연결, 조이스틱 상태, 대기 명령을 정리한다. 실물로 자동 대체하지 않는다.
- BLE write ACK, 알림 구독 재설정, 초기 상태 재요청 및 응답 실패 표시를 보강했다.
- 버전 문자열을 작은 글씨로 표시하고, 서보 전압과 가상 전압을 구분한다.
- 조이스틱 직진 부근의 작은 좌우 입력을 억제하고, 명령/정지/재연결 상태를 검증했다.
- `Spot OMG V0.3.0 (6) - Debug` 빌드가 iPhone에 설치되었다.

## MuJoCo와 Mac BLE 수신 앱

기존 단독 보행 재생을 넘어 앱과 spotctl의 명령을 받는 가상 로봇 콘솔을 추가했다.
MuJoCo GUI와 Mac BLE 수신 앱을 함께 실행하는 것이 기본값이다.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py
```

앱에서 **가상 로봇 · BLE → 선택한 대상 연결**을 누른다. IP 입력은 필요 없다.
Mac BLE 수신 앱은 내부 `127.0.0.1:8765` TCP로 MuJoCo에 연결한다.
앱이 연결 중이면 다른 spotctl 제어 연결은 받지 않는다.

```sh
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python -m servo.cli --sim-host 127.0.0.1 --tcp-port 8765 console send syncstate
```

- 자동 검증은 `--headless --no-ble`을 사용한다.
- 가상 콘솔은 자세, torque off, drive heartbeat/stop, trot5/trot4/turn/crab 등을 처리한다.
- 상태·전압·기울기·실제 관절·토크·접촉은 물리 모델에서 계산한다.
- 지원하지 않는 EEPROM/배포/실제 버스 기능은 오류로 반환한다.
- 통신 종료, 큐 초과, heartbeat 지연 시 제어 입력이 계속 쌓이지 않도록 종료/정지 처리한다.
- Mac BLE 수신 앱은 MTU 분할, notification backpressure, 단일 제어 소유권을 처리한다.

## 모델·보행 정책·펌웨어

- 새 STEP에서 몸체와 12개 모터 관절 그룹을 분리한 STL, 관절 매핑, 물리 설정을 추가했다.
- **300mm는 중앙 알루미늄 프레임 길이이며 로봇 전체 길이가 아니다.**
- PLA/PETG, 모터, 배터리의 질량과 무게중심은 추정 모델이다.
- 중력·관성·마찰·접촉·모터 토크/속도·명령 지연·전원 강하를 적용한다.
- 보행 탐색/자세 비교/전환 검증 스크립트와 결과 JSON, 시각 자료를 함께 보관한다.
- V13 trot5, V14 회전 입력 제한, V15 전진 보폭 확대, V16 보행 중 40/80° 자세 변경의 기록을 정리했다.
- V16은 정적인 stand 45/90°를 유지하면서 연속 보행 진입/종료 자세 전환을 추가한다.
- `gait_policy.h`를 호스트 C 라이브러리로 호출하여 궤적을 공유한다.
- drive 변화 제한과 주기 계산을 공통 C 함수로 추출해 STM32와 MuJoCo 어댑터가 같은 정수 계산을 사용한다.
- 펌웨어 빌드 스크립트와 객체 목록, 기존 V15/V16 릴리스 산출물 및 매니페스트를 보관한다.

현재 가상 서버의 명령 상태기·자세 전환·IMU 처리는 STM32 전체를 재호스팅한 것이 아니다.
가상 IMU는 모니터링이며 실제 균형 보정·필터·안전 상태기와 차이가 있다.
따라서 **시뮬레이션 성공을 실물의 완벽한 동작으로 판정해서는 안 된다.**
실물 동작 검증은 정상 전원을 준비한 후 별도로 진행해야 한다.

## 검증 결과

- Python/펌웨어 호스트 전체: **222개 + subtest 26개 통과** (TCP 포함).
- BLE 추가 후 가상 로봇 테스트: **7개 통과**.
- iOS 최신 전체: **34개 통과**, iPhone용 Debug 서명 빌드 성공.
- Mac BLE 수신 앱: Swift 빌드·서명 성공, 실제 BLE 광고와 MuJoCo 연결 확인.
- iPhone 기록: `@D`, `@S`, 정상 stopped 응답, `backend=sim`, `source=simulated` 전압 확인.
- 사용자가 앱 연동 정상 동작 확인.
- GUI MuJoCo에서 `trot5 3 844` 정상 종료 확인.
- TCP 전진/정지 시험에서 X 약 +6.1cm 이동 확인. 단일 시뮬레이션 결과이며 실물 정확도의 근거는 아님.
- 공통 계산 리팩토링 후 STM32 빌드: 137812 bytes,
  SHA256 `97d1a00911feb0b861f7b6e041c334908b7c7497505e38e768151e6e82b3f252`.
  이 빌드는 검증용이며 이전에 보관한 V16 릴리스 바이너리와 구분한다.
- `git diff --check` 통과.

## 커밋 범위 및 상세 문서

이번 전체 커밋에는 누적된 앱, 펌웨어, spotctl, CAD 메시/설정, MuJoCo 정책/시험 결과,
설명 문서와 기존 VS Code/STM32 IDE 설정 변경을 함께 포함한다.
빌드 캐시와 임시 실행 로그 등 기존 ignore 대상은 포함하지 않는다.
원본 STEP은 사용자 Downloads 경로에 있으며 이번 커밋에는 변환된 메시와 매핑을 포함한다.

- [BLE 및 앱 설치 안내](VIRTUAL-BLE-2026-09-09.md)
- [가상 콘솔 구현 범위](VIRTUAL-ROBOT-2026-09-09.md)
- [V16 펌웨어](UPDATE-2026-09-09-V16.md)
- [다리 자세 비교](GAIT-STANCE-2026-09-09.md)
- [V15 보폭](UPDATE-2026-09-08-V15.md)
- [회전→전진 전환](DRIVE-TRANSITION-2026-09-08.md)
- [V13 trot5](UPDATE-2026-09-08-V13.md)
- [이전 인수인계 기록](HANDOFF-2026-09-08.md)
