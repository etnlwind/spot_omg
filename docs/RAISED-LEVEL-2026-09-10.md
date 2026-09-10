# Level15 발 들림 검증 — 2026-09-10

기존 `level`은 보존하고 `level15`를 추가했다. 앱 표시는 **수평 + 발 들기 · 15mm**다. 기본 프로필 cruise는 유지한다.

## 구현

공유 manifest의 Level15 파라미터는 주기 1.4초, 지지율 0.64, 보폭 65mm, 명령 발높이 15mm, 몸높이 201.75mm다. 기존 Level의 IMU/J1 수평 보정을 적용하며, 앱·가상로봇·STM32가 동일한 프로필과 C 제어기를 사용한다. 후진 입력은 60%로 제한한다. 실제 하드웨어에는 설치하지 않았다.

앱 V0.4.2 (14), 펌웨어 shared-locomotion-v20, 가상로봇 shared-locomotion-v20-sim.

## 측정 방법

추정 질량·마찰·중력·접촉·서보 지연/토크 제한·서보 양자화·BNO055 지연을 유지했다. 제어기에 MuJoCo 정답 자세를 직접 주입하지 않는다. 실제 지면 여유는 발 충돌 구의 중심 높이에서 반지름을 뺀 값이다. 아래 발 높이는 완전히 관측된 보행 주기별 최고 높이의 중앙값이며, 명령 높이와 다르다.

최대 전진 60초, 시작/정지 포함. RMS는 안정 구간의 roll/pitch 제곱합 평균 제곱근, 최대 기울기는 시작/정지를 포함한 개별 축 절댓값 최대다.

| 기본 모델, seed55 | 기존 Level | Level15 |
|---|---:|---:|
| 기울기 RMS (°) | 0.271 | 0.373 |
| 최대 기울기 (°) | 0.644 | 2.029 |
| 속도 (m/s) | 0.092 | 0.076 |
| 수직 흔들림 표준편차 (mm) | 1.863 | 2.066 |
| 중간 스윙에서 1mm 미만 비율 (%) | 92.593 | 26.941 |
| 앞왼쪽 주기 최고 발높이 중앙값 (mm) | 0.13 | 5.95 |
| 앞오른쪽 주기 최고 발높이 중앙값 (mm) | 0.15 | 6.31 |
| 뒤왼쪽 주기 최고 발높이 중앙값 (mm) | 1.22 | 9.52 |
| 뒤오른쪽 주기 최고 발높이 중앙값 (mm) | 1.10 | 9.75 |

기본 모델 3개 노이즈 seed에서 RMS 0.370–0.395°, 최대 1.90–2.03°. 이전 Level보다 발 들림은 개선되었으나 속도는 약 17% 줄고 기울기는 증가했다. nominal 스윙 중간 구간의 약 27%는 여전히 지면 여유 1mm 미만이다. 발 끌림 완전 해소 또는 실제 지면 여유 15mm 달성을 주장하지 않는다.

## 추가 조건과 한계

- 60초 전진 7개 실행(기존 Level 기준 포함): 안전 정지, 비발 접촉 없음.
- BNO055 지연 60ms에서도 넘어짐 없음.
- 배터리 외 질량 +20%, 마찰 0.55, 전압 10.8V에서는 앞발 높이 중앙값 약 1.7mm. 안정성 검사는 통과했으나 높은 발 들림은 유지하지 못했다.
- 배터리 무게중심 이동 및 명령 지연 40ms에서는 발높이 좌우 차이가 커짐. 하중/무게중심 추정에 민감하다.
- 제자리 회전, 전진 회전, 회전→전진, 후진 회전 좌우 총 8개: 넘어짐 없이 통과. 최대 기울기 약 4.01°(회전→전진).
- 실제 로봇 질량·무게중심·서보 응답 실측 전에는 실물 성능을 보장할 수 없다.

## 영상

`artifacts/gait-videos/2026-09-10/final-comparison.mp4`: 왼쪽 기존 7mm, 오른쪽 최종 Level15. 23초, 25fps, 1배속, 2–20초 최대 전진 후 정지. 1920×540, H.264. 프레임 이미지와 전체 디코딩을 확인했다.

`final-level15.mp4`는 배포 프로필을 직접 선택해 촬영했다. `height-comparison.mp4` 및 `candidate-12mm/15mm/20mm.mp4`는 보폭 50mm의 초기 비교이며, 최종 보폭 65mm와 구별해야 한다. 각 영상의 JSON에는 프레임별 자세·발높이를 저장했다.

재촬영:

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/record_gait_video.py --profile level15
```

MuJoCo UI:

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --viewer --profile level15
```

UI 숫자 8은 기존 Level, 9는 Level15. 원격 연결이 없을 때 W로 8초 보행, Space로 정지한다. 앱에서는 가상 로봇 BLE 연결 후 해당 모드를 선택한다.

## 검증 산출물

- `simulation/mujoco/level15_gait_validation.json`, `level15_turn_validation.json`
- Python 제어/프로토콜 테스트 56개 통과. Level/Level15 J1 양자화 이상 보정 테스트 통과.
- iOS XCTest 45개 통과. UJIN17에 V0.4.2 (14) 설치/실행 확인.
- STM32 빌드 성공, 153028 bytes. SHA256 `e36e37db60a6ac4ad3868f045ce7d79e78374183c6a240368d2b36274531deb7`.
- 기존 정책 스냅샷: `config/snapshots/locomotion-before-raised-level-2026-09-10.json`.

탐색용 shape/stance/crawl/weight-transfer/swing-balance 변형은 선택하지 않았다. 임시 라이브러리 경로 기록은 과거 실행의 참고 정보이며 재현 시 해당 탐색 스크립트를 다시 실행해야 한다.
