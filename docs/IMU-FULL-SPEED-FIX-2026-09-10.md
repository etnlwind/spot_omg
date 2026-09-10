# IMU 최대 직진 흔들림 수정 — v18

사용자가 IMU 지지 다리 보정 모드에서 조이스틱을 앞으로 끝까지 밀었을 때 과도한 흔들림을 보고했다. 기존 검증은 직진 입력 800/1000이었고 이 조건을 놓쳤다. 정지 제한에 걸리지 않는 것과 매끄러운 보행은 다르다.

## 재현 및 원인 분석

동일한 모델/궤적에서 full 1000 명령을 재생하고, 각속도 예측/J1/착지 위치/지지 비중 보정을 하나씩 제거했다. 18초 구동 비교에서 기존 lift의 자세 RMS는 2.75도, 최초 IMU 정책은 3.81도였다. 각속도 예측 제거만으로 3.20도까지 감소했다. 보정 항목이 결합된 비선형 폐루프이므로 단일 원인으로 단정하지 않는다. 결과: `simulation/mujoco/imu_wobble_ablation.json`.

처음에는 지지 비중 변화와 J1을 약화하고 예측을 제거했으나, 60초 구동에서 다시 큰 흔들림이 나타났다. 해당 중간 결과는 `simulation/mujoco/diagnostics/imu-first-fix-full-speed.json`에 보존했다.

## 최종 변경

- 보행 중 지연된 Euler 각속도를 사용한 25ms 예측 제거.
- 스윙/지지 전환에 따른 J1 부호 반전 제거.
- J1은 필터된 roll에 작은 비례 보정(0.5 deg/rad)을 적용. 적분 오차를 J1에 증폭하지 않음. 예상 지지 비중은 90~100% 범위에서만 반영.
- J2/J3의 IMU 수평 PI 보정은 유지하고, 스윙 시 길이 보정량을 별도로 줄이는 동작 제거.
- pitch로 스윙 발 착지 위치를 추가 이동시키는 동작 제거.
- IMU 직진 방향 유지, 기존 정지/센서 실패/기울기 안전 제한 유지.
- 명령 입력, 기본 주기 1.05초, 보폭 45mm, 발 들기 최대 명령 20mm는 변경하지 않음. 속도 입력을 80%로 제한하는 방식이 아님.
- 공통 C 코드에 반영하여 STM32와 MuJoCo가 동일한 보정을 사용. revision은 `shared-locomotion-v18` / `shared-locomotion-v18-sim`.

현재 imu 모드는 최초 정책보다 보수적이며, 적극적인 착지 위치 보정은 제공하지 않는다. 기본 lift에도 IMU 수평 및 heading 보정이 있으므로 IMU를 켜고 끈 단순 비교가 아니다.

## 최종 60초 최대 직진 결과

각 실행은 60초 동안 `@D ... 1000 0`을 유지하고 최종 정지를 확인한다. RMS는 초기 준비 구간 및 정지 구간을 제외한다. script: `simulation/mujoco/validate_imu_full_speed.py`, 결과: `simulation/mujoco/imu_full_speed_validation.json`.

명목 물성, seed 55에서:

| 지표 | 최초 IMU v17 | 수정 IMU v18 | 기존 lift |
|---|---:|---:|---:|
| 자세 RMS | 4.056° | 2.820° | 2.673° |
| 자세 변화율 RMS | 36.008°/s | 30.759°/s | 30.678°/s |
| 최대 기울기 | 11.857° | 7.881° | 7.998° |
| 평균 이동 속도 | 0.0502m/s | 0.0766m/s | 0.0819m/s |

수정 전 대비 자세 RMS 약 30%, 최대 기울기 약 34% 감소. lift보다 모든 지표가 우월하다는 결과는 아니며 완전한 수평 유지나 무진동 보행을 의미하지 않는다.

최종 정책은 센서 seed 55/77/101, 지연 60ms, 무게·마찰 변경, 무게중심 변경의 6개 60초 실행에서 안전 정지 없이 최종 명령 정지 완료, 비발 접촉 없음. 비교용 최초 IMU와 lift 포함 8개 실행을 기록했다.

- 추가 제자리 회전·전진 회전·회전→직진 3개 회귀 실행 정상 완료 (`simulation/mujoco/imu_stable_transition_validation.json`).
- 공통 C/서보 명령 parity·IMU 실패·OFF·프로토콜 테스트 52개 통과.
- STM32 v18 빌드 성공, 152688 bytes, `/private/tmp/spot-imu-stable-build/shared-locomotion-v18.bin`.
- 실물 펌웨어는 설치하지 않음. 앱 프로토콜/모드 이름은 동일하여 앱 소스 변경은 필요하지 않음.
- 최초/중간 C 헤더는 diagnostics에 보존하여 비교 실험을 재현 가능하게 함.

## 실행

```bash
cd /Users/etnlwind/project/spot_omg
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --viewer --profile imu
```

앱을 재연결하면 syncstate revision으로 v18 적용을 구분할 수 있다. 같은 `IMU 적응 · 지지 다리 보정` 모드를 사용한다.
