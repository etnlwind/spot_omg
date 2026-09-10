# 실제/가상 로봇 공통 보행 제어 V17

## 구현 범위

STM32 `shared-locomotion-v17`, MuJoCo `shared-locomotion-v17-sim`, 앱 V0.4.0 (12).

조이스틱 `drive`/`@D`/`@S` 경로에서 실제 로봇과 가상 로봇이 같은 C 계산을 사용한다. 실제 로봇을 설치하거나 움직이지 않고 소스 통합, STM32 빌드, 앱 빌드, 호스트 계산 비교, MuJoCo 동역학 검증을 수행했다.

| 항목 | 공통 구현 |
|---|---|
| 크롤·크루즈·트롯·하이 스텝·legacy 선택 | `locomotion.h`, `locomotion_profiles.h` |
| 발 궤적·역기구학·앞뒤 발 횡이동을 통한 제자리 회전 | `locomotion.h` |
| 입력 제한·후진 제한·위상·보행 주기·시작 진폭 | `drive_control.h` |
| IMU yaw 방향 유지, 회전 입력 우선, 재기준 설정 | `heading_control.h` |
| IMU 정수 필터·센서 누락·기울기 판정 | `attitude_control.h` |
| PI 수평 보정·적분 제한·관절 보정 변화율 | `balance_control.h` |
| 관절별 중심·방향·0.1° 반올림·4096-tick 변환 | `locomotion_servo.h`, `robot_config.c` |

`config/locomotion_profiles.json`이 배포 프로파일의 기준이다. `tools/generate_locomotion_profiles.py`로 C 헤더를 생성한다. 펌웨어 빌드와 시뮬레이터 호스트 로더는 manifest가 변경됐는데 헤더를 생성하지 않은 경우 실행을 거부한다. 기존 `simulation/mujoco/gait_search/profiles/selected.json`은 이전 탐색 결과이며 배포 설정이 아니다. 오프라인 탐색 코드의 명시적인 `robot.profiles` 변경만 실험용 별도 파라미터 경로를 사용한다.

## 실제 펌웨어

`robot_drive`는 공통 보행 루프를 사용한다. Stand 확인 뒤 1초간 보행 준비 자세로 전환하고, 50Hz로 명령을 처리하며, 정지 시 감속 및 Stand 복귀를 수행한다. 기존 `trot4`, `trot5`, `turn` 같은 명시적 레거시 진단 명령은 호환성을 위해 남겼다. 새 정책은 프로파일을 선택한 후 조이스틱/`drive`로 사용한다.

BNO055의 기존 6-byte Euler 읽기에 포함된 yaw를 재사용한다. 새 I2C 거래를 추가하지 않는다. 실패·오래된 yaw는 방향 유지에 사용하지 않는다. BNO055가 아닌 yaw 콜백이 없는 센서에서는 `headinghold` capability를 광고하지 않는다.

하드웨어의 관절별 서보 통신·안전 감시를 유지한다. 800ms 명령 watchdog, 기울기/센서 오류 latch, Recover, 정지 요청, 통신 오류 처리, 과부하 감시는 펌웨어에서 수행한다. 40ms를 초과해 밀린 제어 루프는 오래된 명령을 몰아서 보내지 않고 중단한다. Stand 대기 중에도 공통 수평 보정을 사용하며, 다른 자세/개별 관절/토크 해제 명령이 제어권을 가져가면 대기 보정을 해제한다.

## 공통 명령 및 앱

```text
gaitprofiles
gaitprofile cruise
heading on
heading off
balance on
balance off
locomotiondiag
syncstate
```

프로파일과 보정 설정은 정지 중 변경한다. 앱은 보행 중 설정 변경 요청을 받으면 정지 완료를 기다린다. `gaitprofiles`, `balancecontrol`, `headinghold` capability에 따라 실제/가상 로봇에 같은 설정을 표시한다. 이전 가상 로봇의 `simprofile`/`simbalance` 명령도 호환된다. `syncstate`에는 정책·방향 유지 상태·후진 제한을 포함한다. 실제 펌웨어 balance trace에 방향 오차와 보정 명령을 남긴다.

## 검증

- 펌웨어와 같은 헤더를 별도 `-O0` 실행 파일로 컴파일하고, Python 호스트 `-O2` 바인딩과 5개 정책 × 500프레임을 비교했다. 12개 서보의 송신 tick이 모두 일치했다.
- 모든 배포 정책의 회전/전진/후진 궤적이 실제 서보 중심·방향·범위에 들어가는지 확인했다.
- MuJoCo에 실제 서보 명령 반올림을 적용한 뒤 44개 회전·대각선·회전→직진·마찰/무게중심 변화 사례를 통과했다. `pivot_turn_validation.json`.
- ±0.1Nm 지속 yaw 외력을 가한 크루즈/트롯의 ON/OFF 8개 A/B 사례를 통과했다. `heading_hold_validation.json`.

```sh
python tools/generate_locomotion_profiles.py --check
python firmware/stm32-learning/build_firmware.py --output /private/tmp/spot-shared-v17-build
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/python -m pytest -q tools/servo_tool/tests simulation/mujoco
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py
```

## 실측으로 확정할 부분

코드 일치는 실제 움직임의 완전한 일치를 뜻하지 않는다. 실제 무게·무게중심·발 마찰·관절 유격·모터 힘과 응답 지연은 여전히 실측이 필요하다. 하드웨어에서는 서보가 실제로 움직인 결과를 IMU와 서보 피드백으로 읽지만, MuJoCo에서는 추정한 모터·접촉 모델이 그 결과를 만든다. 통신 준비와 안전 복귀 시간도 물리 장치 상태에 따라 달라진다.

새 배터리가 준비되면 우선 하중을 받치고 기존 관절 중심/방향을 확인한다. `imu`/`locomotiondiag`로 몸체를 손으로 왼쪽으로 돌릴 때 로봇 좌표 yaw가 증가하는지 확인한다. BNO055의 현재 장착 방향이 다르면 `BNO055_YAW_SIGN`을 바로잡아야 한다. 그 후 낮은 입력에서 OFF/ON 직진 기록을 비교해 물성을 보정한다. 실제로 측정하지 않은 보정값을 임의로 하드웨어에 저장하지 않았다.

시뮬레이터 `joint_zero_error_deg`에 FL/FR/RL/RR 각 J1/J2/J3 순서의 12개 잔여 영점 오차를 넣어 기구 비대칭을 재현할 수 있다. 기본은 모두 0이다. 실제 관절 중심 자체는 기존 `tools/servo_tool/config/joints.json`과 `robot_config.c`의 검증된 보정값을 유지한다.
