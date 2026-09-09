# 왼쪽 회전 → 전진 전도 조사 및 개선 후보

사용자 보고: 실제 로봇에서 조이스틱을 왼쪽 회전에서 전진으로 바꿀 때 오른쪽으로 넘어짐. 당시 BLE 진단 연결은 장치를 찾지 못해 실패했다. 실제 원인은 미확정이다.

## 반영 범위

- 연속 조이스틱 제어의 회전 요청을 ±1000에서 **±500**으로 제한한다. 제한은 기존 20 ms 입력 변화 제한 **이전**에 적용한다.
- ±500 이하의 세밀한 회전 입력과 전진/후진 최대 입력은 유지한다. 최대 회전 속도는 낮아진다.
- 공통 C `gait_policy_drive_yaw_limit`을 STM32와 STEP 물리 시뮬레이터에서 함께 사용한다.
- 별도 `trot5`, 회전 명령, IMU 보정 설정, 비상 정지와 watchdog은 변경하지 않았다.
- **V14를 실제 로봇에 설치했고 OTA 검증·재부팅·원격 버전 확인을 완료했다. V13 릴리스 바이너리는 복구용으로 보존했다. 실제 보행 시험은 미실행이다.**

## 비교 결과

STEP 모델, 추정 4.398 kg, 11.1 V 3S, 중력·접촉·모터 토크/속도 제한·명령 지연을 적용했다. 정지 2초 → 왼쪽 최대 회전 → 전진 4초를 실행했다. 전환 시점은 시작 후 4.00 / 4.45 / 4.90 / 5.35초, 조건은 기본 / 마찰계수 0.4 / 배터리 위치 이동+40 ms 지연이다. 아래는 자세 보정 OFF 조건이다.

| 지표 | V13 기존 입력 | 회전 제한 후보 |
|---|---:|---:|
| 전도 | 0/12 | 0/12 |
| 전체 조건 최대 기울기 | 6.19° | 4.56° |
| 조건별 최대 기울기의 평균 | 4.20° | 3.09° |
| 기본 조건 최대 기울기 | 6.19° | 3.76° |
| 저마찰 조건 최대 기울기 | 5.29° | 3.03° |
| 위치 이동+지연 최대 기울기 | 6.17° | 4.56° |

기울기는 각 프레임의 max(abs(roll), abs(pitch))이며, 전환 이후 4초까지 평가했다. 모든 개별 전환 시점이 개선된다는 의미는 아니다. 기존 방식에서도 전도가 재현되지 않아 **실제 전도 해결로 판정하지 않는다**. 회전 요청을 줄여 속도를 희생한 보수적 개선이다. 측정 자료는 `simulation/mujoco/gait_search/drive_transition/summary.json`에 보관했다.

단순 1.5초 중립 대기, 입력 변화율 1/4, 발 좌표 합성도 비교했으나 일관된 이점이 없어 채택하지 않았다. 발 좌표 합성 시험 수정은 되돌렸다.

## 검증 범위의 한계

- 실제 몸체 질량·관성·서보 영점·마찰·배터리 위치는 보정되지 않은 추정값이다.
- 궤적 C 코드는 공유하지만 STM32의 전체 스케줄러, IMU 필터, 통신 경합, 틸트 정지 후 Stand 복귀는 재현하지 않는다.
- 시뮬레이터의 실험용 자세 보정을 켠 별도 비교에서는 회전→전진 이전(약 1.72~3.40초)부터 전도했다. 이 보정 경로는 실제 펌웨어 필터 및 모든 파라미터와 동일하지 않으므로 실제 로봇의 원인으로 단정할 수 없다.
- 실제 로봇의 `gaitdiag`, `baldiag`, 틸트 기록과 함께 검증해야 한다. 지금 수정은 지지 발을 감지하는 기능이나 전도 방지 보증이 아니다.

## 다시 실행

프로젝트 폴더에서 새 STEP 모델의 동일 순서를 재생한다. macOS에서는 mjpython을 사용한다. 창은 약 9초 후 자동으로 닫히고 결과가 출력된다.

```sh
cd /Users/etnlwind/project/spot_omg
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/check_drive_transition.py --replay
```

기존 방식 비교는 `--legacy`를 추가한다. 전체 24회 수치 비교는 일반 python으로 실행한다.

```sh
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/check_drive_transition.py
```

검증: 관련 Python/C 검사 **138 passed, 23 subtests passed**. 공유 C 입력 제한 경계와 기본 조건 전환 회귀 검사 포함. STM32 clean build 성공(`/private/tmp/spot-turn-transition-build`); 이 디렉터리의 빌드명은 기존 빌드 스크립트의 V13 이름을 유지하므로 정식 배포 릴리스로 취급하지 않는다.

## V14 설치 요청 후 준비 상태

사용자가 실제 로봇 설치를 요청하여 `ROBOT_CONTROL_REV`를 `turn-limit-v14`로 변경하고 새로 빌드했다. 크기 136700 bytes, SHA256 `dc72fa2ba713bb9f7e867e4b4af4965e232db3f0147f2c4cd5edd29ec3cdf2db`. 설치 전 BLE `syncstate` 요청에서 장치를 찾지 못해 현재 전송 대기 중이다. `releases/turn-limit-v14/manifest.json`의 배포 상태를 확인할 것. IMU 설정은 변경하지 않았다. 이전 원격 확인은 V13 `balance=full`이며, 현재 상태는 재확인되지 않았다.


## V14 실제 설치 완료

사용자가 iPhone 앱 연결을 해제한 후 다시 연결했다. 설치 전 V13 `pose=stand torque=on safety=ok balance=full`을 확인했다. V14 OTA 전송, STM32 flash 100%, `STM32 firmware verified and rebooted` 및 CLI 정상 종료를 확인했다.

재부팅 후 원격 확인:
```
$SPOTSTATE pose=custom error=349 torque=off safety=ok balance=full rev=turn-limit-v14 caps=trot5
```

IMU 설정은 full 유지. 조이스틱 보행은 설정에 따라 자세 보정과 기울기 감시를 적용하며, 별도 trot5는 자세 보정을 억제하고 기울기 감시를 유지한다. 설치 후 물리 보행 명령은 보내지 않았다. BLE 진단 연결은 정상 종료했다.

## 후속 응답 미수신 조사

앱 V0.2.0 (4)의 최신 로그에서 notify 활성화 및 syncstate 3회 ACK는 있으나 RX 없음. 앱 연결을 해제한 뒤 Mac Bleak에서도 syncstate 3회 응답이 없었다. ESP32 진단 UUID `6e400004-...`는 실제 기기에 존재하지 않았다(진단 빌드 미설치). 따라서 앱 버전 파싱 문제로 볼 수 없으며 UART/STM32/ESP32 수신 경로 원인은 미확정. 로봇과 ESP32 전원을 완전히 껐다 켠 뒤 앱 미연결 상태로 조회를 다시 진행하도록 요청했다. 이 조사에서는 움직임·리셋·펌웨어 쓰기 명령을 보내지 않았다.

## 직진 오른쪽 쏠림 후속

완료된 실제 V14 세션: balance=on, samples=595, min_voltage=11300mV, lag=0, derate=no, fall=no. 전체 입력에는 처음 우회전과 후반 작은 좌회전이 섞였으므로 순수 직진으로 볼 수 없다. 앱 V0.2.0 (5)에 직진 근처 회전 입력 제거를 반영해 iPhone 설치 완료(XCTest 29/29). 실제 오른쪽 쏠림이 해결되었다고 판정하지 않는다. 세부 사항은 iOS README의 V0.2.0 (5) 항목 참조.
