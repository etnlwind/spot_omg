# Stow / Landing 실기 중단 진단

사용자 보고: 자동 Stow가 앞다리를 끝까지 뒤로 돌리지 못함. 수동으로 접은 상태에서 Landing을 실행하면 앞다리가 올라가다가 다시 내려옴.

## 확보한 증거

- V27 boot25 저장 로그: Stow 여러 차례와 Landing 여러 차례가 `final position verification failed`로 종료.
- Landing seq3756/3759/3762/3765는 시작 약881~882ms 후 실패.
- 상태: `pose=stow-paused torque=off safety=fault fault_code=7`.
- 읽기 검사: 서보12개 모두 응답, hw=0, 11.2~11.4V, 읽기 재시도0.
- 현재 자세는 수동 조작 이후이므로 실패 순간의 실측 자세를 대신하지 않음.
- 기존 `servo=3` 로그는 추종 오차 분기에서 실패 ID를 갱신하지 않아 원인 축으로 단정할 수 없음.

## V28 진단 변경

- 위치 추종 오차 분기에서 실제 실패 서보 ID, 경과시간, signed 목표/실측값을 보존.
- 종료 후 `STOW_TRACK`를 콘솔과 STM32 영구 로그에 기록. 구동 중 플래시 쓰기는 추가하지 않음.
- `stowdiag`: 앞 J2 ID2/5의 원시 위치, signed 목표, 모드, 해상도, 각도 제한, 토크 및 오류를 읽기만 함.
- 움직임 경로, 속도 및 보호 기준은 변경하지 않음.
- 펌웨어 테스트28개 통과. 600틱 추종 오차 주입 시 실패 축/값/시점 기록과 토크 차단 검증 포함.

원인 확정 및 동작 수정 완료로 간주하지 않는다. 현재 정지 상태 조회만으로 실패 순간의 카운터 변화나 기계적 추종 실패를 구별할 수 없다.

## 진단 이미지 설치 및 원시 조회

V28-diag 설치 검증/재부팅 완료. 이미지169764바이트, SHA256 f4f4ab5039c7b9d2bad2ed82c93f230a672281838ca3317f7d0b374ef26bd156.

```
STOW_RAW id=2 word=858 raw=858 goal=850 mode=0 res=1 min=0 max=0 torque=0 hw=0
STOW_RAW id=5 word=3161 raw=3161 goal=3171 mode=0 res=1 min=0 max=0 torque=0 hw=0
```

조회 결과상 앞 J2 두 축은 min/max 모두0. 단회전 제한4095가 남아 있는 상태는 아니다. 이 정지 상태 조회만으로 다회전 실제 추종 성공을 입증하지 않는다. 이번 진단에서 움직임 명령은 보내지 않았다.

추가 발견: 수동으로 접은 현재 실측 자세는 원래 직선 보간 경로에서 벗어나므로 재부팅 후 stow_probe의 모든 축18도 일치 조건을 충족하지 않을 수 있다. 자동 Stow 도중의 기존 실패와는 별도로 수동 자세 인식/복귀 경로도 검토 필요. 진단 재부팅 이후 일반 Landing 경로가 선택되지 않도록 복귀 전 확인해야 한다.

## V29 실제 복귀 시험

수동 Stow의 앞 J2가 -150도보다 접혔으면 기존 모든 축의 보간선 일치 조건과 별도로 Stow 복귀 경로를 선택한다. snapshot의 모든 관절 범위 검증은 유지한다. 일반 Landing/Stand 오인식 방지, 실제 수동 측정값에서 양쪽 앞 J2의 일방향 연속 펼치기 및 일반 Landing 경로 미호출 테스트 포함, 펌웨어28개 통과.

V29 169828바이트, SHA256 `853498192797794f106e4d760a0b26d8a82e86905f55f58552b3d634667ca40e` 설치 검증 완료. 실제 Landing 한 번 실행 결과:

```
STOW_TRACK id=2 elapsed=760 target=850 actual=328
ERROR: final position verification failed; servo=2, bus=ok, servo_error=0x00
```

시작 ID2 위치858, 목표변화8틱에 비해 실측변화530틱. 한 바퀴 경계에 도달하기 전의 실패이므로 단순 경계 초과만으로 설명할 수 없다. 보호로 토크 해제 후 조회:

```
STOW_RAW id=2 word=860 raw=860 goal=850 mode=0 res=1 min=0 max=0 torque=0 hw=0
STOW_RAW id=5 word=3158 raw=3158 goal=3169 mode=0 res=1 min=0 max=0 torque=0 hw=0
```

실제 위치 추종 문제는 미해결. 동일 큰 동작을 반복하지 않고 위치모드/다회전 설정 및 실제 제어 응답을 조사해야 한다.

## V30 위치 유지 비교

`stowholdcheck ID normal|extended`는 모두 토크OFF인지 먼저 확인하고 앞 J2 한 축에만 현재 위치 유지 목표를 전송한다. 속도 레지스터40, 최대400ms, 10ms 간격 측정, 12틱 초과 편차에서 해제. 검사 후 원래 각도 제한을 복원/읽기 확인한다. 이는 구동 검사이며 읽기 전용 명령이 아니다.

```
STOW_HOLD id=2 mode=normal target=860 peak_actual=847 elapsed=270
STOW_HOLD id=2 mode=extended target=861 peak_actual=848 elapsed=260
```

둘 다13틱 편차로 중단/토크해제. 이 작은 편차의 중단만으로 토크 부족을 확정하거나 배제할 수 없다. 다음 검사는 누적 목표 좌표 기준 비교다.

원시 설정 ID2/5 공통(주소0부터): `03 0A 00 09 0B [ID] 00 00 01 00 00 00 00 50 A0 3C E8 03 0C 2D 2D 20 20 00 00 00 00 00 36 01 01 00 00 00 14 C8 50 0A FA C8`. 위치 보정31/32=0, 위치모드33=0, 해상도30=1.

제조사 라이브러리의 signed magnitude BIT15 위치 인코딩과 현재 패킷 구성을 비교했다: https://raw.githubusercontent.com/ftservo/FTServo_Arduino/main/src/SMS_STS.cpp . 입력 방향을 임의로 반전하지 않았다.

## V31 원인 확인 / V32 수정 및 설치

누적 목표로 위치 유지 검사:

```
STOW_HOLD id=2 mode=canonical target=861 peak_actual=861 elapsed=0
OK
STOW_HOLD id=5 mode=canonical target=3159 peak_actual=3159 elapsed=0
OK
```

위 출력의 target은 비교용 시작 피드백이다. 실제 전송 목표는 canonical 계산 결과(약4957 및−937)다. 두 축 모두400ms 관측 편차0. 읽힌 피드백만으로 누적 목표 원점을 추정한 이전 구현이 원인임을 확인했다. 하드웨어 특성과 구현 기준은 `STS3250-POSITION-CONTROL.md` 참조.

V32 변경:
- 동작 전 앞 J2 각각의 누적 명령 원점을 짧은 위치 유지로 검증. 이전 기준/0/±4096 후보를 중복 없이 확인하며, 잘못된 후보로12틱 이상 움직이면 해당 축을 해제한다. 검증 실패 시 전체 펼치기를 시작하지 않는다.
- 검증된 기준을 보존하고, 일반 위치 조회가 단회전 피드백으로 이를 덮어쓰지 않도록 수정.
- 앞 J2 목표/실측 오차를 순환 좌표로 비교해 경계에서4096틱의 가짜 오차가 생기지 않도록 수정. 비앞J2 및 안전 기준은 유지.
- Stop은 단회전 원시값 대신 검증된 누적 좌표를 사용. Landing 종료 시 실제 명령 원점에 따라 일반 보행 좌표와 제한 복원을 처리.
- 테스트 서보에 내부 누적 위치, modulo 피드백, 잘못된 유지 목표의 작은 움직임을 구현. 수납 왕복, MCU/서보 재부팅 차이, 일반 읽기 후 목표 원점 보존 회귀 포함. 펌웨어28개 통과.

V32 이미지173584바이트, SHA256 `e8858cbfb4ed3be35533d930562e1459ea075b68a1fa1c077d9135cd95ca14ef`. BLE OTA 이미지검증/플래시/재부팅 성공.

설치 직후 syncstate 조회는 BLE device not found로 실패. 앱 연결 해제 요청 후 실제 Landing 전체 동작 검증 대기. 설치 성공과 전체 실기 동작 검증 완료를 구분한다.

## V32 실제 펼치기 및 V33 완료 처리

연결 해제 후 V32 `syncstate` 확인. Landing 실행은 기존760ms 추종 오류를 넘어 앞다리를 펼쳤으며 사용자가 “동작한다”고 확인했다. 다만 최종 결과는 `position limit`이었다. 이후 조회에서 앞 J2 ID2=1620(목표1622 부근), ID5=2190 등 토크 해제 후 자세 변화가 관측됐다.

V33에서는 성공적인 펼치기 끝에서 전체 토크를 해제하고 모드 제한을 복원하던 단계를 제거한다. Landing 도달 검사 후 현재 위치를 유지한다. 임시 다회전 설정과 검증된 명령 기준은 유지하며 일반 API의 소프트웨어 각도 한계도 유지한다. 정상 펼치기에서 `robot_relax`가 호출되지 않고 모든 토크가 유지되는 회귀 추가. 펌웨어28개 통과.

V33 빌드173264바이트, SHA256 `25a7c7366e7a8cda46c28087a1dc40abc1ffff7dd40e98bb5379ec62ef7a1bca`. 설치와 전체 왕복 검증 결과는 아래에 추가한다.

## 최종 V34 실기 왕복 성공

V33 수납 시험은 수납 전 일반 Landing 준비에서 완료 검증 실패로 중단됐다. 기존 HEAD의 `ROBOT_VERIFY_TOLERANCE=120`과 달리 느린 자세 전환 코드에서24틱을 하드코딩한 회귀를 확인했다. V34에서 기존120틱 기준을 재사용하도록 복원했다. Stow 추종 보호512틱, 정지 원점 검사12틱 및 과부하 보호는 완화하지 않았다.

V34 이미지173264바이트, SHA256 `129af7e5a6650517f88f75be5d11cb959a6e37115efe0d18279e4a21191401bc`를 실제 STM32에 BLE 설치하고 검증/재부팅 완료. 펌웨어 테스트28개 통과(원점 실패 시 전체 동작 차단 추가 회귀도 통과).

실제 로봇에서 연속 실행 결과:

```
> stow
OK stow
> landing
OK landing
> syncstate
$SPOTSTATE pose=landing error=10 torque=on safety=ok balance=full rev=shared-locomotion-v34 caps=trot5,gaitprofiles,balancecontrol,commandretry,stow,headinghold profile=cruise heading=on reverse_limit=1000 recovery=new-command fault_code=0
```

최대 자세 오차10틱=약0.879도, Landing 토크 유지 및 fault_code=0 확인. 보행 정책은 업데이트 전 사용하던jointsport로 복원한다. 이 결과는 현재 기체/전원에서 실제 수납→펼치기1회 성공을 확인한 것이며 모든 하중·전원 재인가 시나리오의 실기 검증을 의미하지 않는다. 전원/원점 변화 및 Stop 회귀는 호스트 서보 모델로 검증했다.

원시 실행 로그(현재 Mac): `/private/tmp/spot-v34-stow.log`, `/private/tmp/spot-v34-landing.log`, `/private/tmp/spot-v34-final.log`.
