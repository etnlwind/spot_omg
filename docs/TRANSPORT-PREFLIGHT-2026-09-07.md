# 운반 자세 / 부팅 landing 사전 확인 — 2026-09-07

> 2026-09-08까지의 시간순 작업 기록이다. 초반 가설은 후속 실기에서 정정됐다.
> 현재 상태와 다음 작업은 [최신 인수인계](HANDOFF-2026-09-08.md)를 먼저 읽는다.

## 사용자 요구

- 뒤로 주저앉아 네 다리를 앞으로 뻗고, 앞다리는 위로 들어 뒤로 눕힌다.
- 부팅 시 실제 관절 위치를 읽어 landing으로 복귀한다.
- J1/J2 위의 전선 때문에 J2가 불필요하게 한 바퀴 더 돌면 안 된다.
- 사용자는 제안한 접기 경로 자체에는 전선 문제가 없다고 설명했다.
  최단 회전 방향으로 임의 대체하거나 360도 모듈러 연산으로 목표를 바꾸지 않는다.

## 실제 로봇 조회

BLE를 통해 continuous-drive-v10의 syncstate와 status를 순차 조회했다.
당시 pose=stand error=12 torque=on safety=ok balance=full이었다.
12개 서보 모두 응답했고 moving=0, hw=0x00, 전압 11.1~11.3V였다.
조회값은 해당 시점의 기록이며 현재 상태로 간주하지 않는다.
이 최초 조회 단계에서는 이동 명령, 펌웨어 플래시, 서보 EEPROM/보정 변경은 하지 않았다.
서보 동작 모드는 이번 조회에서 확인되지 않았다.

```text

=== 2026-09-07T22:40:17 SpotOMG-Bridge ===
# status
Servo status (12 configured IDs):
ID 1 pos=1934 speed=0 load=48 voltage=11100mV temp=32C current=0 moving=0 hw=0x00
ID 2 pos=1570 speed=0 load=40 voltage=11300mV temp=29C current=3 moving=0 hw=0x00
ID 3 pos=2998 speed=0 load=80 voltage=11300mV temp=34C current=3 moving=0 hw=0x00
ID 4 pos=2092 speed=0 load=32 voltage=11300mV temp=32C current=0 moving=0 hw=0x00
ID 5 pos=2431 speed=0 load=-24 voltage=11300mV temp=29C current=1 moving=0 hw=0x00
ID 6 pos=1050 speed=0 load=-64 voltage=11300mV temp=35C current=2 moving=0 hw=0x00
ID 7 pos=2101 speed=0 load=-32 voltage=11200mV temp=30C current=1 moving=0 hw=0x00
ID 8 pos=1428 speed=0 load=-16 voltage=11300mV temp=28C current=0 moving=0 hw=0x00
ID 9 pos=2995 speed=0 load=64 voltage=11300mV temp=40C current=7 moving=0 hw=0x00
ID 10 pos=1957 speed=0 load=56 voltage=11300mV temp=33C current=1 moving=0 hw=0x00
ID 11 pos=2770 speed=0 load=24 voltage=11300mV temp=31C current=1 moving=0 hw=0x00
ID 12 pos=1010 speed=0 load=-104 voltage=11200mV temp=43C current=4 moving=0 hw=0x00
Bus read retries: attempts=0 recovered=0 failed=0

```

## 현재 보정값에서 확인한 제한

실제 robot_config.c를 호스트 C 컴파일러로 빌드해 각도 변환을 실행했다.
당시 J2=0은 아래, +90은 앞, +180은 위, +270은 뒤라고 가정했다.
**이 방향 가정은 잘못이었다. 이후 사용자는 실물의 +90이 뒤쪽이라고 확인했다.**
따라서 +180을 넘어 +270으로 접는다는 당시 경로 추론도 실행 기준으로 사용하지 않는다.
아래 수치는 각도-원시 위치 변환 검사 결과이며 물리적 전방 방향의 증거가 아니다.
최종 운반 각도는 아직 정하지 않았다.

```text
ID 2 current-frame interval -177.36..182.55 deg
  +40 deg: accepted raw=1622
  +90 deg: accepted raw=1053
  +150 deg: accepted raw=370
  +180 deg: accepted raw=29
  +190 deg: REFUSED raw=65535
  +200 deg: REFUSED raw=65535
  +225 deg: REFUSED raw=65535
  +270 deg: REFUSED raw=65535
  -90 deg: accepted raw=3101
ID 5 current-frame interval -168.93..190.99 deg
  +40 deg: accepted raw=2377
  +90 deg: accepted raw=2946
  +150 deg: accepted raw=3629
  +180 deg: accepted raw=3970
  +190 deg: accepted raw=4084
  +200 deg: REFUSED raw=65535
  +225 deg: REFUSED raw=65535
  +270 deg: REFUSED raw=65535
  -90 deg: accepted raw=898
ID 8 current-frame interval -189.23..170.68 deg
  +40 deg: accepted raw=1487
  +90 deg: accepted raw=918
  +150 deg: accepted raw=235
  +180 deg: REFUSED raw=65535
  +190 deg: REFUSED raw=65535
  +200 deg: REFUSED raw=65535
  +225 deg: REFUSED raw=65535
  +270 deg: REFUSED raw=65535
  -90 deg: accepted raw=2966
ID 11 current-frame interval -198.19..161.72 deg
  +40 deg: accepted raw=2710
  +90 deg: accepted raw=3279
  +150 deg: accepted raw=3962
  +180 deg: REFUSED raw=65535
  +190 deg: REFUSED raw=65535
  +200 deg: REFUSED raw=65535
  +225 deg: REFUSED raw=65535
  +270 deg: REFUSED raw=65535
  -90 deg: accepted raw=1231
PASS: +270 is rejected; it is not silently replaced with -90.

```

왼쪽 앞 J2(ID 2)는 +182.55도, 오른쪽 앞 J2(ID 5)는 +190.99도까지
현재 원시 위치 범위 안에서 표현된다. 현 보정으로 뒤로 충분히 눕히는 목표는
범위를 벗어나며, 기존 변환 함수는 이를 거부한다.
+270을 -90으로 바꾸어 보내면 같은 종점이더라도 물리 이동 경로가 달라진다.
이를 해결하기 위해 wrap, clamp, 다회전 모드 전환을 임의로 적용하면 안 된다.

## 다음 구현의 선행 조건

1. 앞다리 J2의 실제 operating mode, 위치 보정/각도 제한 레지스터를 확인한다.
   현재 BLE console의 read/status는 상태를 읽지만 mode/보정값은 제공하지 않는다.
2. 몸통을 지지한 상태에서, 요구하는 전체 접기/펼치기 경로가 한 위치 구간에
   들어가도록 위치 기준 재설정 방법을 검증한다. 기존 설정을 먼저 보존한다.
   물리 이동 없이 좌표만 바꾸더라도 목표 위치/토크 상태를 함께 다루어야 한다.
3. 기준 변경 시 joints.json과 STM32 robot_config.c의 보정을 함께 갱신한다.
   기존 stand/landing/보행이 물리적으로 같은 자세를 가리키는지 검증한다.
4. 중간 자세, 작은 목표 증분, 단계별 피드백/정지 처리를 구현하고
   추가 회전 및 위치 경계 통과를 거부하는 시험을 수행한다.
5. 운반 자세와 검증된 중간 자세에서의 부팅 landing을 시험한다.
   전원 차단 중 손으로 한 바퀴 돌린 이력은 단일 회전 위치값만으로 알 수 없으므로
   이를 자동 판별한다고 보고하면 안 된다.

## 참고

- 펌웨어 위치 변환: firmware/stm32-learning/Src/robot_config.c
- 기존 landing 경로: firmware/stm32-learning/Src/robot.c의 robot_move_to_pose
- STS3250 제조사 자료: https://www.feetechrc.com/en/562636.html
- ST3215 공급사 제어 설명: https://www.waveshare.com/wiki/ST3215_Servo

운반/부팅 복귀 기능은 아직 구현·설치되지 않았으며 실제 접기 시험도 미실시이다.

## 추가 실기 시험: landing → 네 다리 전방 일자

사용자가 범위를 "1. landing, 2. 천천히 네 다리를 앞으로 11자"로 줄여
실행을 요청한 뒤, 앞다리를 뒤로 넘기는 동작 없이 기존 v10으로 시험했다.

- 시작: stand, torque=on, safety=ok. 12개 위치/상태를 개별 확인했다.
- profile은 speed=120, acceleration=5로 낮췄다. 이 저속 설정은 유지했다.
- 단발 landing 명령의 2초 도착 확인 제한을 피하기 위해, 기존 move 명령으로
  약 32tick 이하의 목표 증분을 보내고 매번 실제 도착을 확인했다.
- landing: 14단계 완료. 최종 syncstate pose=landing error=9 확인.
- 전방 일자 목표: J1=0, J2=90, J3=0. 위치 범위를 넘지 않는다.
- 전방 전환 첫 단계에서 ID 3(FL J3)에 target=3417을 보냈으나
  actual=3442, moving=0, load=208, hw=0 상태로 남았다.
  오차 25tick(약 2.20도)이 도착 판정 16tick을 넘은 채 2초가 지나 중단했다.
- 이는 시험 스크립트의 도착 시간 초과이며 펌웨어 safety fault나
  전선 걸림을 확정한 결과가 아니다. 접촉/하중/서보 정지 오차를 구분해야 한다.
- 실패 직후 스크립트가 relax를 쓰고 연결을 닫았으나, 이후 조회는 torque=on이었다.
  최초 쓰기만으로 토크 해제가 완료됐다고 간주하지 않았다.
  별도의 relax 명령을 보내 OK 응답을 확인했다.
- 네 다리를 앞으로 펴는 단계는 완료되지 않았다. 펌웨어/보정값은 변경하지 않았다.
- 상세 기록: tools/servo_tool/logs/transport_forward_trial_20260907.jsonl
- 최종 재조회: `pose=landing error=36 torque=off safety=ok balance=full`.

## 수동 변경 후 landing 재요청

사용자가 직접 다리를 건드렸다고 알리고 landing만 다시 요청했다.
첫 조회는 custom error=255, torque=off, safety=ok였다.
현재 위치에서 점진 복귀를 시도했으나 ID 1이 목표와 18tick 차이에서 정지해
16tick 도착 판정에 걸렸다. load=152, current=9, hw=0이었다.
도착 허용치를 24tick으로 수정해 재시도했으나 전체 위치 검사에서
30tick 변위가 확인되어 중단하고 relax의 OK 응답을 받았다.

일부 관절은 이동할 필요가 없어 torque=off인 채 남았으므로 수동 변위 가능성이
있었다. 전체 현재 위치 hold를 먼저 수행하도록 임시 실행기를 수정했으나,
해당 실행은 자동 승인 검토가 거절했다. 반복된 추종 실패, 위치 변위,
부하/온도 관측으로 물리 동작이 불확실하므로 실물 점검이 선행돼야 한다는 사유다.
거절된 hold 및 landing 재시도는 실행하지 않았다. landing 복귀는 미완료이다.

상세 기록: tools/servo_tool/logs/transport_landing_retry_20260907.jsonl

## 명시적 승인 후 현재 위치 hold + landing 재시도

사용자가 자동 검토 차단 설명 이후 "승인해"라고 명시적으로 승인했다.
동일한 저속/증분/피드백 제한으로 전체 현재 위치 hold 후 landing을 재시도했다.
6단계 중 5단계를 완료했고 마지막 단계 ID 9(왼쪽 뒷무릎)에서
목표 3443, 실제 3469, moving=0, load=216, current=29, temp=39C,
voltage=11100mV, hw=0으로 26tick(약 2.29도) 오차가 남았다.
24tick 도착 허용치를 넘은 상태가 2초 지속되어 중단하고 relax OK를 확인했다.
이후 도착 허용치를 추가로 늘리거나 움직임을 재시도하지 않았다.
최종 재조회: `pose=landing error=46 torque=off safety=ok`.
펌웨어의 landing 판정 범위에는 들어왔으나 엄격한 단계별 도착 검사는 미완료이며, 토크 해제 상태다.

## 사용자가 확인한 landing을 기준으로 전방 펴기

사용자가 현재 landing 자세에 큰 문제가 없어 보인다며 이 값을 기준으로
진행하라고 요청했다. 해당 실측 12개 위치를 docs/poses/landing-reference-2026-09-07.json에
저장했다. joints.json 또는 펌웨어의 보정 중심은 변경하지 않았다.
현재 위치 hold 후 J1은 그대로 유지하고, 실측 기준 J2 +569tick(약 +50도),
J3 -1479tick(약 -130도)의 논리 방향 이동을 47단계로 나누었다.
모든 목표는 원시 위치 범위 내부이며 모듈러 변환/추가 회전은 없다.

사용자 확인 편차에 맞추어 도착 허용 오차를 48tick(약 4.22도)으로 알리고
실행했다. profile 120/5, 목표 증분 32tick 이하, 실측 대비 목표 차이 80tick 이하,
부하/전압/온도 제한을 사용했다.
3단계까지 완료했다. 단계별 최대 load는 200 → 280 → 360으로 증가했다.
4단계 ID 9에서 target=3363, actual=3416, moving=0, load=432,
current=85, voltage=10900mV, temp=37C, hw=0이 확인됐다.
53tick(약 4.66도) 오차가 남아 2초 도착 시간 제한으로 중단했다.
relax 명령의 OK를 확인했다. 부하도 증가했으므로 추가 허용치 변경/자동 재시도는
하지 않았다. 네 다리 전방 일자 자세는 아직 도달하지 못했다.
바닥 접촉, 몸통 지지, 기구/전선 간섭 등 실물 상태를 확인해야 한다.
상세 기록: tools/servo_tool/logs/transport_accepted_forward_20260907.jsonl

## stand만 실행하도록 범위 축소

사용자가 몸통을 들자 다리가 움직였다고 보고했고, 먼저 stand만 요청했다.
손으로 자세가 바뀌어 최초 실행은 새 위치 조회 후 이동 전에 거부했다.
해당 최신 실측값으로 경로를 갱신하고 현재 위치 hold 후 기본 stand
(J1=0/J2=45/J3=90)로 저속 18단계 복귀를 시도했다.
3단계 완료 후 4단계 ID 9에서 target=3394, actual=3448, moving=0,
load=440, current=85, voltage=10900mV, temp=37C, hw=0이었다.
54tick 오차가 2초 유지되어 중단하고 relax OK를 확인했다.
stand는 미도달이며 전방 펴기는 실행하지 않았다.
다음 시도에는 몸통 하중을 받친 상태에서 원인을 확인해야 한다.
상세 기록: tools/servo_tool/logs/transport_stand_only_20260907.jsonl

## 원래 stand 경로와 비교 및 복원

사용자가 잘못 동작했다며 원래 stand 명령을 참고하라고 지시했다.
원래 BLE/iOS stand는 robot_stand → robot_move_to_pose → robot_hold 후
12개 목표를 sts3215_sync_positions로 한 번에 송신한다.
반면 이번 임시 스크립트는 move ID TARGET를 순차 실행하면서 관절마다
도착을 기다렸고, 프로파일을 원래 3400/254에서 120/5로 바꾸었다.
원래 도착 판정은 120tick/2초이나 스크립트는 16→24→48tick 판정을 썼다.
이는 같은 목표라도 물리적 지지와 이동 과정을 변경한다. 임시 절차의
실패를 원래 stand 실패나 기구 한계로 확정해서는 안 된다.

첫 실기 preflight 로그에서 원래 profile=3400/254를 확인하고 해당 값으로
복원한 뒤, 기존 spotctl --via ble stand를 한 번 실행했다.
응답: Starting direct synchronized stand move / OK.
펌웨어·보정값은 변경하지 않았다. 전방 펴기는 재시도하지 않았다.
상세 기록: tools/servo_tool/logs/transport_native_stand_20260907.log

최종 기존 stand 조회는 `pose=stand error=19 torque=on safety=ok`였고,
사용자가 "좋았어 잘섰어"라고 실제 정상 기립을 확인했다.

## 새 동시 제어 forward11 준비 (아직 미설치)

사용자는 stand에서 천천히 전방 일자로 펴고, 뒤집히지 않도록 하라고 요청했다.
기존 펌웨어에는 전방 목표를 동시 송신하는 명령이 없어 `forward11`을 추가했다.
기존 stand11(J2=0/J3=0)은 아래로 펴는 자세이므로 대신 실행하지 않았다.

- 기존 stand/landing 및 보정 중심은 유지한다. 부팅 자동 이동도 추가하지 않았다.
- stand 근처에서만 시작해 J1 실제 위치 유지 → J2 전방 90도까지 8초 →
  J3 0도까지 16초, 20ms마다 12관절 Sync Write를 사용한다.
- 각도 wrap 없이 인코더 범위 안에서 보간하며 지연 후 큰 목표 도약도 하지 않는다.
- roll/pitch 15도 초과, IMU 오류, Ctrl+C는 현재 위치 hold 후 종료한다.
  BNO086은 forward11 실행 중 500ms 이상 오래된 자세를 거부한다.
- 기존 stall/과열/하드웨어 오류 감시를 사용한다. 서보 통신/위치/도착 오류는
  torque off하며, 자동 stand 복귀나 동작 재시도는 하지 않는다.
- 실제 새 제어 함수를 모의 버스/시계로 실행하는 시험을 추가했다.
  전체 suite는 sandbox에서 174 passed, 25 subtests passed였고 localhost bind
  제한으로 실패한 TCP 시험 2개는 sandbox 밖 재실행에서 통과했다
  (test_transport.py 4 passed). 총 176개 시험과 25개 subtest가 통과했다.
- ARM 전체 재빌드/링크/OTA 벡터 및 크기 검증 완료, 빌드 경고 없음.
- 아직 로봇에 설치하거나 새 forward11을 실행하지 않았다.
- OTA는 토크 해제/재부팅을 포함하므로 사용자의 몸통 지지 준비 확인 후 진행한다.

빌드 아티팩트:
```json
{
  "revision": "forward11-v11",
  "binary": "/private/tmp/spot-forward11-build/forward11-v11.bin",
  "size": 134360,
  "sha256": "39f48b6cadbe4ff2f7444c342b588c3d88f3af419d816353b645b13dcada8128",
  "initial_sp": "0x2001fff0",
  "reset_vector": "0x8028f09"
}
```

사용자가 몸통을 받침으로 지지했다고 확인하고 설치/시험 진행을 승인했다.
설치 전 HEAD e321456의 기존 제어 소스로 v10 복귀용 이미지를 별도 빌드했다.
이는 기기에서 읽어낸 원본 이미지가 아니라 소스 재빌드이며 과거 배포 해시와
다르다. 새 이미지와 복귀용 이미지 모두 MCU 벡터/크기 검증 및 무경고 빌드 완료.
복귀용 아티팩트:
```json
{
  "revision": "continuous-drive-v10",
  "binary": "/private/tmp/spot-v10-rollback-build/continuous-drive-v10.bin",
  "size": 131592,
  "sha256": "d423ff591e7ebf4af01b83c4d13cbddd657ccee80d62efea995b28345858cde0",
  "initial_sp": "0x2001fff0",
  "reset_vector": "0x8028599"
}
```
forward11-v11의 BLE OTA를 시작했다. 완료 여부는 아래 후속 기록을 확인한다.

## forward11-v11 설치 및 동시 펴기 도착 확인 — 실물 방향은 뒤쪽

- BLE OTA staging/flash/검증/재부팅 완료. syncstate에서 rev=forward11-v11 확인.
- 재부팅 직후 custom error=408 torque=off safety=ok였다.
- IMU dry-run은 Roll=-3.5°, Pitch=0.6°였다.
- 기존 stand 명령 한 번 실행 → OK.
- `console send forward11`을 35초 timeout으로 한 번 실행 →
  "hips 8s, knees 16s" 시작 안내 후 OK. 자동 재시도 없음.
- 완료 후 12개 서보 모두 moving=0, hw=0x00, 버스 재시도 0.
- J2 실측(FL/FR/RL/RR): 89.74/89.65/88.42/89.74도.
- J3 실측(FL/FR/RL/RR): 0.44/0.44/4.04/0.18도.
- 완료 후 IMU Roll=-3.0°, Pitch=0.7°. 이 조회 직후 사용자는 뒤로 뻗었다고
  확인했다(아래 방향 정정 참조).
- 전체 도착 조회에서 전압 10.3~10.9V, ID 9 load=376, current=106,
  temp=57C로 다른 관절보다 높아 해당 관절을 별도 재조회했다.
- 성공 종료 시 토크를 유지한다. 전방 자세 뒤 자동 stand/landing 복귀는 없다.
- 저장 기록: tools/servo_tool/logs/forward11_*_20260907.log.
ID 9 후속 재조회: pos=1968(보정 기준 J3 약 0.35도), moving=0, load=40, current=1, voltage=10900mV, temp=58C, hw=0. 최초 조회보다 위치 오차와 부하는 줄었다.

## 사용자 관찰로 전방 방향 정정: forward11-v12

사용자는 v11 완료 자세가 앞이 아니라 뒤로 뻗은 자세라고 확인했다.
따라서 v11의 OK는 명령한 위치에 도착했다는 뜻이며, 요청한 전방 자세의
실기 성공으로 해석하면 안 된다. 앞서 모델 설명을 실제 전방으로 가정한 것이
잘못이었다. 기존 stand로 먼저 복귀했고 OK를 확인했다.

v12에서는 stand의 J2 +45도에서 -90도로 반대 방향으로 이동하고,
J3 90도에서 0도로 동시에 펴도록 변경했다. J1은 시작 위치를 유지한다.
원래 v11의 무릎 고정 단계를 반대로 적용하면 아래 링크가 고관절 위로
돌아갈 수 있어, J2/J3를 24초 동안 함께 보간한다. 모든 원시 위치 목표는
같은 인코더 구간 안에 있으며 +270도 치환이나 추가 회전은 없다.
기존 보정값/보행 정책/stand 명령은 변경하지 않았다.

실제 제어 함수 시험 및 전체 회귀 시험 통과, ARM 재빌드 경고 없음.
아직 아래 이미지의 설치/실기 결과는 후속 기록을 확인해야 한다.
```json
{
  "revision": "forward11-v12",
  "binary": "/private/tmp/spot-forward12-build/forward11-v12.bin",
  "size": 134180,
  "sha256": "51a78ac3dfd7ca7db6ba5435a9fdbd189109e0e4201e95102caa65545195c5db",
  "initial_sp": "0x2001fff0",
  "reset_vector": "0x8028e59"
}
```

## 빠른 움직임 지적 후 추가 동작 중단

- 사용자가 “뭐야? 왜 이렇게 빨라?”라고 지적했다. v12 전방 명령은 실행하지 않았다.
- v11 자세에서 복귀할 때 기존 `stand`를 기본 프로파일 3400/254로 실행했다.
  느린 전환 요청에 맞게 복귀 속도를 처리하지 못했다. OTA에는 토크 해제도 포함된다.
  사용자가 관찰한 빠른 움직임이 복귀 중인지 토크 해제 중인지는 확인되지 않았다.
- 진행 중이던 v12 OTA는 ESP32 staging 100% 및 이미지 검증 후
  `STM32 bootloader is programming flash`를 출력하고
  `STM32 BLE update response timed out`으로 종료했다. 설치 성공으로 기록하지 않는다.
- 이후 읽기 전용 `syncstate` 한 번도 STM32 console prompt timeout, 수신 0줄로
  실패했다. 현재 실행 버전/자세/토크 상태는 미확인이다.
- 추가 stand/forward11/relax/재플래시 명령은 보내지 않았다. 물리적 정지나 토크
  상태를 확인한 것은 아니다. 통신 복구와 느린 복귀 경로 검토 전 자동 동작 금지.
