# Stand11 → Stand 중 주저앉음: 초기 조사

2026-09-13. 사용자 보고: 전환 도중 토크가 풀린 듯 주저앉음. 아직 실기 로그를 확보하지 못했으므로 발생 원인은 미확정이다. 새 구동·복구·업데이트 명령은 보내지 않았다.

## 현재 소스에서 확인한 경로

- `app_console.c`의 stand → `robot_stand` → `robot_move_to_pose`.
- 느린 전환은 20ms마다 보간 목표를 보내며 speed=300, acceleration=30을 사용한다. Landing → Stand로 인식된 경우만 빠른 예외 경로다.
- 보간 중 Sync Write 실패 또는 서보 샘플링 오류가 반환되면 `robot_relax`로 모든 축 토크 해제를 시도한다.
- 안전 판정에서 지속 추종오차+부하/전류, 하드웨어 오류 또는 과열을 확인하면 `safety_trip`이 12축 토크를 모두 해제한다.
- 완료 확인은 목표 오차120틱 이내를 요구하고, 보간 후2초가 지나도록 도달하지 못하면 토크를 해제하고 VERIFY_ERROR를 반환한다.
- 따라서 사용자가 본 현상은 소프트웨어 전체 토크 해제와 부합하지만, 전원 순간 차단 및 실제 서보 보호와 구분해야 한다. 아직 설치 펌웨어 revision과 현재 소스 일치 여부도 실기로 확인하지 못했다.

## 진단 정보의 빈틈

`finish_mechanical_pose`는 성공했을 때만 전체 기구 진단 스냅샷을 남긴다. 일반 자세 전환의 command/sample은 보행용 jointtrace처럼 충분히 기록되지 않는다. 실패 시 RESULT와 safety record가 남더라도 실패 직전 최소 전압·12축 추종·실제 송신 목표의 연속 기록이 부족할 수 있다. 조회 시 현재 전압만 정상이라는 이유로 순간 전압 강하를 배제할 수 없다.

## 연결 조회 결과

- USB 직렬 장치 목록에 로봇 ST-LINK/URT-2가 없었다.
- `spotctl app status`: 등록 iPhone을 CoreDevice가 찾지 못했다.
- `spotctl --no-app-control --via ble targets`: SpotOMG-Bridge not found.

앱이 연결을 점유하는지, 로봇 전원이 유지되는지 아직 미확정. 전원 종류를 사용자에게 질문했다. 연결 가능해지면 먼저 `log show 64`, `safety`, `syncstate` 순으로 확보하고 필요 시 12축 `read`를 수행한다. 로그 삭제, 복구 또는 새 자세 명령을 먼저 보내지 않는다.

## 원인별 해결 방향 — 아직 구현/설치하지 않음

1. 통신 실패: 이미 적용된 버스 재시도까지 실패했는지 확인하고, 전환 시간축과 통신 지연을 분리한다. 피드백이 신뢰되는 경우의 제한적 위치 유지와 실제 전원 상실을 구분하는 중단 정책을 설계한다.
2. 정상 이동 중 추종 지연: 모터별 실측 속도·가속·하중에 맞춘 궤적과 완료 판정. 보호 임계값을 무작정 늘리지 않는다.
3. 실제 끼임·과열·하드웨어 보호: 해당 축과 오류 비트를 근거로 기구/배선/모터 원인을 해결한다. 억지 토크 유지는 하지 않는다.
4. 전원 순간 저하: 전환 직전/도중 서보 전압·공급 전류·리셋 증거와 커넥터를 확인한다.
5. 진단 개선: 일반 자세 전환에도 송신 목표·실측·전압·전환 단계·종료 사유를 기록하고 실패 직전 데이터를 보존한다.

검증 수준: 코드 경로 확인 완료. 실기 사건 원인 확정·재현·수정 검증은 미완료.

## V49 코드 수정 및 호스트 검증

`pose_supervisor.c`를 추가하고 Stand/Stand11/Landing의 일반 자세 전환에 연결했다. 기존 Stow 전용 경로, 보행·Forward11 오류 처리는 이번 변경의 범위 밖이다. 전체 로봇의 모든 토크 해제 경로를 수정했다고 해석하면 안 된다.

- 일반 자세 전환에서는 통신 실패/시간 초과/자세 이상을 이유로 전체 토크 해제 명령을 보내지 않는다. 초기 토크 활성화 일부 실패 시에도 이미 지지 중인 다른 축을 OFF로 되돌리지 않는다.
- 12축 위치·부하·전류·온도·전압·하드웨어 오류를 조회한다. 상위에서 최대3회 읽기를 시도한다(기존 버스 계층 재시도와는 별도). 완성된 읽기 묶음의 최초 샘플이500ms보다 오래되면 진행하지 않는다.
- 모든 축을 하나의 궤적 단계로 움직이며 실측 위치보다 목표가40틱 이상 앞서지 않도록 단계 증가량을 줄인다. 개별 축 목표를 독립적으로 잘라 대각선 동기를 깨지 않는다.
- 현재 송신 목표와 오차24틱 이내일 때 다음 단계로 진행한다. 이전 코드의 전환 완료 후2초 제한은 제거했다. 목표 방향으로3틱 이상 진행한 이력을 축별로 관리하고, 실측 진행이 계속되면 명목 시간을 넘겨도 계속한다.
- 남은 오차가 있는데1.8초 동안 유의미한 진행이 없으면 중단한다. 부하/전류도 높으면 `obstruction-suspected`, 그렇지 않으면 `no-progress`다. **기계적 끼임과 정상 고하중을 확정 판별하는 접촉 센서는 아니다.**
- 저전압/서보 오류/온도 이상, IMU 조회 불가/기울기 한계도 별도 중단 사유다. BNO086은500ms 이내 자세 보고만 사용하고, BNO055는 기존 동기 읽기를 사용한다. 원시 가속도·각속도 충격 감지는 구현하지 않았다. 15도 자세 제한은 충격 검출기가 아니다.
- 중단 시 새로 읽힌 범위 내 위치를 목표로 보내 위치 오차를 줄이고, 다시 읽어 유지 여부를 확인한다. 지속 통신 실패에는 추측 위치를 보내지 않는다. `hold=0` 미시도, `1` 목표 송신, `2` 재조회 위치 오차 범위 확인, `3` 조회/송신 실패. `2`도 토크 레지스터 검증이나 장시간 지지 안정성 검증을 뜻하지 않는다.
- 버스가 끊기면 서보에 마지막 목표가 남는다. 목표 선행량 제한은 위험을 줄이는 조치이며, 정지나 장애물 힘 제거를 보장하지 않는다. 실제 과열/지지 상실 시 지지 다리 재배치·개별 축 감력 등의 물리 제어는 아직 없다.
- RAM에 최근24개의 최악 추종 축 관측(목표/실측/오차/전압/부하/전류/roll/pitch/시간/판정)을 보관하고 동작 종료 후 플래시 로그로 저장한다. 동작 루프 안에서 플래시를 쓰지 않는다. 모든 축의 연속 고속 파형 기록은 아니며 순간 전압/충격을 완전히 포착하지 못한다.

### 검증 근거

실제 C 루틴에 지연/모터/버스 fake를 연결한 시험: 정상, 명목 시간+2초 초과의 느린 진행, 고부하 정지, 저부하 정지, 지속 읽기 실패, 일시 읽기 실패, 송신 실패, 전압 저하, 과열, 기울기, IMU 누락, Stop, 잘못된 엔코더, 오래된 버스 샘플의14조건. 모든 조건에서 torque-OFF 명령이0회인지 검사했다. 특히 느린 경우에도 완료하고, 실패 시 위치 유지가 성공했는지/불가능했는지 구분했다.

기존 포함 호스트 시험25개 통과. STM32 전체 빌드 통과. BLE 로그 재조회는 여전히 not found였다. **이번 사건의 촉발 조건은 미확정이며, 위 숫자는 실기 학습값이 아니라 보수적 초기 설계값이다.** 실제 장착 인터페이스가 URT-2라는 이전 설명도 사용자가 부정했으므로 철회했다.

실기 설치·구동 검증은 수행하지 않았다. 먼저 기체를 지지한 상태에서 실측 기록으로 임계값과 유지 처리, 시간과 동기, 전원 상태를 검증해야 한다. 정상 하중 지연과 위험한 접촉을 데이터로 완전히 구분한다고 주장하지 않는다.

### 현재 실물 인터페이스 정정

사용자가 Waveshare Bus Servo Adapter (A) 제품 링크로 현재 보드를 확인했다. URT-2가 아니다. 통신 오류는 STM32의 서보 버스 송수신 결과이며, 어댑터의 독립적인 상태 응답을 읽은 것이라고 해석하지 않는다. 목표 Sync Write는 모터별 수신·도달 확인을 대신하지 않는다. 어댑터/배선/서보 중 어디서 응답이 유실되는지는 별도 진단이 필요하다.

최종 V49 빌드: 249680 bytes, SHA256 `6e0d5a5ae5047d9c524390826eddcfdc5247ffb11bbf62691d6c600e2cf76ca9`. 파일은 `/private/tmp/spot-pose-supervisor-v49-20260913/shared-locomotion-v49.bin`. 실물 전송하지 않음.

## 실물 설치 결과

사용자 재시도 요청 후 BLE 연결 성공. 기존 펌웨어에서 Landing이 `OK`, `$MECHLOG label=landing complete=1`로 완료되고 상태 확인 후 업데이트 진행. 249680바이트 V49 이미지를 ESP32에100% 전송하고, STM32 플래시100% 및 `STM32 firmware verified and rebooted` 응답을 확인했다. 배포 이미지는 `artifacts/firmware/2026-09-13-v49/shared-locomotion-v49.bin`에 보존했다.

첫 재부팅 후 `syncstate` 연결 시도는 BLE not found였다. 설치 프로토콜의 검증·재부팅 성공과 독립적인 실행 버전 조회는 구분한다. 새 펌웨어 Stand11→Stand 실기 동작 시험은 아직 하지 않았다.

## 재접속 후 실제 로그와 V49 첫 시험

Mac 앱 연결 해제 후 `syncstate`에서 V49 실행을 확인. 12축 응답 정상, 정지 전압11.5~11.7V, 온도31~36C, hw=0, torque=off. `artifacts/audits/pose-supervisor-v49/preflight.log`와 `history.log`에 원본을 저장했다.

과거 boot88의 Stand11(19279)과 Stand(19332)은 각각 RESULT ok로 종료했다. 후자는13085ms 소요되었고 해당 구간에서 오류/토크 해제 사유는 확인되지 않았다. **사용자가 보고한 사건이 timeout으로 발생했다고 단정할 근거는 여전히 없다.** 옛 코드의 전체 토크 해제 결함 발견과 이번 사건의 실제 원인은 구분한다.

V49 Landing 실제 시험: `POSE landing reason=no-progress elapsed=2990 nominal=1500 retries=0 waits=43 hold=2`. ID9/RL-J3의 실측3478, 중간목표3452, 오차26틱이 약1초 이상 고정. 부하216, current_raw8~70, 11.3~11.6V, roll=-3.2도/pitch0.1도. 최종 `pose=landing error=53 torque=on safety=ok` 확인. 이번 중단은 전체 토크를 끊지 않았고, 재조회 위치 유지를 확인했으나 **24틱 진행 장벽이 작은 정적 잔여 오차와 교착을 만든 것**이 확인됐다. 이 데이터만으로 마찰·데드밴드·하중 중 기계적 원인을 확정하지 않는다.

## V50 보완

40틱 선행 한도 안에서 공통 위상을 증가시키되, 24틱 이내라는 별도 장벽은 다음 목표 진행을 막지 않도록 했다. 종료 허용은 두 수준이다: 기존 엄격 도달24틱, 또는 최종 명령까지 도달한 뒤 모든 축이40틱 이내이고 각 연속 샘플 위치 변화2틱 이내·부하/전류가 보호 기준 미만인 상태를500ms 유지한 경우 `complete-residual`. 전압·IMU·서보 오류 검사는 계속 적용한다. 잔여 오차 완료를 정확한 목표 도달로 표현하지 않는다.

실측26틱의 잔여 오차를 유지하는 모터 fake 조건을 추가했다. 총15가지 실행 시나리오와 기존 포함 호스트 시험25개 통과. STM32 V50 빌드250136bytes, SHA256 `cdd1b43f135a6e619dd698b5205279d6f3f22f1440f72bf039432e453ae4721b`. 이 수정은 무조건 시간이나 안전 한계를 늘리는 것이 아니라, 같은40틱 목표 한도 안에서 중간 진행과 잔여 오차 종료를 분리한다.

## V50 시험과 V51 보완

V50 설치/재부팅 후 실행 버전 확인. Landing1217ms에서 `servo-fault`로 중단, ID4가 판정 축이었다. 중단 후 상태는 `pose=landing error=30 torque=on safety=ok`, `hold=2`. 즉 토크를 끊지 않았다. 직후 ID4 조회는11.6V/34C/hw0이었다. V50 관측 링에 온도/hw 필드가 없어서 당시 값의 원인을 확정할 수 없었다. 또한 기존 `print_safety_fault`가 실제 safety latch 없이 `TORQUE_OFF_ALL`을 출력하는 표시 결함이 있었다. 원본은 `artifacts/audits/pose-supervisor-v50/sequence.log`, `sequence-flight.log`.

V51: 이상 온도/hw 값을 읽으면 목표 진행 없이15ms 후 같은 축을 재조회하여 재현 여부를 확인한다. 최초 이상값과 축, 정상으로 복구된 횟수를 `POSE_HEALTH`에 기록한다. 지속 이상은 이전처럼 자세 진행을 중단하고 신선한 위치 유지로 전환한다. 실패 상태 기록 없이 실제로 실행하지 않은 전체 토크 해제 메시지를 표시하던 경로를 분리했다. 단발150C 샘플 후 정상34~35C로 복구하는 호스트 시험을 추가했고, 기존 지속 과열 중단도 계속 통과한다.

호스트 시험25개/자세 실행16조건 통과. V51 빌드250760bytes, SHA256 `f7320ae1cfab8ba1e79762a72a6876e51b7e65cf5243311381de74cdbec2c63e`. 이번 중단을 실제 과열 또는 통신 불량이라고 단정하지 않는다. 지속 하중·충격을 완전히 판별한다는 주장도 하지 않는다.

## V53 — bounded starting-effort probe and terminal errors

V52 Stand failed at 2186ms: RL-J3 actual3387/command3347 (40 ticks), load328, current67–92 raw, voltage11.0V. This proves saturation of the host command envelope, not the physical cause of the stopped servo. Front J2 origin readback remained preserved. Current idle readback all12 replies, 11.2–11.4V,31–39C,no hardware faults.

V53 retains a 40-tick normal envelope. After300ms without progress at >=36ticks error, healthy low-effort feedback permits one expansion to60ticks per axis per command. It retains the original1800ms no-progress deadline and24/40tick completion criteria; no goal overshoot, EEPROM/PID changes or torque-off. `POSE_PROBE` logs count and axis mask. A low-effort obstruction is still possible; the extra20ticks is a bounded diagnostic allowance, not a contact classifier. New `servoconfig ID` reads registers0..49 without writes.

Host regression25tests passed; ARM build252840bytes, SHA256 e83bbded3d699085982a6a09d4c7c82751c0152dfc88af9e5f733faf0da8187c. Physical outcome recorded separately after deployment.

App main terminal: errors are colored red per line using attributed text. Normal position-error telemetry remains green. iPhone Debug build passed. Installation/readback separate from visual UI verification.

V53 real follow-up: servo3 and9 register0..39 snapshots match except ID (saved in artifacts/audits/pose-supervisor-v53/stand.log); no uniquely misconfigured RL-J3 identified. After STM32-only reboot, ordinary Landing failed at1963ms: front J2 commands1606/2409 while feedback moved away to2167/1858. Command-origin cache had been lost while servo multi-turn mode remained active. This invalidates treating this trial as an effort-probe test: Stand was not attempted.

V54 fixes this second software regression by requiring the existing bounded physical origin-reference procedure for front J2 before any supervised ordinary pose after cache loss. Verified origins are reused. Added host regression for normal Landing geometry with -4096/+4096 internal origins and modulo feedback after MCU reboot; all25host tests pass. This reference procedure can briefly release an individual front hip while checking its origin, as in the existing Stow implementation; it is not a universally supported-pose calibration. No further ordinary movement under V53 is attempted before deployment.

iPhone error-color app installation confirmed after device unlock. Visual screenshot verification not performed.

V54 physical: reference restored; no reverse front-hip drift in ordinary Landing. Landing stopped at7431ms although RL-J3 was only27ticks from final target with load224. The per-axis no-progress check ran before common-path residual completion; a near-arrived axis was incorrectly penalized while the common phase finished. Stand then stopped at3850ms: RL/RR knee errors55/58ticks, loads448/472, voltage10.8–11.0V. Thus the60tick trial did not solve the envelope stall; it is not reported as success.

V55: one-shot effort envelope is120ticks (10.55deg), not a relaxed arrival criterion; 24tick exact/40tick stable residual arrival stays unchanged. Axes already within40ticks of final target with low effort no longer trip no-progress while other axes finish. Added early-arrival residual/slow-other-axes regression. This is a bounded position-control change, not torque/contact control; battery sag remains a possible contributor. 25host tests pass. ARM253208bytes SHA256 c44983205c17f0eb79f7cc553e7601a5e72b97573f91ea227ee6400d90f19f07.

V55実機/실기: OTA 검증/재부팅/버전 확인 완료. Landing `complete-residual`2217ms, Stand `complete-residual`6225ms로 모두 OK. Stand에서 probe mask0x900(IDs9,12) 사용. 최종 pose=stand,error=32,torque=on,safety=ok. 12개 설정/상태 조회 및 통신 재시도0 확인. 이는 실제 Landing→Stand 1회 통과이며 정확한0오차·모든하중·Stand11 왕복 검증이 아니다. 원본 `artifacts/audits/pose-supervisor-v55/stand.log`. 단일 이상 온도 샘플은 재조회로 해제됐으며 원인은 별도 미확정이다.

후속 유지 조회: pose=stand,torque=on,safety=ok이나 최대오차가32→67틱(약5.9도)으로 증가했다. 따라서 전환 완료와 장시간 정밀 자세 유지 성공을 구분한다. 하중하 위치 유지 편차는 남아 있으며 숨기지 않는다.
