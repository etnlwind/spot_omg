> 사용자 실행 원칙 (2026-09-10): 앱은 실제 폰 UJIN17에서만 실행합니다. iOS Simulator를 실행하지 마십시오. MuJoCo 로봇 시뮬레이터는 사용합니다.

# SpotOMGController for iOS

## V0.4.8 (20) — 조종기 여백 조정

- 버튼 모음 위쪽 여백을 22pt로 늘리고 조이스틱을 가용 영역 아래쪽에 정렬한다.
- 조이스틱 최대 지름을 250pt로 늘려 하단 빈 공간을 줄인다. 터미널은 유지한다.

## V0.4.7 (19) — 첫 화면 조종기 배치

- 상단 터미널의 크기·위치·입력 기능은 유지한다.
- 터미널 아래 첫 화면은 연결/동기화/IMU 보정/정책/자세 아이콘 버튼, 전압·정책 표시와 조이스틱으로 구성한다.
- 남은 화면 높이에 맞춰 조이스틱 영역을 배치하고 상세 설정은 그 아래 스크롤 영역에 유지한다.
- 아이콘은 짧은 이름, 상태 색상, 접근성 이름을 제공하며 최소 44pt 높이로 누를 수 있다.
- 기존 손 떼기 정지, 설정 변경 시 정지, Relax 확인 동작은 유지한다.
- 실제 폰 대상 Debug 빌드 성공. iOS Simulator는 실행하지 않았다.

## V0.2.0 (3) Debug — V13 개선 보행

- 보행 섹션에 **개선 전진 · 3회 · 844ms** 버튼을 추가했습니다.
- 로봇의 `syncstate`에 `caps=trot5`가 있을 때만 활성화합니다. 이전 펌웨어에 새 명령을
  보내지 않으며 재연결/연결 해제 시 지원 상태를 다시 확인합니다.
- 준비·복귀 동작을 포함한 상태 갱신 지연을 적용했습니다.
- 조이스틱은 기존 연속 전후진/회전 제어입니다. 개선 전진은 별도 버튼에서 실행합니다.
- 표시 버전은 **Spot OMG V0.2.0 (3) - Debug**이며 작은 regular 버전 글꼴을 유지합니다.
- 새 명령/기능 감지와 기존 초기 화면·BLE·조이스틱·전압 표시 XCTest 26개 통과.

새 컴퓨터에서 작업을 재개할 때는
[작업 인수인계](../../../docs/HANDOFF-2026-09-08.md)를 먼저 참고하세요.

SwiftUI와 CoreBluetooth로 `SpotOMG-Bridge`에 직접 연결하는 iPhone 앱입니다.
자세·진단 버튼은 기존 STM32 text console을 사용하고, 조이스틱은 console prompt와
분리된 sequence/heartbeat realtime 제어 lane을 사용합니다.

## 실행

1. Xcode에서 `SpotOMGController.xcodeproj`를 엽니다.
2. Signing & Capabilities에서 개인 Team을 선택합니다.
3. Bluetooth가 가능한 실제 iPhone을 실행 대상으로 선택합니다.
4. 로봇 전원을 켜고 앱을 실행한 뒤 Bluetooth 접근을 허용합니다.

iOS Simulator에서는 실제 BLE peripheral 연결을 검증할 수 없습니다.

### 첫 화면과 연결 시작 순서

앱 모델 생성 시에는 CoreBluetooth와 연결 로그를 초기화하지 않습니다.
터미널·연결·조이스틱 레이아웃을 창에 표시하고 화면 갱신 주기를 지난 뒤,
한 번만 Bluetooth 초기화 → 검색 → 연결 → 상태 동기화를 시작합니다.
로봇 연결이 늦거나 불가능해도 제어 화면은 먼저 표시됩니다.
연결 추적의 `ui-visible` 이벤트는 화면 표시 후 연결 초기화 지점을 기록합니다.
이 변경은 앱 내부 시작 순서를 분리한 것이며, iOS가 앱 프로세스를 시작하기
전의 launch screen 시간이나 실제 기기의 전체 시작 시간을 보장하지 않습니다.

실기 시작 속도 비교에는 Release 빌드를 홈 화면에서 직접 실행합니다.
Release는 Swift `-O`/whole-module 최적화를 사용하고 코드 커버리지 계측과
디버그 dylib를 끕니다. Debug는 `-Onone`이며 디버거 부착 여부는 별도 조건입니다.
설치 직후 첫 실행과 이후 프로세스를 종료한 재실행을 구분해서 측정합니다.

## 현재 기능

- 상단 제목에 `Spot OMG v0.1.0 (2) - Debug` 형식으로 앱 버전·빌드·구성 표시.
  버전 부분은 작은 regular 글꼴이며 연결 영역의 Section 제목은 생략한다.
  연결 항목에는 로봇 펌웨어 revision만 표시해 앱 버전 행의 높이를 줄인다.
  로봇 버전은 syncstate 실측값이며 연결 전에는 "연결 후 확인", 수신 전에는 "확인 중".
- 상태 행에 배터리 전압 추정값 표시: `syncstate` 수신 후 보행 세션이 없을 때
  기존 `read 1`로 서보 전원선 전압을 읽는다. 연결/수동 상태 동기화/동작 후 상태
  조회에 따라 갱신하며 상시 polling은 하지 않는다. 15초가 지난 값은 "이전" 표시,
  연결 해제 또는 새 조회 대기 중에는 `— V`로 표시한다. 셀별 전압이나 배터리 단자
  직접 측정값이 아니며 저전압 차단 기능은 아니다.
- service UUID로 `SpotOMG-Bridge` 검색, 자동 연결과 연결 해제 후 재검색
- 조종 버튼과 같은 화면에서 STM32 console 명령 송신 및 BLE notification 로그 표시
- STM32 `syncstate` snapshot을 이용한 실제 자세·torque·safety·balance 상태 동기화
- 변위 비례 연속 조이스틱: 전진·후진과 좌우 회전을 동시에 혼합하고 gait phase를
  유지한 채 200ms마다 목표를 갱신. 손을 떼면 중앙 복귀와 감속 정지
- Landing/Stand/Stand11/Hold/Recover와 확인 절차가 있는 Relax
- 보수적인 단일-cycle trot4/crab 명령
- targets, scan, gaitdiag, baldiag
- 연결 직후 iPhone 시각을 STM32에 동기화하고 persistent flight log 64개 조회
- 앱이 비활성화될 때 best-effort `stand` 요청

## 연속 조종 프로토콜

```text
drive LINEAR YAW SEQ\n       최초 시작, 각 축 -1000..1000
@D SEQ LINEAR YAW\n          실시간 목표 및 heartbeat 갱신
@S SEQ\n                     감속 정지
```

STM32가 gait phase와 50Hz actuator/IMU 루프를 소유합니다. 앱은 드래그 이벤트마다
보행 command를 재시작하지 않고 최신 벡터만 보냅니다. 200ms heartbeat가 800ms 이상
도착하지 않으면 STM32가 연결 상실로 판단해 독립적으로 정지합니다. sequence가 오래된
지연 packet은 무시됩니다. 긴 console diagnostics와 `# ` prompt는 조종 지속 여부에
관여하지 않습니다.

앱이 background로 전환될 때는 Ctrl+C와 Stand를 요청합니다. BLE가 그 전에 끊기더라도
STM32 watchdog이 최종 정지를 보장합니다.

## 연속 조종 정지·재시작 수정

연속 보행 중 자동 `syncstate`가 일반 명령 경로를 거쳐 `stopDrive()`를 호출할 수
있었던 문제를 수정했습니다. 보행 시작 시 예약된 조회를 취소하고, 실행 중 상태
조회는 정지 명령을 보내지 않고 보행 완료 후 snapshot으로 대신합니다.

STM32의 `# ` prompt는 줄바꿈이 없습니다. 뒤따르는 `$SPOTDRIVE` 또는 `$SPOTSTATE`와
붙거나 BLE notification 경계에서 분할되어도 prompt와 응답을 수신 순서대로 분리해
처리합니다. Swift에서 CRLF는 하나의 Character이므로 LF 문자만 찾던 코드도
`isNewline` 기반으로 수정했습니다. 이전 line parser에서는 CRLF 응답과
`# $SPOTDRIVE stopped ...`를 놓쳐 앱만 계속
보행 중이라고 판단하고 다음 터치에 시작 명령 대신 `@D`를 보내는 문제가 있었습니다.

손을 떼거나 자세 명령을 누르면 `@S`를 한 번 보내고 heartbeat를 중단합니다.
STM32 종료 응답과 diagnostics 뒤 prompt를 확인하기 전에는 같은 세션을 `@D`로
되살리지 않습니다. 완료 후 새 터치 입력이 오면 새 `drive`를 시작합니다.
정지를 요청했거나 STM32가 종료를 알린 뒤 종료 확인이 5초 동안 오지 않으면
Ctrl+C를 요청하고 연결을 해제해
무기한 대기를 방지합니다. 이후 사용자가 다시 연결하며, 이전 보행 입력을 자동으로
재생하지 않습니다. 제스처 취소도 손을 뗀 경우처럼 중앙 복귀·정지 처리합니다.

회귀 시험은 BLE 전송을 대체한 실제 manager에서 예약 조회, heartbeat 유지,
정지 후 재시작, 안전 자세 명령, 응답 유실을 확인하고, parser는 모든 packet
분할 경계와 1문자씩의 수신을 검사합니다. 로봇은 시험 중 움직이지 않습니다.

실기 로그의 `RESULT result=ok`, `lag=0`, `fall=0`는 정상 종료를 나타내지만 정지
입력의 UI 발생 원인까지 기록하지는 않습니다. v10의 `@D/@S`는 ISR에서 처리되어
persistent command log에 남지 않으므로 해당 기록의 부재를 heartbeat 유실의
증거로 해석하면 안 됩니다. 이 수정은 iOS 앱 업데이트이며 STM32 v10 재플래시는
필요하지 않습니다.

검증: iOS 26.4/iPhone 17 Pro simulator에서 XCTest 15개 통과, CR/LF 사이를
포함한 byte 경계 확장 시험도 통과했습니다. iPhone용 서명 빌드도 성공했습니다.
앞서 CoreSimulator runtime을 찾지 못했던 오류는 샌드박스 밖에서 정상 접근해
해결했으며, 실제 BLE 보행의 지속·정지·재시작은 기기에서 확인해야 합니다.

### 시작 알림 유실에 따른 앱의 강제 연결 해제 수정

종료 확인용 5초 제한을 보행 시작 시에도 등록한 첫 수정에는 회귀가 있었습니다.
`$SPOTDRIVE started` notification이 유실되면 조이스틱 heartbeat가 전송되는
중에도 앱이 5초 뒤 Ctrl+C와 연결 해제를 실행했습니다. 시작 알림을 생략한
회귀 시험에서 이 현상을 재현했습니다.

시작 시 timeout 등록을 제거하고, timeout은 명시적 정지 요청 또는 STM32 종료
응답 뒤에만 동작하도록 제한했습니다. BLE 연결 상실 시의 STM32 800ms watchdog,
손을 뗄 때의 `@S`, 앱 비활성화 시 Ctrl+C는 유지합니다. 시작 알림의 수신 여부는
지속 보행의 조건이 아닙니다. 앱 터미널에는 정지 요청 sequence를 기록하고 timeout
문구는 '보행 종료 확인 시간 초과'로 구분합니다.

### 전송 순서와 실기 연결 추적

- 보행 phase는 `idle → controlling → stopping → draining → idle`로 관리합니다.
  오류 응답으로 시작이 거부된 경우에도 heartbeat를 중단하고 prompt에서 세션을
  닫습니다. 재연결 시 이전 입력과 전송 대기열을 초기화합니다.
- GATT `withResponse` 전송은 `didWriteValueFor`가 확인된 뒤 다음 chunk를 보냅니다.
  한 명령의 UART 줄이 끝나기 전에 다른 명령의 chunk를 끼워 넣지 않습니다.
- 아직 전송하지 않은 `@D`는 최신 목표 하나만 유지합니다. 정지/interrupt 요청은
  대기 중인 입력을 제거합니다. 일반 명령 대기열은 최대 16개이며 쓰기 ACK가 2초
  동안 오지 않거나 오류가 발생하면 연결을 정리합니다. 이는 console notification
  유실과 구분되는 전송 실패입니다.
- iPhone의 `Library/Caches/RobotConnection/current.jsonl`에 시각·uptime과
  enqueue/write/ACK/RX, 제스처 정지 원인, 앱 비활성화, phase, 연결 해제 원인을
  비동기로 기록합니다. 512KiB 파일 두 개만 유지하고 앱이 외부에 자동 전송하지
  않습니다. iOS가 cache를 삭제할 수 있으므로 장기 flight log는 STM32 기록을
  사용합니다.

검증 결과: 시작 알림을 누락시키면 기존 구현이 `disconnected`가 되는 실패를
재현한 뒤 수정했으며, 전송 순서·coalescing·명령 거부·정지·재시작을 포함한
XCTest 21개가 iOS simulator에서 통과했습니다. 시뮬레이터 앱 컨테이너에 JSONL
파일이 생성되는 것도 확인했습니다. 실제 iPhone용 서명 빌드는 성공했고, 실기
지속 보행 확인은 앱 업데이트 후 별도로 진행합니다.

개발 연결된 iPhone에서 실제 앱 기록을 가져오는 예:

```bash
xcrun devicectl device copy from --device UJIN17 \
  --domain-type appDataContainer --domain-identifier com.etnlwind.spotomg.controller \
  --source Library/Caches/RobotConnection/current.jsonl \
  --destination /tmp/spot-ios-connection.jsonl
```

### V0.2.0 (4): 연결 후 버전 확인 복구

실제 iPhone 로그에서 `syncstate` 쓰기 ACK는 있으나 RX가 없는 상태를 확인했다. V14 문자열 파싱 문제가 아니라 상태 응답 미수신이다. 재연결 시 유지된 알림 구독은 해제 후 재구독하며 notify 상태를 기록한다. 초기 `syncstate`는 2초 간격 최대 3회 읽기 전용 재시도한다. 응답 수신/연결 해제 시 타이머를 취소하고, 보행 중에는 재시도를 보내지 않는다. 끝까지 응답이 없으면 오류와 `응답 없음`을 표시한다.

V14 분할 응답 파싱 및 재시도 테스트 추가. XCTest 28개 중 27개 첫 실행 통과; 기존 5초 종료 타이머 검사 1개가 ready/disconnected 타이밍으로 실패했으며 분리 재실행에서 해당 검사와 V14 검사가 모두 통과했다. iPhone Debug 빌드 및 설치 완료. 설치 후 자동 실행은 iPhone 잠금 상태로 실패하여 실제 수신 복구는 아직 미확인이다. 사용자가 잠금을 해제해 앱을 열면 연결 로그로 확인할 수 있다.

### V0.2.0 (5): 직진 근처 회전 입력 제거

사용자가 직진 시 오른쪽으로 휜다고 보고했다. 최신 완료 세션 전체에서는 초기 `drive 544 398 16` 이후 작은 양수 yaw가 약 5초간 남고, 후반부에 -3%까지 바뀌었다. 마지막 패킷만 보고 좌회전 입력 때문이라고 판단하면 안 된다. 이 기록은 순수 직진 시험이 아니며 실제 기구 비대칭 여부를 확정하지 못한다.

앱은 `abs(x) <= 0.10 * min(1, abs(y))` 범위의 회전 입력을 0으로 만든다. 바깥쪽 입력은 연속적으로 다시 매핑하여 경계에서 갑자기 회전하지 않게 했다. 전진/후진 성분 및 순수 제자리 회전은 유지한다. 강한 대각선 입력을 직진으로 바꾸거나 IMU yaw 방향 고정을 구현한 것은 아니다.

직진 범위·좌우 대칭·후진·경계 연속성·기존 제어를 포함한 XCTest 29개 통과. Debug 서명 빌드 및 iPhone V0.2.0 (5) 설치 완료. 실제 직진 개선은 설치 후 새 보행 기록으로 확인해야 한다. STM32는 V14 유지, FR J1 기존 -2° 보정과 IMU 설정은 변경하지 않았다.

## 실제 / 가상 로봇 선택

제어 화면의 대상 선택에서 실제 BLE 또는 가상 TCP를 선택합니다. 변경 시 기존 세션이 해제되며 새 연결을 눌러야 합니다.
가상 대상은 Mac IP와 8765 포트를 입력합니다. 시뮬레이터 식별을 통과한 연결에만 조작을 허용합니다.
물성과 전압은 추정/가상 표시이며 현재 가상 IMU는 모니터링만 적용됩니다.
자세한 실행 방법과 제어기 공유 범위는 [가상 로봇 문서](../../../docs/VIRTUAL-ROBOT-2026-09-09.md)에 있습니다.

### V0.3.0 (6): 가상 로봇 BLE

실제 BLE, 가상 BLE, 가상 TCP를 선택할 수 있습니다. 가상 BLE는 별도 서비스 UUID로 Mac의
`SpotOMG-Sim`을 검색하고 simulator 식별이 완료되어야 제어를 허용합니다.
MuJoCo 실행 방법은 [가상 BLE 안내](../../../docs/VIRTUAL-BLE-2026-09-09.md)를 참고하십시오.
V0.3.0 (6) Debug를 iPhone에 설치하고 BLE 보행 입력·정지·상태·전압 왕복을 확인했습니다.

### V0.3.1 (7): 가상 로봇 보행 선택

`simprofiles` 기능을 제공하는 가상 로봇 연결에서 5개 보행 정책을 선택할 수 있습니다.
기본은 크루즈이며, 보행 중 선택을 바꾸면 정지 완료 후 적용합니다.
실제 로봇에서는 시뮬레이터 전용 정책 명령을 거부합니다.
[정책별 측정 및 한계](../../../docs/GAIT-PROFILES-2026-09-09.md).

### V0.3.2 (8) — 가상 로봇 수평 보정

가상 로봇의 `simbalance` capability가 있으면 BNO055 수평 보정 ON/OFF를 표시한다. 보행 중 변경은 정지 후 전송한다. `balance=active/suspended/off`를 구분해 표시하며 실제 로봇에는 전용 명령을 보내지 않는다. Debug 기기 빌드 및 설치 완료. 상세 검증은 `docs/ACTIVE-BALANCE-2026-09-09.md` 참조.

### V0.3.3 (9)

기울기/센서 안전 정지 후 같은 터치로 재출발하는 문제를 수정했습니다. 스틱 해제 및 Recover 후 정상 상태 확인이 필요합니다. stopping 중 거부 응답 처리와 watchdog 후 재시작 제어를 포함해 XCTest 40개 통과. 트롯·하이 스텝의 후진 제한을 정책 설명에 표시합니다.

### V0.3.4 (10): 좌우 입력 제자리 회전

대각선 입력은 전진·후진과 좌우 회전을 함께 전달합니다. 좌우 수평 입력에서는 `abs(y) <= 0.10 * min(1, abs(x))` 범위의 작은 상하 흔들림을 무시해 전진량을 0으로 유지하고, 상태에 ‘제자리 좌회전/우회전’을 표시합니다. 범위 바깥에서는 전진량이 연속적으로 증가하며 기존 직진 보정 범위도 유지합니다. 실제 BLE·가상 BLE·가상 TCP에 같은 입력 매핑을 사용합니다.

수평 범위, 경계 연속성, 네 방향 대각선 및 기존 제어를 포함한 XCTest 42개 통과. Mac 대상 Debug 빌드 성공.

### V0.3.5 (11): IMU 직진 방향 유지 ON/OFF

지원 제어기의 `headinghold` capability에 따라 ‘직진 보정 → IMU 직진 방향 유지’ 토글을 표시합니다. `heading on/off`로 변경하고 상태 응답의 `heading=on/off`를 표시합니다. 보행 중 변경하면 정지 완료 후 적용합니다. 현재는 `heading-v6-sim` 가상 로봇이 지원하며 기존 실제 펌웨어에는 항목을 표시하지 않습니다. [제어 범위와 검증](../../../docs/HEADING-HOLD-2026-09-10.md).

### V0.4.0 (12): 실제/가상 공통 보행 제어

V17의 `gaitprofiles`/`balancecontrol`/`headinghold` capability에 따라 실제 BLE 로봇에서도 보행 정책·수평 보정·IMU 직진 방향 유지 설정을 제공합니다. 이전 시뮬레이터 명령도 호환합니다. [공통 코드·검증·실측 항목](../../../docs/SHARED-LOCOMOTION-V17-2026-09-10.md).

## Level15 (2026-09-10)

V0.4.2 (14) / shared-locomotion-v20: `level15` (**수평 + 발 들기 · 15mm**) 추가. 기존 Level은 유지합니다. 15mm는 명령 높이이며 기본 모델의 실제 발높이 중앙값은 앞발 약 6mm, 뒷발 약 10mm입니다. 상세 측정과 영상은 `docs/RAISED-LEVEL-2026-09-10.md`를 참고하십시오.

## V0.4.3 (15)

`joint`: **J2·J3 협응 보행** 추가. 공통 정책 shared-locomotion-v21. 상세 기록은 `docs/J2-J3-COORDINATION-2026-09-10.md`.

## J2 협응 · 빠르게 (v22)

앱 V0.4.4 (16), `jointfast` 모드 추가. 기존 joint 유지. 최대 전진 보폭 85mm, 주기 1.5초로 기본 모델에서 약 25% 속도 향상. 측정값·제약·영상은 `docs/J2-FAST-GAIT-2026-09-10.md`. 실제 폰 UJIN17만 사용하며 iOS Simulator를 실행하지 않습니다.

## 스포츠 모드 / 전체 정책 속도 (v23)

앱 V0.4.5 (17): `jointsport` (**J2 협응 · 스포츠**) 추가. 전체 정책 이름에 동일 조건 측정 속도를 표시하며 실패 시 `(검증실패)`로 표시합니다. `docs/J2-SPORT-AND-SPEED-LABELS-2026-09-10.md` 참고. 실제 폰 UJIN17만 사용합니다.

### V0.5.0 (21) — 터미널 / MuJoCo 영상 전환

- 스크롤 영역의 컨트롤 버튼 위 여백을 32pt로 늘렸습니다.
- 상단 로그 영역을 좌우로 스와이프하면 터미널과 MuJoCo 화면을 전환합니다. 터미널 크기와 하단 조종기 배치는 유지합니다.
- Mac과 실제 폰을 같은 Wi-Fi에 연결하고, 앱의 로컬 네트워크 접근을 허용하십시오. 영상 페이지가 Bonjour로 Mac을 자동 검색합니다.
- 자동 검색이 안 되면 영상 헤더의 네트워크 버튼에서 Mac의 IP만 입력하십시오. 빈 값으로 연결하면 자동 검색으로 복귀합니다.
- 기존 `virtual_robot.py --viewer` 실행 시 영상 서버도 시작합니다. 먼저 Mac의 `spot_omg` Python 환경에서 `python -m pip install -r simulation/mujoco/requirements-video.txt`로 영상 의존성을 설치하십시오.
- 영상은 LAN TCP 8766 포트의 읽기 전용 JPEG 전송(640×360, 최대 10fps)입니다. BLE 제어와 별개이며, LAN에서 해당 포트 접근이 가능해야 합니다. 네트워크 지연이 있는 모니터 영상입니다.
- Mac 뷰어의 카메라 시점과 로봇 상태를 별도 프로세스에서 렌더링합니다. 화면 수신이 없으면 렌더링을 쉬며, 오래된 영상은 표시하지 않습니다. 앱에서 터미널로 돌아가거나 백그라운드로 가면 수신을 중지합니다.
- 영상 서버 비활성화: `--no-video`. 바인딩 주소/포트: `--video-host` / `--video-port` (수동 앱 주소는 기본 8766, Bonjour는 공지된 포트 사용).
- 실제 iPhone 대상 Debug 빌드와 Mac 영상 HTTP 수신을 확인했습니다. 휴대폰 에뮬레이터는 사용하지 않았습니다.

### V0.5.0 (22) — 컨트롤 패널 바깥 여백 수정

- 흰색 패널 내부 상단 여백은 기존 22pt로 복원했습니다.
- 스크롤 콘텐츠 상단에 16pt 여백을 두어 패널 바깥 회색 영역을 늘렸습니다.
- 늘어난 바깥 여백만큼 패널 높이를 조정하여 첫 화면의 조이스틱이 아래로 밀리지 않도록 했습니다.

### V0.5.0 (23) — 회색 상단 여백 표시 수정

- 실제 폰 스크린샷에서 List의 콘텐츠 상단 여백이 나타나지 않는 것을 확인했습니다.
- List 자체의 바깥 상단에 16pt 패딩과 systemGroupedBackground 배경을 적용했습니다. 흰색 패널 내부 여백은 22pt로 유지합니다.

### V0.5.0 (24) — MuJoCo 영상 부드러움 개선

- 10fps 제한과 앱의 프레임 수신 후 100ms 대기를 제거했습니다. 물리 상태가 갱신될 때만 영상을 생성하며 최대 50fps를 목표로 합니다.
- HTTP 연결을 재사용하고 마지막 프레임 번호를 전달하여 새 프레임이 준비될 때 수신합니다. 중간 프레임이 쌓이지 않도록 큐는 한 개로 유지합니다. 프레임 보간으로 보행을 꾸미지 않습니다.
- 960×540 / JPEG 품질 85로 올리고 로봇을 기존보다 약 1.33배 확대합니다. 영상 헤더에 실제 수신 FPS가 표시됩니다.
- Mac 로컬 HTTP 8초 측정: 약 31.94fps, 시뮬레이션 시간 진행 1.001배, 영상 약 4.93Mbps. 폰의 Wi-Fi 성능에 따라 수신 FPS는 달라집니다. 이 수치는 정지 자세에서 측정했으며 보행 안정성 검증을 의미하지 않습니다.
- 영상 전송 및 물리 시간 조절 테스트 6개 통과, 실제 iPhone 대상 Debug 빌드 성공.

### 2026-09-10 — 영상 FPS 우선 설정 (앱 V0.5.0 (24) 호환)

- 서버 영상은 480×270, JPEG 품질 65로 낮췄습니다. 그림자는 512 크기로 유지하고 오프스크린 MSAA를 껐습니다.
- 작은 응답의 전송 지연을 줄이기 위해 TCP_NODELAY를 적용했습니다.
- 일괄 물리 갱신 마지막에만 프레임을 전달하던 코드를 각 20ms 물리 갱신 직후로 이동했습니다. 단일 큐와 별도 렌더링 프로세스는 유지합니다.
- 테스트 종료 후 Mac 로컬에서 8초 측정: 40.27fps / 약 1.64Mbps / 시뮬레이션 진행 1.0004배. 이전 960×540의 약 31.94fps / 4.93Mbps 대비 개선되었습니다. 정지 자세 측정이며 폰의 실제 FPS는 Wi-Fi와 부하에 따라 달라집니다.
- 관련 테스트 36개 통과. 서버 재시작으로 적용되며 앱 재설치는 필요하지 않습니다.

### 2026-09-10 — 영상만 연결되는 BLE 브리지 종료 문제

- 장애 확인 당시 MuJoCo 제어 TCP와 영상 서버는 실행 중이었지만 SpotOMGSimBridge 프로세스는 없었습니다. 제어 TCP의 가상 로봇 identity 응답은 정상이었습니다.
- BLE 상태창의 마지막 창 닫기가 앱 종료로 이어지는 동작을 수정했습니다. 상태창을 닫아도 BLE 서비스는 유지하며, 앱을 다시 열면 상태창을 표시합니다. 명시적 앱 종료 및 MuJoCo 종료 시 브리지 정리는 유지합니다.
- Swift 브리지를 다시 빌드하고 MuJoCo와 함께 재시작했습니다. 폰 앱 재설치 없이 가상 로봇 · BLE에 다시 연결할 수 있습니다.

### 2026-09-10 — 폰 TCP 연결 시간 초과 수정

- 제어 서버의 기본 수신 주소가 127.0.0.1이라 Mac 내부에서만 접근할 수 있었습니다. 기본값을 0.0.0.0으로 변경하여 같은 LAN의 폰에서도 TCP 8765에 연결할 수 있도록 했습니다.
- 폰의 가상 로봇 · TCP 주소에는 Mac의 Wi-Fi IP를 입력합니다. 127.0.0.1은 폰 자신이므로 Mac 주소로 사용할 수 없습니다.
- Mac 내부에서만 제어하려면 `--host 127.0.0.1`을 지정합니다. 영상 TCP 8766과 조종 TCP 8765는 서로 별개입니다.

### V0.5.0 (25) — 실제 폰 TCP 연결 복구

- 개발 실행 옵션 `--simulator-tcp --simulator-host <Mac IP>`를 추가했습니다. 이 옵션은 제어 대상을 가상 로봇 TCP로 저장하고, 지정한 Mac 주소를 저장한 뒤 화면이 준비되면 TCP 식별을 시작합니다. 보행 명령은 자동 실행하지 않습니다.
- 실제 UJIN17에 설치하고 `--simulator-tcp --simulator-host 192.168.0.62`로 실행했습니다. 서버에서 폰 192.168.0.10 → Mac 192.168.0.62:8765의 ESTABLISHED 연결을 확인했습니다.
- 이후 일반 실행에도 저장된 TCP 대상과 Mac 주소를 사용합니다. 정상 연결에는 동일 Wi-Fi와 현재 Mac IP가 필요합니다.

### V0.5.0 (26) — Tailscale / 5G 영상 주소 연동

- 가상 로봇 TCP를 선택하면 영상 서버 주소도 TCP의 Mac 주소를 사용합니다. 영상용 수동 주소 또는 LAN Bonjour 검색에 따로 의존하지 않습니다. TCP 주소를 변경하면 활성 영상도 재연결합니다.
- 예: TCP `100.67.61.114:8765`, 영상 `http://100.67.61.114:8766/frame.jpg`. 폰의 Tailscale 주소가 아닌 MuJoCo를 실행하는 Mac의 Tailscale 주소를 입력합니다.
- 영상 URLSession의 셀룰러 접근을 명시적으로 허용하고 안내 문구에 Tailscale을 추가했습니다. 양쪽 Tailscale이 연결되어 있어야 합니다. 포트/서비스의 접근 정책은 기존 Tailscale 설정을 따릅니다.
- 사용자가 Tailscale IP를 입력하여 조종되는 것을 확인했습니다. Mac에서 해당 Tailscale 주소로 제어 identity와 영상 HTTP 200 응답을 확인했습니다. 폰의 5G 영상 표시는 별도 확인이 필요합니다.

### V0.5.0 (27) — Tailscale 영상 HTTP 예외

- TCP 조종은 되지만 영상이 안 나오는 증상에서, 기존 ATS 설정이 로컬 네트워크만 허용하고 Tailscale 원격 IP의 HTTP 예외는 없음을 확인했습니다. iOS의 원격 IP HTTP 제한이 원인일 가능성이 있어 `100.64.0.0/10` 범위에 `NSExceptionAllowsInsecureHTTPLoads`를 추가했습니다. 전역 HTTP 허용은 사용하지 않습니다.
- Apple 참고: https://developer.apple.com/documentation/bundleresources/information-property-list/nsapptransportsecurity/nsallowslocalnetworking 및 https://developer.apple.com/documentation/bundleresources/information-property-list/nsapptransportsecurity/nsexceptiondomains
- 영상 연결 실패 시 NSError 코드와 설명을 화면/개발 콘솔에 표시합니다. `--simulator-video` 실행 옵션으로 초기 영상 페이지를 열어 실기기에서 확인할 수 있습니다.
- V0.5.0 (27)을 실제 폰에 설치하고 Tailscale 주소로 영상 페이지를 열었습니다. 폰 100.70.226.1에서 Mac 100.67.61.114의 TCP 8765(조종), 8766(영상) 연결이 모두 ESTABLISHED이며 영상 프레임이 갱신되는 것을 확인했습니다. 이 확인은 폰 Wi-Fi를 켠 상태의 Tailscale 경로이며 5G 전환 후 확인은 별도입니다.

### V0.5.0 (28) — 가상 로봇 Stow 버튼

`--stow`로 실행한 MuJoCo가 `simstow` 기능을 알리면 작은 자세 버튼 모음과 상세 자세 메뉴에 Stow가 표시됩니다. Stow로 12초 동시 접기, Landing으로 12초 동시 펼치기를 실행합니다. Landing 준비가 필요한 경우 먼저 2초 동안 Landing으로 이동합니다. Hold로 중단할 수 있습니다. 실제 로봇에서는 사용되지 않는 확장 관절 설계 검토 기능입니다. 실제 서보의 기계적 가능 여부는 미검증이며 현재 펌웨어 인코딩 그대로 적용할 수 있다는 뜻은 아닙니다.
