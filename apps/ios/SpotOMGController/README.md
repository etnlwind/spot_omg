# SpotOMGController for iOS

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
