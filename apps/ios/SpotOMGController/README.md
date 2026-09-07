# SpotOMGController for iOS

SwiftUI와 CoreBluetooth로 `SpotOMG-Bridge`에 직접 연결하는 iPhone 앱입니다.
자세·진단 버튼은 기존 STM32 text console을 사용하고, 조이스틱은 console prompt와
분리된 sequence/heartbeat realtime 제어 lane을 사용합니다.

## 실행

1. Xcode에서 `SpotOMGController.xcodeproj`를 엽니다.
2. Signing & Capabilities에서 개인 Team을 선택합니다.
3. Bluetooth가 가능한 실제 iPhone을 실행 대상으로 선택합니다.
4. 로봇 전원을 켜고 앱을 실행한 뒤 Bluetooth 접근을 허용합니다.

iOS Simulator에서는 실제 BLE peripheral 연결을 검증할 수 없습니다.

## 현재 기능

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
