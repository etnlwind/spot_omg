# 앱 정지와 최초 자세 복귀

최신 후속: [v62 제자리 두 걸음 정지](S-NATIVE-STOP-PLACEMENT-2026-09-15.md).
현재 V6.1은 대각선 한 쌍씩 S에 옮긴다. 아래 감속·완만한 복귀 설명은 이전 기록이다.

후속: 사용자의 실기 반영 요청으로 [V6.1 실기 이식](S-NATIVE-V61-FIRMWARE.md)을 추가했다.
아래 v58/실기 Stand 설명은 이식 전 기록이다. v59에서는 V6.1의 S를 사용한다.

Windows STOP/Space와 iOS Stop 버튼은 연속 보행 세션에서 `@S`를 한 번 보내고
제어기의 감속·복귀 완료 응답을 기다린다. 복귀 중 반복 누름으로 Ctrl+C를 보내지 않는다.
Windows Esc, 앱 응답 지연, 긴급 중단 경로는 유지한다. 일반 자세 전환 중 Stop은
기존 중단 기능을 사용하므로 Stow/Landing을 강제로 S로 바꾸지 않는다.

MuJoCo V6.1의 기존 복귀 제어를 앱 프로토콜과 함께 시험했다. 2초 대기, 8초 보행,
10초 정지 명령, 11.5초 반복 Stop, 14초 종료 조건에서 정지 패킷은 1개이고
Ctrl+C 없이 S 목표 오차 0°, 실제 관절 최대 오차 1.1° 미만으로 복귀했다.
Windows 검사 34개 통과, 실행 파일 빌드 및 실제 MuJoCo 연결 smoke 시험 성공.

STM32 `robot_shared_drive`는 이미 정상 `@S`/watchdog 정지 후 감속하여
출발 자세 Stand(J1=0°, J2=45°, J3=90°)로 복귀한다. 따라서 해당 동작은
덮어쓰지 않고 앱에서 정상 정지 경로를 사용하게 했다. 실기의 Stand는 CAD 기반
S와 다르며 V6.1 보행 모델은 여전히 시뮬레이터 전용이다. 펌웨어 프로필 순서,
서보 부호 및 누적 위치 좌표는 변경하지 않았다.

Windows에서도 펌웨어를 빌드하도록 `build_firmware.py --compiler`를 추가했다.
빌드 결과: `artifacts/s-native-v6-1/firmware-stop-return/shared-locomotion-v58.bin`,
271084바이트. 체크섬과 부트 벡터 검증은 같은 폴더 `manifest.json`에 있다.
이 바이너리는 기존 v58 제어 동작의 현재 소스 빌드이며 새 보행 펌웨어가 아니다.

장치 확인 시 STM32/ST-Link USB가 없고 SpotOMG BLE 광고도 검색되지 않았다.
실물 업로드·설정 readback·위치 유지·전체 동작은 미실시다.
iOS 소스와 Stop 회귀 시험을 추가했으나 Windows에는 Xcode가 없어 빌드·설치는 미실시다.
후속 작업: 실제 기체 연결 후 펌웨어 버전/설정을 조회하고 필요 시 업로드,
위치 유지 및 정상 정지 시험을 순서대로 수행한다. iOS는 Mac에서 XCTest와 빌드 후 설치한다.
