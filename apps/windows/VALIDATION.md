# Windows 검증 기록 — 2026-09-14

대상 기반: `develop`의 `0c9cb72`. Windows x64, Conda `spot_omg` Python 3.10.20,
MuJoCo 3.11.0, PySide6 6.11.2, Bleak 1.1.1, Zig 0.15.2.

- 호스트 회귀 **99개 통과**: Windows 앱 28개 + 기존 virtual robot / Stow /
  body stabilizer integration·transport 71개. 조각난 CRLF/UTF-8/무개행 prompt,
  heartbeat·중립·역방향·정지 응답 유실, sequence wrap, Relax ACK와 후속 자세
  readback, Stow 제한, capability, GATT 분할, TCP identity/EOF, GUI를 검증했다.
- 실제 Windows MuJoCo를 앱에서 시작한 **10개 통합 확인 통과**:
  identity·상태, 영상, IMU PD 정책 readback, 1.5초 조종 유지, release 정지,
  후진 재시작·Ctrl+C, GUI stall 정지, 프로세스 종료, 재시작, 창 닫기 정리.
  headless 영상 모드와 별도 GLFW 3D viewer 모드에서 실행했다.
- 배포 `.exe`에서도 로컬 MuJoCo 시작, identity·상태 응답과 JPEG 화면 표시를
  확인하고 정상 종료 코드 0을 확인했다. Python 소스 실행만의 검증이 아니다.
- 공용 보행 C와 IMU PD를 DLL로 빌드·로드했다. 관절 수식, 서보 원점·부호,
  STM32의 안전 임계값 및 펌웨어는 변경하지 않았다.

실제 BLE 검색/접속·실제 로봇 설정 readback·서보 위치 유지·전체 로봇 동작은
수행하지 않았다. BLE 테스트는 대체 GATT 클라이언트/전송 계약 검증이다.
MuJoCo는 추정 물성을 사용하므로 위 결과를 실기 동작 검증으로 해석하지 않는다.

재현 명령과 제한은 [README](README.md)에 있다. 로컬 로그·스크린샷은
`test-output/`에 생성되며 실행 파일 및 빌드 캐시와 함께 Git 추적에서 제외된다.
