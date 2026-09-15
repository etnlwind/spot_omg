# V6.1 실기 펌웨어 이식 — 2026-09-15

> 후속 진단: v59에서 실제 앞 J1 두 개가 바깥으로 벌어지는 좌표 변환 오류를 확인했다.
> [v60 수정·설치·검증](S-NATIVE-V61-FRONT-J1.md)을 우선 참고한다.
> 현재 소스/최종 빌드는 [v62 두 걸음 정지](S-NATIVE-STOP-PLACEMENT-2026-09-15.md)이며 [v61 회전 수정](S-NATIVE-YAW-FIX-2026-09-15.md)을 포함한다.
> 마지막 실기 설치는 v60이다. v62는 호스트·시뮬레이터 복귀 검증/빌드까지 수행했으며 접지 동기 한계가 남고 실기 미설치다.

아래 최초 이식 소스/바이너리는 `s-native-v6-1-v59`, 모델은 `s_native_v6_1`이다.
V6 및 이전 정의를 보존하고 펌웨어 모델 배열 끝에 V6.1을 추가했다.
기존 인덱스는 변하지 않는다. Windows/iOS는 명시적인 `s_native_v6_1`
capability가 있는 실기에서만 모델을 허용하며 지원되는 연결의 기본으로 선택한다.

**2026-09-15 14:45 KST: 실제 기체 설치와 버전 readback 완료.** iPhone 연결을
해제한 뒤 `SpotOMG-Bridge` (38:18:2B:2F:7C:22, RSSI −39dBm)가 검색됐다.
기존 v57에서 토크 OFF를 확인하고 `--skip-landing` OTA로 자세 이동 없이 설치했다.
ESP32 이미지 검증, STM32 플래시 100%, 검증·재부팅이 모두 완료됐다.
재접속 후 `rev=s-native-v6-1-v59`, `profile=s_native_v6_1`, 해당 capability,
`safety=ok torque=off`를 확인했다. ID1은 12.2V, 32°C, moving=0, hw=0x00이다.
실제 위치 유지 및 보행·정지 동작 시험은 아직 하지 않았다.
로그: `artifacts/s-native-v6-1/hardware-deployment-2026-09-15/`.

## 구현

- `s_native.h`, `s_native_impl.h`: CAD 기구학의 고정 쿠션 재질점 XY와 최저 표면 Z,
  잠긴 J1에서의 J2/J3 풀이, FR 첫걸음 2배 오므림과 대각선 공통 X/Z·위상.
- S 기준 앞쪽 +20mm / 뒤쪽 −65mm, 최대 입력 주기 1.2초, 발 들기 12mm.
- 정상 `@S`와 watchdog 정지는 위상을 진행하며 감속·오므림 해제 후 1초 동안 S로 보간한다.
  반복 정상 Stop은 복귀를 중단하지 않으며 긴급 중단 경로는 유지한다.
- S 목표는 CAD로 계산한 약 J1 0.0076°, J2 46.0114°, J3 90.0160°다.
  기존 45°/90° Stand와 구별한다. V6.1의 Stand/보행 진입/보행 종료/idle 유지에 S를 사용한다.
  프로필 변경만으로 서보가 움직이지 않도록 idle 보정을 중단한다.
- 기존 서보 보정/부호/누적 좌표를 재사용한다. Stow/Landing 경로는 그대로다.
  기존 두 링크 균형 보정은 S-native 목표에 덧씌우지 않는다. IMU 관측/기울기 보호,
  서보 추종 보호, watchdog, 늦은 프레임 보호, 방향 유지 제어는 유지한다.
- CAD 동역학 지지 기준은 전진/회전 입력 각각 0.1 간격의 64개 위상 표로 내보낸다.
  STM32에서는 마이크로미터 int16 표를 보간한다. 이것은 연산·정밀도상의 근사이며
  접촉력 센서 피드백이 아니다. 물리 상태/힘 측정치를 제어 입력으로 쓰지 않는다.

공용 CAD 메시를 한 번만 포함하여 320KiB 앱 플래시 안에 넣었다.
바이너리: `artifacts/s-native-v6-1/firmware-v59/s-native-v6-1-v59.bin` (306248바이트).
SHA256: `5804ece1d2c10dca778f2c9b1ca5e2370a52d91906a1a0e4f2433d7246983332`.
부트로더·보정 저장 구역·이미지 시작 주소는 유지하며 `manifest.json`에 부트 벡터와 해시가 있다.

## 검증 범위

- 호스트 C와 보존된 Python V6.1 비교: 전진/후진/회전/복합 입력, 초기·정상 보행 정지.
  기록된 8개 조건에서 최대 관절 목표 차이 0.007013°. 각 목표의 실기 보정/서보 범위도 검사한다.
- C 커널로 MuJoCo 2초 대기 + 8초 전진 + 4초 복귀: 최대 기울기 3.2055°,
  영상 시간 12.20초에 완료 응답, S 목표 오차 0°, 실제 관절 오차 최대 0.9897° 미만.
- 앱·신규 C 커널·기존 공용 C 정책 검사 70개 통과. 비상/반복 Stop, 실기 capability,
  기존 정책의 O0/O2 서보 틱 일치 검사 포함.
- 기존 C 안전/다회전 서보 좌표/Stow 동작/자세 감독/명령 복구 실행 검사 5개 통과.
  Windows Zig로 `-UNDEBUG`를 사용하여 C assert를 켜고 실행했다.
- 이전 S-native 및 OTA 배치 검사도 통과. Windows 실행 파일 빌드와 MuJoCo 연결 smoke 성공.
- 실물 펌웨어/모델/토크/안전 상태와 ID1 상태 readback 완료. 전체 서보 설정,
  위치 유지, 실제 20ms 제어 연산 시간, 전체 보행/정지 시험은 미실시.
  iOS 소스/시험은 갱신했지만 Xcode 빌드와 기기 설치는 Mac에서 필요하다.

결과: `artifacts/s-native-v6-1/firmware-validation/`, `firmware-host-units/`.
기존 저속 뒷발 들림 부족, 옆 방향 편류, 접촉 물성 추정의 한계는 남아 있다.

## 재현과 다음 단계

```powershell
python -X utf8 simulation/mujoco/scripts/validation/export_s_native_firmware.py --output firmware/stm32-learning/Inc/s_native_data.h
python -X utf8 tools/generate_locomotion_profiles.py
python -X utf8 simulation/mujoco/scripts/validation/validate_s_native_firmware.py --output artifacts/s-native-v6-1/firmware-validation
python -X utf8 simulation/mujoco/scripts/validation/check_firmware_windows.py --output artifacts/s-native-v6-1/firmware-host-units
python -X utf8 firmware/stm32-learning/build_firmware.py --compiler <arm-none-eabi-gcc 경로> --output artifacts/s-native-v6-1/firmware-v59
```

로봇 BLE 이름은 `SpotOMG-Bridge`, 서비스는 `6e400001-b5a3-f393-e0a9-e50e24dcca9e`다.
이번 설치는 기존 펌웨어/자세/안전/토크 상태를 읽고 기존 OTA 도구를 사용했다.
토크 OFF가 확인된 상태에서는 `spotctl --via ble firmware stm32 <bin> --skip-landing`으로
자세 동작 없이 업데이트할 수 있다. Windows에서는 `--no-app-control`도 지정한다.
명령 도구의 macOS 전용 fcntl import를 iPhone 제어 경로로 옮겼으며 회귀 6개가 통과했다.
업로드 진행률은 즉시 출력하도록 수정했다. 그 외에는 도구의 기존 Landing 경로를 따른다.
설치 후 `syncstate`로 v59 및 capability 확인까지 완료했다. 다음 검증은 실기 위치 유지부터다.
실기에서 관측된 실패를 시뮬레이터/호스트에 재현하기 전에는 검증 완료로 기록하지 않는다.
