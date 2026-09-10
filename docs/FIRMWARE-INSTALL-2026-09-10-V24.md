# 실제 STM32 펌웨어 업데이트 — 2026-09-10T19:28:41

사용자의 실제 로봇 펌웨어 업데이트 요청으로 Bluetooth OTA를 수행했다.
ESP32 리셋 후 `SpotOMG-Bridge` 연결이 복구되었다.

- 업데이트 전: `stride-drive-v15`, torque=off, safety=ok.
- 설치 소스: `shared-locomotion-v24` (git df3a095 기준; VS Code 설정 변경 제외).
- 빌드: `build_firmware.py --output /private/tmp/spot-physical-update-20260910`.
- 호스트 검증: `pytest firmware/stm32-learning/tests -q` — 25 passed.
- 이미지 크기: 158824 bytes.
- SHA256: `506617f34ed886472bd99cca68a494f7a3b53dd74a6bb9455391bf6bbf89174a`.
- BLE 전송 100%, ESP32 이미지 검증 완료, STM32 Flash 100%, 검증 및 재부팅 성공.
- 재연결 후 실제 보드 응답:

```text
$SPOTSTATE pose=custom error=836 torque=off safety=ok balance=full rev=shared-locomotion-v24 caps=trot5,gaitprofiles,balancecontrol,headinghold profile=cruise heading=on reverse_limit=1000
```

보행이나 자세 이동 명령은 실행하지 않았다. `error=836`은 목표/현재 자세 차이이며,
이번 검증은 펌웨어 설치와 버전/상태 조회에 한정한다. 실험용 Stow는 시뮬레이터 전용이며
실물 기능으로 추가하지 않았다. ESP32 브리지 펌웨어는 변경하지 않았다.

전송 로그: `/private/tmp/spot-physical-update-upload.log`.
설치 후 조회 로그: `/private/tmp/spot-update-after.log`.
