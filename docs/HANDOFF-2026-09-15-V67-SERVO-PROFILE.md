# Handoff: V67 설치 완료, 실제 프로필 확인 실패로 보행 미시작

이 문서는 V623 handoff보다 최신이다. 사용자 목표는 실제 다리의 빠른 움직임과
후방 신전이며, 사진의 무릎 아래 링크가 바닥에 거의 수직인 자세가 기준이다.
사진 투영각을 J3 관절각으로 취급하지 않는다.

## 현재 상태

- 모델 V6.2.4, 펌웨어 `s-native-v6-2-4-v67` OTA 설치와 부팅 조회 완료.
- 바이너리 325220 bytes, SHA256
  `0a235a0a50785de5f52279818c0f0bc0e606c2107f848b079515b84b2087043a`.
- 10초 입력 시험은 보행 시작 전 프로필 검증에서 servo=4,
  `final position verification failed`, elapsed=0ms로 종료했다.
  이 오류 문구는 일반적인 문구이며 이번 코드 경로는 속도/가속도 readback
  불일치다. 구체적으로 어느 값이 달랐는지는 현재 로그에 없다.
- gait samples 없음, joint trace 없음. 보행 속도 개선 실기 검증은 미완료.
- 시험 종료 후 `relax` 성공, 마지막 상태 torque=off, safety=ok, fault_code=0.
- 후속 실제 레지스터 조회 연결은 `BLE device 'SpotOMG-Bridge' not found`로
  실패했다. 전원과 몸체 고정 유지 여부를 사용자에게 질문한 상태다.
- Git 변경은 미커밋. 기존 `hardware/power-junction/`은 건드리지 않았다.

## 원인과 구현

[상세 진단](SERVO-PROFILE-RESTORE-2026-09-15.md).
v66 실기 12개 서보는 소프트웨어 요청 3400/254와 달리 실제 300/30이었다.
S 진입이 낮춘 설정을 위치만 갱신하는 보행에서 복구하지 않았기 때문이다.
V67은 S 진입 후 프로필만 쓰고 12개를 검증하도록 했다. 시험 스크립트의
중복 외부 `stand`는 제거했으나 내부 감독된 S 진입은 남아 있다.
사용자의 최신 질문은 “이제 출발준비는 없어진거 아냐?”였다. 별도의 보행
준비 단계와 실제 시작 함수에 남은 S 진입을 구분하여 설명했다.

## 검증

- Python 관련 검사 67개 통과, C 중요 회귀 5개와 새 프로필 회귀 1개 통과.
- 펌웨어 빌드 및 OTA 성공. 물리 프로필 복구 검증은 실패했다.
- 기존 기록 재생은 300/30을 반영했을 때 오차가 크게 감소했다.
- 몸체 고정 시뮬레이션은 완료했지만 바닥에서는 기울기 보호 중단.
- 독립 정지 위치 유지 시험과 전체 실제 바닥 보행 성공은 이번 수정에서
  확인하지 않았다. 사용자에게 속도/자세 문제가 해결됐다고 보고하면 안 된다.

## 다음 작업

1. 연결 복구 후 토크를 켜지 않고 servo 1..12의 주소 0..49를 조회해
   가속도 byte41, 속도 byte46/47 및 torque byte40을 기록한다.
   서보 전원을 재인가했다면 값이 바뀔 수 있으므로 이전 실패 당시 값과 구분한다.
2. ID4 불일치의 실제 값과 쓰기 처리 시점을 확인한다. 근거 없이 검증을
   제거하거나 오차를 허용하지 않는다. 현재 오류에 요청값·readback이 없어
   후속 진단 개선도 필요하다.
3. 몸체 고정 상태에서만 짧게 재시험하고 실제 적용값·동작 속도를 구분해 보고한다.
   `$SPOTDRIVE started`는 S 진입 전에 나와 입력 시간과 보행 시간이 다르다.

## 자료

- `artifacts/servo-profile-restore-v67/hardware-deployment/`
- `artifacts/servo-profile-restore-v67/hardware-pilot-10s/`
- `artifacts/servo-profile-restore-v67/c-tests/summary.json`
- `artifacts/s-native-v6-2-4/hardware-pilot-8s-retry/servo-registers.json`
- `scripts/hardware/check_supported_walk_30.py`

BLE 작업은 한 프로세스만 사용한다. 전원 재인가와 STM32 재부팅을 구분하며
STS3250 누적 좌표/원점을 임의로 변경하지 않는다.
