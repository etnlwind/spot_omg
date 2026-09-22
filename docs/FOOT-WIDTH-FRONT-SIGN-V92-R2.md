# 앞다리 간격 부호 수정 · V92-R2 (66)

## 설치 완료

ESP32 전원 재시작 후 install-r2-05에서 설치에 성공했다.
Landing 도착(27틱) → 12축 토크 OFF → R2 이미지 검증·OTA·재부팅 →
Landing 도착(22틱)을 확인했다. 종료 상태는 Landing/토크 ON/safety OK.
12축 영구 레지스터와 들림20/20/20/20mm, 간격-30/-30/+30/+30mm를 보존했다.
설정값을 임의로 반전하지 않았다. 실제 보정 보행/위치 유지 검증은 미수행이다.
증거: `artifacts/foot-position/install-r2-05/post-verify.json`.
아래는 성공 이전 실패·복구 과정 기록이다.

## 설치 진행 기록

사용자 진행 요청 후 Windows R2 (66) 설치·실행과 바탕화면 바로가기 교체 완료.
설치 위치는 `%LOCALAPPDATA%/Programs/SpotOMG/V92-R2-66/SpotOMGController.exe`이다.
실기 설치 전 R1은 Stand/토크 ON/정지였으나 Landing 명령이 IMU balance error로
실패했다. `imurecover`는 chip=0xA0, mode=0x08을 읽었지만 body samples=0/3으로
실패하여 토크 해제와 OTA는 하지 않았다. 사용자 재부팅 후 BLE 연결 시간 초과,
재검색에서 브리지 미발견으로 설치는 대기 중이다. 증거는
`artifacts/foot-position/install-r2-01`부터 `install-r2-03`에 보존했다.
마지막 조회값: 추가 들림20/20/20/20mm, 간격-30/-30/+30/+30mm(FL/FR/RL/RR).
재부팅 이후 현재 자세·설정은 연결 복구 뒤 다시 조회해야 한다.

사용자가 V92-R1 설치 후 앞다리 간격 부호가 반대라고 관측했다. V6 비-native
보행의 공통 보정에서 모터 J1 각도를 CAD J1 각도로 오인한 오류를 확인했다.
이미 기록된 앞 J1의 기구 방향 차이를 신규 기능 검증에 반영하지 못했다.

R1은 -5mm에서 앞발이 실제 모터→CAD 변환 기준 약+5mm 바깥으로 향했고,
뒷발은 -5mm 안쪽으로 향했다. 앞발 높이도 약2.4~2.7mm 달라졌다.
이는 새 실기 변위를 측정한 수치가 아니라 명령을 실기 부호로 재해석한
기구학 계산이다. 사용자 관측과 일치하며 부호 오류는 소프트웨어에서 재현된다.

공통 `foot_width_apply`에서 앞 J1만 모터→CAD로 변환하고, 기본 X/Z를 보존하며
Y를 조정한 후 CAD→모터로 되돌린다. 이미 CAD 좌표를 사용하는 S-native 보정과
서보 영점·방향 설정은 변경하지 않는다. 0mm는 기존 명령과 완전히 동일하다.

## 검증

- C 호스트: 배포 비-native 보행 전후진/위상/각 다리 ±5mm, 실제 모터→CAD 발 위치,
  X/Z 보존, 다른 다리 불변, 0 무변경, 도달불가 원자적 실패 확인.
- 원시 틱 방향도 검사: 안쪽일 때 FL ID1 감소, FR ID4 증가, RL ID7 증가, RR ID10 감소.
- MuJoCo CAD에 양자화된 모터 명령을 실제 장착 부호로 투영해 ±간격과 X/Z 보존 확인.
  이 검사는 동역학 보행 성공 시험과 다르다. 기존 legacy simulator의 일반 모터/CAD
  경계는 이번 수정에서 전면 변경하지 않았으므로 그 화면만으로 실기 방향을 판단하지 않는다.
- simulator 공용 C 라이브러리 캐시 키에서 누락된 `foot_lift.h`를 추가했다.
  헤더 수정 후 예전 DLL을 재사용하던 문제를 함께 수정했다.
- 관련 firmware/simulator/Windows 시험 31개 통과. Native 7종의 시작/정지 일치 포함.
- 펌웨어 빌드 326260바이트, SHA256
  `cf97c89ca87c509d0e8c5296fe3b42e6e82bdaf898b30c6e959a2b1fd7ac22ed`.

수정 빌드는 V92-R2, 앱은92.2.0/build66이다. Apple 빌드는 미수행.
**구현 직후에는 R1이었으며, 후속 R2 설치는 위 설치 완료 기록을 참고한다.**
보정값을 임의로 반전하거나 EEPROM을 변경하지 않았다. R2 설치 후에도 실제 위치 유지와
전체 보행 검증을 구분한다. 설치 시 Landing 도착을 확인하고 토크 OFF 후 진행한다.

명령 계산 비교: `artifacts/foot-position/sign-audit/before.txt`, `after.txt`.
