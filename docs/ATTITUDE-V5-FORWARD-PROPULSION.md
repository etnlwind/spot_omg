# IMU 자세 안정화 V5 — 전진 추진 개선 (V91)

2026-09-21 사용자 의도: V4에서 빠른 회수를 보고 모터 여력이 있어 보이는데, 뒤로 미는 동작 자체가 너무 느리다는 요청. 회수를 늦추는 것이 목적이 아니다. 전진을 우선 개선한다. 사용자는 현재 보행 대기 자세 B 유지를 선택했다. 이 V5는 과거 S-native V5와 별개인 `attitudepd_v5`다.

## 최종 동작

- V4 보폭·발 경로·회수 모양·다리별 추가 높이·Stand B를 유지한다.
- 전진 입력 0→0.588 사이에서 주기 가속을 smootherstep으로 1→1.3배 적용한다. 0.588은 앱의 반 스틱 입력이다. 그 이상 전진에서는 1.3배다.
- 최대 전진 주기 1.05→약 0.8077초. 지지/회수 비율 52/48%, 대각선 위상 그대로. 미는 속도와 회수 속도가 함께 30% 증가하며, 회수를 느리게 하지 않는다.
- 후진과 순수 제자리 회전은 V4와 동일한 궤적·주기다. 전진하면서 회전할 때는 전진량에 따라 가속된다.
- 서보 속도·가속도·토크·보호 임계값은 변경하지 않았다. 보행 시작의 기존 12축 프로필 readback도 유지한다.
- 새 모델을 프로필 인덱스 27에 추가했다. 기존 인덱스 0..26과 V4는 보존했다. 지원 capability가 있는 연결에서 앱 모델 목록 맨 위·기본 모델이며 시뮬레이터 기본 모델도 V5다.
- Apple/Windows 앱 소스는 V91(91.0.0), Apple build 63. 펌웨어는 `attitudepd-v5-v91`.

## 방향을 바꾼 근거

초기 회수 완화/CAD 재구성 초안은 사용자의 뜻과 달라 제외했다. 기록은 `artifacts/attitudepd-v5/retired-draft/`와 개별 simulation 결과에 남겨 두었다. 앞뒤 모두 주기를 줄인 후보는 후진 이동속도가 오히려 감소하여 채택하지 않았다.

최종 선택은 전진 30% 가속이다. 50% 후보는 앞발 +30mm 조건에서 전진이 더 느려졌고, 앞 J3 양자화 전 목표속도도 현재 설정 speed 3400(ticks/s로 해석 시 298.83°/s)을 초과했다. 30% 후보의 최대 명령속도는 281.25°/s다. 이것은 설정과의 비교일 뿐, 하중 상태에서의 추종 가능성을 보장하지 않는다. 저장소의 J3 추정 무부하 정격과 분석용 가속도 기준을 넘는 구간도 있어 실측 검증이 필요하다.

## 검증 수준

### 호스트·설정·빌드

- V4/V5 동일 위상 목표각과 발 좌표가 정확히 일치한다. 69,300개 입력/위상/높이 사례와 시작·전후진 전환·정지, 실제 시간 기준 후진/순수회전 동일성을 검사했다.
- 다리별 높이 0, 앞 30/30/뒤 0/0, 비대칭 높이에서도 IK·서보 변환을 검사했다.
- 펌웨어/프로토콜/시뮬레이터·Windows 관련 검사 270개 통과. 추가 simwalk/PD 시간 일치·UI 검사 18개 통과.
- Apple 테스트와 iOS 빌드를 수행했다. 상세 수·빌드 결과는 아래 로그 참고.
- 펌웨어 326,988바이트, 고정 OTA 슬롯 327,680바이트 내. 진단 코드 `mechanical_diagnostics.c`만 추가로 크기 최적화했다. 플래시 배치와 운동 제어 컴파일 최적화 수준은 유지했다.
- STM32 상태 응답에 V5 capability를 추가했고 766바이트 제어 프레임 한계 내임을 확인했다.

### 추정 물리 시뮬레이션

기존 2,754g 모델·마찰·접촉·전압·서보 제한 그대로, IMU PD 및 heading OFF, 각 8초 보행을 비교했다. 충돌이나 보호를 끄지 않았다. 수치는 실제 로봇 측정이 아니다.

| 앞 추가 높이 | 전진 입력 | V4 몸체 속도 | V5 몸체 속도 | V5 시작/보행/정지 |
|---|---:|---:|---:|---|
| 0/0mm | 0.588 | 0.0605m/s | 0.0841m/s | 완료 |
| 0/0mm | 1.0 | 0.1606m/s | 0.1561m/s | 완료 |
| 30/30mm | 0.588 | 초반 기울기 중단 | 0.0924m/s | 완료 |
| 30/30mm | 1.0 | 0.1653m/s, 정지 중 기울기 중단 | 0.2315m/s | 완료 |

사용자가 앞서 사용한 앞발 +30mm 조건에서 최대 전진의 추정 몸체 속도는 약 40% 증가했다. 추가 높이 0mm의 최대 전진에서는 몸체 속도가 약 3% 줄었으므로 모든 설정에서 이동이 30% 빨라진다고 주장하지 않는다. +30mm 조건의 추정 추종 오차는 여전히 약 31°까지 나타나며 접지/미끄러짐도 완벽하지 않다. 후진 +30mm의 기존 기울기 중단 문제는 이번 전진 우선 변경으로 해결하지 않았다. 2026-09-22 설치 직전 실기에서 읽은 추가 높이는 네 다리 모두 0mm였다.

### 실기

2026-09-22 사용자 요청으로 STM32 V91과 Mac 앱 91.0.0(63)을 설치했다. iPhone은 이번 설치 대상이 아니다.

- 기존 V90-R3에서 일반 Landing 명령의 실제 완료(`elapsed=5488ms`)와 위치 오차 8틱, torque ON, safety OK를 확인했다. 이후 Relax 및 12개 서보의 토크 레지스터 OFF readback을 확인하고 OTA를 진행했다. 직후 자세 조회는 Landing, 오차 54틱이었다.
- OTA는 326,988바이트 전체 기록·검증·재부팅을 완료했다. SHA-256은 `93f295671721580378fc4c3a979f625c6044aa1ba434b679c50c99ab50087267`이다.
- 재부팅 후 `rev=attitudepd-v5-v91`, `profile=attitudepd_v5`, capability V5, safety OK/fault 0을 확인했다. 추가 높이는 설치 직전 값 0/0/0/0mm가 보존됐다. 보존 확인을 위해 새 설정을 덮어쓰지 않았다.
- **설치 후 Landing 분류 검증은 통과하지 않았다.** 재부팅 후 `pose=custom error=351 torque=off`였다. 추가 읽기에서 J3의 Landing 목표 대비 편차는 3~8틱인 반면, J2 네 축의 편차는 307~351틱(약 27~31°)이었다. 분류 코드는 12축 최대 편차가 80틱을 넘으면 custom으로 표시하며, 이 Landing 기준과 분류 규칙은 V5에서 변경하지 않았다. 토크 OFF 이후 자세 변화와 일치하지만, 직접 관찰 없이 중력 안착이나 기구적 끝점 도달을 확정하지 않는다.
- 설치 후 확인은 읽기만 수행했고 토크 OFF 상태를 유지했다. 분류를 통과시키려고 보호 기준을 바꾸거나 자세 이동을 반복하지 않았다. 별도 실제 위치 유지·접지 보행·부하 추종 검증은 미실시다.
- Mac 앱은 `/Users/etnlwind/Applications/Spot OMG!.app`에 설치했고 서명과 기존 주황색 아이콘을 검증했다. 이전 앱은 백업했다.

설치 이전의 IMU fault 11 때문에 첫 Landing 시도는 거부됐다. `imurecover`는 I2C 재초기화에는 성공했으나 body 샘플 검증은 0/3으로 실패했다. 이후 상태 조회가 safety OK/fault 0이었고, 두 번째 일반 Landing 명령이 위와 같이 완료됐다. V91 설치로 기존 IMU 읽기 문제가 해결됐다고 판단하지 않는다.

### 후속 사용자 보행 관측: 네 발 복귀 끌림

설치 이후 사용자가 직접 Mac 앱으로 Landing·Stand·전진을 실행했다. 2026-09-22 00:13:32 기록에서 V91의 Landing(error 8, torque ON)을 확인했다. 이는 위 설치 자동 확인 실패 이후의 별도 사용자 실행 결과다.

사용자는 네 발 모두 복귀 시 바닥을 끈다고 보고했다. Mac에 기록된 마지막 최대 전진 세션(seq 278)의 J3 최대 목표/실측 차이는 FL15.7°, FR14.5°, RL17.1°, RR15.9°였다. J3 명령은 12축 쓰기에 포함되지만 실제 추종 오차가 크다. 실제 접지 높이를 독립 측정한 것은 아니며, 특정 모터/하중 원인을 확정하지 않는다. 기존 추정 시뮬레이션의 추가높이0/최대전진에서도 복귀 중앙 구간의 접촉 비율이 79~94%였으므로, 단순 보행·정지 완료는 발 끌림 해결의 검증이 아니다. 새 진단에서는 이 결과를 명시적인 개선 대상으로 취급한다.

상세: [J3 복귀 끌림 관측](../artifacts/attitudepd-v5/j3-recovery-diagnosis/OBSERVATIONS.md). 진단 중 실기 명령·펌웨어 변경은 하지 않았다.

## 근거 파일

- `config/locomotion_profiles.json`, `firmware/stm32-learning/Inc/locomotion.h`
- `artifacts/attitudepd-v5/cadence-analysis-final/{README.md,summary.json,compact.csv}`
- `artifacts/attitudepd-v5/simulation-final/summary.json` 및 각 사례 CSV
- `artifacts/attitudepd-v5/final-tests.log`, `final-integration-tests.log`, `apple-tests.log`, `ios-build.log`
- `artifacts/attitudepd-v5/firmware/manifest.json`
- `artifacts/attitudepd-v5/install/deploy-02/{prepare.json,ota.json,verify.json}` — verify 실패 기록도 보존
- `artifacts/attitudepd-v5/install/post-install-inspect-01/inspect.json`
- `artifacts/attitudepd-v5/mac-install/installation.json`

재현: `PYTHONPATH=tools/servo_tool:apps/windows:. python scripts/validation/validate_attitudepd_v5.py --linears 588 1000 --output artifacts/attitudepd-v5/recheck`
