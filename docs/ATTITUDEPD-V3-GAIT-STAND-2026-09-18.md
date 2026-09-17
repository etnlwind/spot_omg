# IMU 자세 안정화 V3: Stand와 보행 대기 자세 통일

사용자 요청: V2의 Stand(A) → 보행 자세(B) → 보행 → Stand(A) 흐름에서
불필요한 A/B 전환을 없앤다. V2를 보존하고 `attitudepd_v3`를 별도 추가했다.
펌웨어 리비전은 `attitudepd-v3-v79`이며, 사용자 요청으로 실기 설치와 설정 readback을 완료했다.

## 자세와 동작

- A: 기존 canonical `[J1=0°, J2=45°, J3=90°]`.
- B: V2 보행 커널의 **정지 입력, 전체 진폭** 목표. 각 다리 canonical
  `[0.75°, 56.939125°, 90.613525°]`. 초기 S 보간 자세와 구분한다.
- V3의 Stand 명령, 보행 시작 진폭 0, 정상 정지 목적지, Stand 유지,
  `syncstate`의 Stand 인식이 같은 B 정의를 사용한다.
- Stand 완료 뒤 별도의 1초 A→보행 준비 단계 없이 보행 커널에 진입한다.
  측정 위치 확인·안전 감시·서보 속도/가속도 readback은 유지한다.
- 정지 시 마지막 자세 보정 포함 명령에서 B로 부드럽게 정리한다.
  첫 보행 프레임 전에 정지가 들어와도 초기 명령을 B로 설정해 두었다.
- 모델 선택 자체는 모터 이동 명령이 아니다. 다른 자세에서 V3를 선택한 뒤
  Stand를 누르거나 보행을 시작하면 측정 위치에서 B로 준비한다.
- 전진·후진·회전의 **정상 진폭 목표는 V2와 정확히 동일**하다. 앞발 +4mm 보정,
  다리 위상, 보폭, 속도, PD 이득, 서보 좌표 변환·부호·영구 설정은 유지한다.
  입력 방향/속도에 따른 V2의 보행 중 높이 변화는 여전히 존재한다.

앱 표시명은 `IMU 자세 안정화 V3 · 보행 대기 Stand · 실험`이다.
Mac/iPhone/Windows 목록 맨 위에 두고 지원 capability가 있는 연결에서 기본
선택한다. V78에서는 V2를 계속 선택한다. 시뮬레이터 기본도 V3이다.
펌웨어 프로필은 기존 인덱스 0–24를 유지하고 V3를 25번에 추가했다.

## 검증 중 발견한 공통 수치 오류

후진·회전 시 보정 OFF 상태에서도 planner 오류가 발생했다. 접지 가중치의
smootherstep 계산이 float 반올림으로 `1.0000006`을 만들 수 있어 유효 범위
0–1 검사에 걸렸다. 출력만 수학적 범위 0–1로 제한했다. 보행 궤적이나
PD 이득을 바꾸지 않는다. 100,001개 위상에서 가중치 범위를 회귀 검사한다.
초기 실패 기록과 수정 후 물리 시험 기록을 모두 artifacts에 보존했다.

## 검증 수준

- 호스트 회귀 테스트 173개 통과. V2/V3 전체 진폭 목표 23,331개 입력·위상 조합의 정확한 일치,
  모든 방향에서 진폭 0=B, 실제 C Stand/idle 목표, 시작 직후 정지 분기,
  감속 후 B, 기존 V2 Stand, 프로필 capability 및 TCP 명령 검증.
- 빌드: STM32 펌웨어, Mac Catalyst, iOS 빌드 성공. Swift 선택 테스트 3개 통과.
- 추정 물리 시뮬레이션: 전진·후진·좌회전·우회전 각각 Stand→4초 보행→정지.
  네 경우 모두 안전 오류 없이 Stand(B)로 종료했다. 목표 B 오차 0°,
  최종 물리 관절 오차 최대 0.88°. 전 구간 최대 roll 8.27° 미만,
  pitch 4.09° 미만. 실제 V2 시험과 같은 PD OFF/heading OFF 조건이다.
- 실기 설정 readback: V79 리비전, V3 기본 선택 및 capability 확인. 12개 서보 영구 레지스터 보존 확인.
- 실제 위치 유지 시험: V3 Stand(B) 유지 시험 미실시. 설치 전후 Landing 도착은 확인했다.
- 실기 전체 동작 시험: V3 미실시. V2의 이전 8걸음 관측과 혼동하지 않는다.

MuJoCo의 질량 분포·구동기·접촉은 추정 모델이다. 시뮬레이션 통과를 실물
접지 동기·발높이·보행 안정성 검증으로 해석하지 않는다. 실행 시 기존 CAD
mesh 연산의 numpy 경고가 있었으며, 자세/명령/물리 결과는 유한값이었다.

## iPhone 설치

사용자 요청으로 UJIN17에 `Spot OMG 0.5.0 (49)`를 서명 빌드·설치했다.
기기 앱 목록에서 빌드 49 readback 및 앱 실행 성공을 확인했다. 이후 사용자 요청으로
로봇도 V79로 업데이트했다. 앱에서 재연결하면 V3 capability를 인식한다.
설치 증거: `artifacts/attitudepd-v3/ios-install.json`, `ios-app-readback.json`, `ios-launch.json`.

## 실기 V79 설치

Landing 완료(오차 7틱) → 12개 서보 토크 OFF → BLE OTA SHA-256/Flash 검증 →
재부팅 순서로 설치했다. 설치 후 리비전 V79·기본 프로필 V3를 확인했다.

설치 후 Landing은 `complete-residual`로 완료됐다. 최초 검증 스크립트는
`complete` 문자열만 허용하여 중단됐으나, 로봇은 오류 없이 자세를 유지하고
있었다. 이동을 반복하지 않고 독립 `syncstate`/12개 `status`/`syncstate`로
최대 오차 22틱·모든 관절 정지·safety OK를 확인했다. 기존 펌웨어의 40틱
안정 잔여 오차 완료 기준에 맞춰 읽기 검증을 이어갔다.

12개 서보의 영구 레지스터 0–39가 설치 전과 동일했다. 마지막에 토크 OFF를
12개 모두 읽어 확인했고, 최종 상태는 Landing / torque OFF / safety OK /
fault 0이다. 토크 OFF 후 정착 오차는 78틱으로, 토크 ON 도착 오차와 구분한다.
Stand(B)나 보행은 실행하지 않았다.

설치 기록: `artifacts/attitudepd-v3/hardware-install/`의 `prepare.json`,
`ota.log`, `verify.json`(초기 검사 중단), `landing-readback.json`,
`finish-verify.json`(최종 성공).

## 재현과 결과

- `firmware/stm32-learning/tests/test_attitudepd_v3.py`
- `simulation/mujoco/tests/test_attitudepd_v3_controller.py`
- `simulation/mujoco/scripts/validation/validate_attitudepd_v3_stand.py`
- `artifacts/attitudepd-v3/regression-tests.log`
- `artifacts/attitudepd-v3/sim-stand-flow.json`
- `artifacts/attitudepd-v3/summary.json`
- `artifacts/attitudepd-v3/firmware/manifest.json`

전체 프레임 시뮬레이션 JSON과 앱 빌드 캐시는 로컬 artifacts에 보관하며, 저장소에는 요약·재현 스크립트·설치 원문·검증 결과를 저장한다.

실기 설치 시 저장소 규칙대로 Landing 실제 도착 확인 → 토크 OFF → 업데이트
→ 설치 후 Landing 확인 순서를 따른다.
