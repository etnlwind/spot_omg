# V67: S 진입 후 보행 속도 설정 복구

## 원인

V6.2.4/v66 실기에서 `profile` 명령은 speed=3400, acceleration=254를
출력했지만, `servoconfig`로 읽은 서보 12개의 실제 RAM 레지스터는 모두
speed=300, acceleration=30이었다.
[원시 readback](../artifacts/s-native-v6-2-4/hardware-pilot-8s-retry/servo-registers.txt).

`robot_shared_drive()`는 시작할 때 `robot_stand()`로 S 자세에 진입한다.
일반 자세 진입은 서보에 300/30을 기록한다. 이어지는 보행 프레임은 위치만
갱신하므로 낮아진 속도/가속도 설정이 남았다. `robot_set_profile()`과
`profile` 조회는 소프트웨어의 요청값만 다뤘다. 요청값을 실제 적용값으로
보고한 이전 설명은 잘못됐다. 시험 스크립트의 선행 `stand`도 중복이었다.

보행 궤적의 별도 준비 단계와 함수 진입 시 S로 맞추는 동작은 구분해야 한다.
V67은 중복 외부 `stand`를 제거했지만 내부의 감독된 S 진입은 유지한다.
`$SPOTDRIVE started`가 S 진입보다 먼저 출력되므로 호스트 입력 시간은
실제 보행 시간과 같지 않다.

## 수정

- S 진입 성공 후 `sts3215_sync_profile()`로 가속도 주소 41과 속도 주소
  46/47만 기록한다. 위치, 토크, 영점, J2 누적 좌표는 변경하지 않는다.
- 12개 서보를 읽어 요청값과 일치해야 보행 위상을 진행한다. 통신 실패나
  값 불일치는 보행 시작 전에 오류를 반환한다.
- simulator/mock에 서보별 지속되는 속도/가속도 레지스터를 추가했다.
  속도는 tick/s, 가속도는 100 tick/s² 단위로 내부 목표 제한에 반영한다.
  기존 모터 토크·속도 한계와 충돌도 유지한다.
- 가상 로봇의 `profile`은 요청값, `servoprofile`은 적용값을 표시한다.
  `firmware_servo_profile_restore=false`로 이전 누락을 재현할 수 있다.

레지스터 주소 근거: [제조사 SMS_STS.h](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.h).
단위 근거: [제조사 입문 설명서](https://www.feetechrc.com/Data/feetechrc/upload/file/20220618/%E5%85%88%E7%9C%8B%E8%BF%99%E9%87%8C-%E5%85%A5%E6%89%8B%E6%95%99%E7%A8%8B2-1.pdf).

## 검증 수준

호스트 C 검사는 자세 이동→위치만 갱신했을 때 300/30이 남는 상황과
프로필 복구, 위치·토크 보존, 통신 실패를 검사한다. Python 관련 검사 67개가
통과했다. 펌웨어 빌드는 325220 bytes로 슬롯 안에 들어간다.

동일한 실기 명령 기록을 고정 몸체 모델로 재생한 평균 관절 RMS 오차:

| 실기 기록 | 3400/254 가정 | 실제 300/30 반영 |
|---|---:|---:|
| v65 | 7.066° | 1.121° |
| v64 별도 기록 | 8.946° | 1.380° |
| v66 별도 기록 | 2.269° | 0.858° |

이는 서보 설정 누락에 대한 근거다. 내부 제어기·마찰·접촉·관성 전부가
실측 보정됐다는 뜻은 아니다. 몸체 고정 시뮬레이션은 최대 추종 오차
8.296°, 내부 접촉 0, 최종 S 오차 0.219°였다. 바닥 시뮬레이션은
보행 시작 약 2.08초 뒤 기울기 보호로 중단됐다. 바닥 보행 성공은 미검증이다.

실기 설치, 실제 레지스터 readback, 정지 자세 유지, 전체 바닥 보행은 별도의
검증 단계로 보고한다. 실제 시험 결과는 후속 handoff에 기록한다.

## 재현 자료

- [V67 결과 폴더](../artifacts/servo-profile-restore-v67/)
- `scripts/analysis/replay_servo_profile_trace.py`: 실제 명령 재생 비교
- `scripts/analysis/compare_supported_joint_traces.py`: 실측 관절 범위·속도 비교
- `scripts/analysis/estimate_lower_leg_angle.py`: 순차 관절 피드백을 CAD로 환산

관절별 약 240ms 간격의 순차 피드백에서 얻는 속도는 샘플 간 평균이며,
최고 속도나 동시 관절각을 직접 측정한 값이 아니다. CAD 하부 링크 각도는
몸체가 수평이라는 가정도 포함한다. 사용자 사진의 바닥 기준 각도를 대체하지 않는다.

## V67 실기 실패와 V68 후속 수정

V67 설치 후 두 시험 모두 보행 전 프로필 검증에서 중단됐다(ID4, ID1).
두 번째 시험 후 readback은 speed=3400이 12개 모두 적용됐지만,
ID1/3/7/9/10의 acceleration=50, 나머지는 254였다.
[실측](../artifacts/servo-profile-restore-v67/hardware-pilot-10s-retry/servo-registers.json).
따라서 V67은 속도 복구 구현을 실기 성공으로 인정할 수 없다.

V68은 주소 41과 46의 별도 쓰기 대신 기존 위치 제어와 같은 41..47
전체 블록을 보낸다. 각 서보의 기존 목표 위치와 시간 바이트를 먼저 읽어
그대로 보존한다. PRESENT_POSITION에서 누적 목표를 재구성하지 않는다.
12개 readback이 일치해야 출발하는 조건은 유지한다. 별도 쓰기가 가속도를
50으로 만드는 정확한 서보 내부 원인은 아직 확정하지 않았다.

서명된 목표 위치·시간 바이트 보존 C 회귀 및 실제 혼합 가속도 readback을
개별 모터 제한으로 반영하는 simulator 회귀를 추가했다. V68의 실기 결과는
설치 후 시험 기록으로 별도 확인해야 한다.
