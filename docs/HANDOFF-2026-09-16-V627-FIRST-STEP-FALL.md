# V6.2.7 / v76 첫걸음 전도 — 다른 컴퓨터 인수인계

## 가장 먼저 읽을 상태

**V6.2.7 실기 보행은 실패했다.** v76 설치 이후 사용자가 “한걸음 걷더니
넘어졌어”라고 보고했다. 설치·30초 시뮬레이션 성공을 실기 보행 성공으로 취급하지 않는다.
사용자가 넘어진 로봇을 직접 세우고 Stand를 명령했으며, 이후 앱 연결도 해제했다고 확인했다.
마지막 읽힌 상태는 **Stand · torque=on · safety=ok · error=31ticks · fault_code=0**이다.
이는 사용자의 복구 후 상태이며, 넘어지지 않았다는 뜻이 아니다.

사용자 요청으로 이 시점에서 추가 구현/구동을 멈추고 문서·코드·결과를 Git에 보존한다.
이번 전도 이후 에이전트가 보낸 것은 기존 기록 조회 명령뿐이다.
추가 Stand/Landing/Recover/Relax, 보행 재실행, 펌웨어 재설치는 하지 않았다.
다른 컴퓨터에서 자동으로 재보행하거나 v76을 다시 설치하지 말고 먼저 아래 기록을 검토한다.

## 설치된 것과 변경 범위

- 브랜치 `develop`, 원격 `https://github.com/etnlwind/spot_omg.git`.
- 이전 기준 커밋 `a9fbba1`: 집에서 작업한 V6.2.5 및 배터리 변경.
- 현재 실기 STM32: **`s-native-v6-2-7-v76`**, 기본 프로필 `s_native_v6_2_7`(인덱스23).
  V6.2.6은 인덱스22로 보존했다. ESP32 브리지는 이번에 업데이트하지 않았다.
- v76 바이너리: `artifacts/s-native-v6-2-7/hardware-v76/firmware/s-native-v6-2-7-v76.bin`.
  315580bytes, SHA256 `c312a8ab7afdc2937690b92fb3529e2ed9ef09f0a91007de0ac8d404efdc133c`.
- PC/iOS 모델 목록 맨 위와 지원되는 연결의 기본값, 시뮬레이터 기본값은 V6.2.7이다.
  **현재 기본값이 실기 검증된 모델이라는 뜻은 아니다.** iOS는 소스만 갱신했으며 Mac 빌드/설치하지 않았다.
- 이번 커밋에는 중간 V6.2.6 연속 회수, v73 빠른 Stand, v74/v75 응답 측정,
  J1 고정·J2/J3 회수 실험 및 v76 설치까지 누적 변경과 원본 결과가 함께 포함된다.

## V6.2.7이 실제로 하는 동작

V6.2.6의 S, 전진+20mm/후진−125mm 추진 스트로크는 보존한다.
추진이 끝난 뒤 회수 X를 Beta(3,5) 누적 곡선으로 앞당겨 J2/J3가 접히면서
앞으로 계속 이동하도록 했다. 목표 X 역행이나 발 교차 대기를 넣지 않았다.
들기16mm, 명목 주기1.8초, 회수 처음/마지막25%에서 부드럽게 들고 내린다.
실제 주기는 입력과 기존 추종 감속에 따라 달라진다.

FR/RL 첫걸음은 동시에 내딛고 FR J1만 S 대비−18°로 오므린다.
두 번째 걸음에 모두 S 대비−9°로 전환한 뒤 주기별 J1 보정 잔차를 제거한다.
정지는 대각선 쌍 두 번의 발 옮김으로 S에 복귀하는 기존 방식을 보존한다.

직진 정상 회수 목표의 FR J2 최대72.19°, J3 최대102.68°다.
J2/J3는 시작67.78°/76.16° → 회수20%에서72.17°/89.46° →
40%에서63.80°/99.99° → 70%에서49.61°/102.52° → 착지42.76°/93.09°다.
이 값은 **코드/CAD 목표각**이고 실제 관절각이나 링크 내각과 구분해야 한다.

이전 J2=85°/J3=104° 그림은 정적 기하 예시이며 배포 궤적이 아니다.
큰 추가 접힘/들기 후보들은 동적 시험에서 기울기 보호 또는 접촉 증가를 보여 제외했다.
현재도 **실제 J3·IMU·몸체 높이를 사용해 쿠션5mm 여유를 보장하는 피드백 제약은 없다.**
목표각이 충분하다는 이유로 실제 발이 떴다고 판단하면 안 된다.

## 첫걸음 전도에서 확보한 증거

원본: `artifacts/s-native-v6-2-7/hardware-first-step-fall/collection01/`, `collection02/`.
모든 시각은 boot139의 uptime이며 PC 시각/영상 시각과 직접 같은 값이 아니다.

| 기록 | 내용 |
|---|---|
|117230ms|사용자 `stand` 명령|
|117986ms|Stand 성공 결과|
|120932ms|`drive 344 0 1` 최초 입력|
|124404ms|`RESULT result=tilt safety limit reached`|
|실제 보행 구간|`gaitdiag` elapsed=1797ms, samples=180, 각 서보15회|
|151275ms|후속 Stand 성공 결과(사용자가 일으켜 세운 뒤 명령했다고 설명)|
|191084ms|별도 `drive -965 181 19` 명령 기록|
|191347ms|그 후 `motion aborted` 결과|

첫 전도의 시작 입력은 **344/0**이었다. 앞서 대표 시뮬레이션은1000/0이었다.
실시간 `@D` 변화는 이 영구 로그에 모두 저장되지 않으므로 전체 보행이344로
고정됐다고 확정하지 않는다. 후속 음수 입력을 최초 전도 원인으로 섞지 않는다.
후속263ms 중단 뒤 같은180개 진단이 출력되므로 새 보행 데이터인지 이전 데이터인지
구별해야 한다. `robot_shared_drive()`는 Stand/프로필 준비 뒤에야 진단을 초기화한다.

`gaitdiag` 및 저장된 MECH 요약에서 관측된 최대 추종 오차:

| 관절 | 최대 오차 | 해당 요약의 문맥 |
|---|---:|---|
|FL J3|114ticks ≈10.02°|회수, phase109, 목표2942/실측2828|
|FR J1|56ticks ≈4.92°|지지, phase301, 목표2227/실측2283|
|FR J3|104ticks ≈9.14°|지지, phase311, 목표1105/실측1001|
|RR J2|75ticks ≈6.59°|회수, phase281, 목표2882/실측2957|
|RR J3|109ticks ≈9.58°|회수, phase137, 목표1015/실측1124|

각 행은 **다른 조회 시점의 최고 오차**다. 동시에 발생한 자세로 합성하지 않는다.
FR J1은 phase301에서 정상−9°로 돌아가는 도중 아직 더 오므려진 실측값이었다.
첫걸음/두 번째 걸음 전환과 관절 추종을 함께 검토할 근거는 되지만,
이것만으로 전도 원인이 J1 또는 J3 하나라고 확정할 수는 없다.

최저 관측 전압10.5V, lag4, lag+droop1, late_frames0, 버스 재시도0이 남았다.
전압 강하가 원인인지 부하 증가의 결과인지 아직 모른다.
`GAIT fall=no/0`와 실제 `tilt safety limit reached`가 함께 나온다.
native 경로의 진단 플래그 연결을 확인해야 하며 `fall=no`를 전도 부정 근거로 사용하지 않는다.

### 로그의 한계

- `jointtrace status`는 `$JT,M,1,0,0,0,0,0,0,0`: 상세 시계열이 사전 활성화되지 않았다.
  첫 전도의 전체 목표/실측/몸체 시계열은 없다. 나중에 arm해도 과거 기록을 복원할 수 없다.
- `log show 128/512` 응답에 시퀀스 번호 공백이 있다. 전송 누락인지 저장 누락인지
  확정하지 않았다. BEGIN/END가 있어도 완전한 로그라고 간주하지 않는다.
- `gaitdiag`는 두 번 모두 끝 prompt 대기에서 timeout이었다. collection02는
  받은25개 줄을 `gaitdiag-stream.txt`에 즉시 저장하여 보존했다. 마지막 부분은 없다.
  뒤에 예정했던 baldiag/trackingdiag/status/servoconfig 수집은 수행되지 않았다.
- `gaitdiag`는 모터를 움직이지 않지만 MECH 요약을 영구 로그에 다시 추가한다.
  조회 시각의 새 MECH 기록을 새로운 보행으로 해석하면 안 된다.
- 원인 분석과 실제344 입력 조건의 시뮬레이션 재현은 **미완료**다.

## 이미 검증된 범위와 아직 아닌 것

설치 전 C/Python 목표 비교와 관련 Python123개, C 호스트8그룹이 통과했다.
실제 C 커널의 추정 물리30초 직진은 최대 기울기6.20°, 최종 S 오차0.99°로 종료했다.
Python30초에서는 위상 정지0초였으나 회수 중반 접촉이 각 발 약15~18% 남았다.
우회전 입력8초는 yaw 변화가+0.93°뿐이고 후진8초는 yaw 편차+7.77°였다.
회전 기능/끌림 없는 보행에 합격한 결과가 아니다.

v76 설치는 Landing 도착 → Relax →12개 토크 주소40=0 확인 →OTA →
버전/기본 프로필 readback →Landing 재확인까지 완료했다.
설치 직후 설정 주소0..39가 준비 시점과 동일했다. **사용자가 이후 수행한 보행은 전도 실패다.**
독립적인 실기 위치 유지 시험 및 정상적인 실기 Stop→S는 이번 모델에서 미검증이다.

이전 v74/v75의 RR J3 바닥 작은 각도 응답 시험도 가속도 식별에 실패했다.
시뮬레이터의 가속도/20ms 지연 가정은 실증값으로 취급하지 않는다.
원시 목표 증가가 물리적 접힘인지 사전 확인이 부족했던 한계도 문서에 남겼다.
Landing은 J3의 기구적 최대 접힘으로 토크 없이 지지되는 자세라는 사용자 설명을 지킨다.
정적 기구 끝점에서 안 움직이는 현상을 가속도 부족으로 해석하지 않는다.

온도 보고의 간헐적 이상값도 남아 있다. 설치 후 Landing 중137°C 표본이 재확인에서
정상으로 돌아왔고 최종 전체 조회는31~37°C였다. 실제 과열/센서 정상 어느 쪽도
이 한 번의 이상값만으로 확정하지 않는다.

## 다음 컴퓨터에서 진행할 순서

1. `AGENTS.md`, [STS3250 실기 기준](STS3250-POSITION-CONTROL.md), 이 문서를 먼저 읽는다.
2. 위 최초344/0 입력과 FR 첫걸음 전환을1000/0 대표 시험과 비교한다.
   전도 전 몸체 기울기와 실제 J2/J3·FR J1의 도달 상태를 확보할 방법부터 정한다.
3. 로그 전송 누락/prompt timeout, 오래된 진단 재출력, native fall 플래그를 확인한다.
   새 상세 기록 수단을 준비해도 승인 없이 다시 걷게 하지 않는다.
4. 제어 변경은 시뮬레이터와 C에 같은 방식으로 반영해 비교한다. 보호 기준을 높이거나
   충돌·온도 보호를 끄는 식으로 통과시키지 않는다. 아직 root cause를 확정하지 않았다.
5. 실기 재시험은 사용자의 바닥 지지/전도 대비와 연결 상태를 새로 확인한 뒤
   짧은 출발 동작에서 실제 반응을 관측한다. 긴 보행이나 새 OTA를 바로 시작하지 않는다.

## 환경·실행 명령

이 컴퓨터는 `D:/project/spot_omg`, Python은
`C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe`다.
다른 컴퓨터에서는 자기 경로를 사용하고 저장소 루트에서 실행한다.

```powershell
git pull --ff-only origin develop
conda env create -f config/environment.yml
conda activate spot_omg
python -m pip install -r apps/windows/requirements.txt
python -m pip install pytest pyinstaller
python -m pytest simulation/mujoco/tests/test_s_native_v627.py -q
python simulation/mujoco/scripts/validation/validate_s_native_v627.py
python apps/windows/main.py
```

`validate_s_native_v627.py`는 모터를 움직이지 않는 오프라인 C/MuJoCo 재생이다.
호스트 C 컴파일러가 필요하다. Windows의 `.toolchain/zig`는 Git에 포함되지 않으므로
`servo.host_build` 및 기존 설정을 확인하고 설치한다.
Windows 앱 빌드 명령은 `./apps/windows/build.ps1 -Python python -DistPath "$PWD/apps/windows/dist/v627"`이다.
`apps/windows/dist/` 실행 파일은 Git 제외 대상이며 다른 컴퓨터에서 다시 빌드해야 한다.
MuJoCo/FFmpeg 및 CAD 모델 경로는 [폴더 구조](PROJECT-LAYOUT.md)를 참고한다.

## 관련 기록 위치

- [V6.2.7 설치·기하·검증](S-NATIVE-V627-INSTALL-2026-09-16.md)
- [V6.2.6 인수인계와 서보 모델 수정](HANDOFF-2026-09-16-V626-CONTINUOUS-RECOVERY.md)
- [v73 빠른 Stand](FAST-STAND-V73-2026-09-16.md)
- [실기 가속도 측정의 한계](SERVO-ACCELERATION-VALIDATION-2026-09-16.md)
- [J2/J3 쿠션 최저점 계산](S-NATIVE-J2-J3-CLEARANCE-2026-09-16.md)
- [r3 회수 X 역행 제거](S-NATIVE-RECOVERY-MONOTONIC-2026-09-16.md)
- 원본 실기: `artifacts/servo-acceleration-validation-2026-09-16/`,
  `artifacts/s-native-v6-2-7/hardware-v76/`, `hardware-first-step-fall/`.
- 영상·실험: `artifacts/s-native-v6-2-7/` 아래 `recovery-monotonic`,
  `j1-hold-simulation`, `post-push-video`, `registered-v627` 등.

사용자 요청: 시행착오를 숨기지 않고 기존 모델/후보/실제 실패를 보존한다.
새 정책은 기존 이름을 덮어쓰지 않고 새 버전으로 추가하며 펌웨어 인덱스를 유지한다.
