# V6.2.1 실기 설치 후 인수인계 — 2026-09-15

## 재개 시 가장 먼저 알아야 할 상태

**실제 로봇에 V6.2.1 / STM32 v63을 설치했다. 이후 사용자가 “다리를 끌다가 넘어지네”라고 보고했다. 실기 보행은 성공한 상태가 아니다.**

넘어짐 보고 뒤 원인 분석을 시작하려 했으나, 사용자가 작업을 중단하고 집의 다른 노트북에서 이어가기 위해 이 문서를 요청했다. 넘어짐 이후의 새 실기 로그 수집·원인 확정·궤적 수정·재설치는 수행하지 않았다. 이 문서 작성에서는 코드 동작을 변경하거나 로봇에 명령을 보내지 않았다.

- 현재 설치된 모델: `s_native_v6_2_1`, 펌웨어 revision: `s-native-v6-2-1-v63`.
- 마지막 **넘어짐 전** 확인은 토크 OFF, safety OK, fault 0이었다. 이를 현재 상태로 재사용하지 않는다.
- 넘어짐 이후 몸체 지지·정지·앱 연결 해제 여부에 대한 질문은 답을 받기 전에 작업이 중단됐다. 새 작업에서 현재 상태를 확인한다.
- 가장 먼저 해결할 문제는 **회수 중 실제 발 끌림과 그 뒤의 넘어짐**이다. 기존 교차 감속 수정이나 C/Python 일치만 다시 확인해서 완료로 판단하면 안 된다.
- 최신 인계 기준은 이 문서다. [이전 V5 인계](HANDOFF-2026-09-15-S-NATIVE-V5.md)의 기본 모델·설치 버전·다음 작업 설명은 당시 기록이다.

## Git과 다른 노트북으로 전달할 내용

2026-09-15 문서 작성 시 확인:

- 저장소: `https://github.com/etnlwind/spot_omg.git`
- 브랜치: `develop`
- 로컬 HEAD와 실제 원격 `develop`: **`0d8b0d72223f37a65239699909b536fc4c9b2276`**
- 커밋 제목: `Add V6.2.1 gait support and graceful walking stop`
- V6.2.1 소스, v63 빌드 산출물과 기존 시뮬레이션 영상은 위 커밋에 들어 있다.
- 위 해시는 인계 문서 작성 전 코드 기준이다. 이후 사용자가 커밋·푸시를 요청했으며, 아래 설치 후 로그와 인계 문서를 후속 커밋에 함께 포함한다. 새 노트북에서는 최신 `develop`을 받는다.

다음 자료가 이번 인계 커밋의 대상이다. 새 노트북에서 이 문서와 설치 후 로그가 함께 있는지 확인한다.

| 파일 | 전달해야 하는 이유 |
|---|---|
| `docs/HANDOFF-2026-09-15-V621-REAL-ROBOT.md` | 이 인계 문서 |
| `docs/HANDOFF-2026-09-15-S-NATIVE-V5.md` | 최신 인계 문서로 연결하는 안내 |
| `docs/S-NATIVE-V621-FIRMWARE.md` | 설치 성공 및 이후 실기 넘어짐 보고 |
| `apps/windows/README.md` | v63 실기 연결/모델 선택 확인 |
| `artifacts/s-native-v6-2-1/hardware-deployment/ota-v63.log` | OTA 최종 완료까지의 로그 |
| `artifacts/s-native-v6-2-1/hardware-deployment/after-ota.log` | 설치 직후 revision·토크·안전 상태 |
| `artifacts/s-native-v6-2-1/hardware-deployment/servo-status.log` | 설치 직후 12개 서보 조회 |
| `artifacts/s-native-v6-2-1/hardware-deployment/deployment.json` | 설치 결과 요약 |

`git pull`로 전달되는 것은 원격에 커밋된 파일뿐이다. 로컬 Python 환경, Windows 실행 파일 `apps/windows/dist/`, `.toolchain/`, 앱 설정·연결 상태, 로봇 RAM 진단은 자동으로 옮겨지지 않는다. 현재 PC 경로 `D:/project/spot_omg`와 Python 경로를 새 노트북에 그대로 쓰지 않는다.

## 지금까지 만든 기능과 버전 구분

Windows 앱은 iPhone 앱의 BLE 콘솔·조이스틱에 대응한다. 실제 로봇 BLE와 로컬 MuJoCo TCP/영상 연결을 지원한다. 표기는 Spot OMG이며 키보드 입력 시 조이스틱 표시 갱신, 방향키의 앱 스크롤 방지, 전체 UI를 고려한 기본 창 크기, 별도 3D 뷰어 실행을 반영했다.

| 보행 모델 | 현재 의미 |
|---|---|
| V1–V5 | S 기준 출발, 대각선 교대, FR 첫걸음 추가 오므림 등을 발전시킨 보존 모델. 당시 안정 보행 실패 기록을 보존한다. |
| V6 | FL/RR 전환 시 충격을 줄이기 위해 명목 높이 12mm, 주기 1.2초로 조정. |
| V6.1 | 최대 입력에서 S 기준 앞 +20mm / 뒤 −65mm. 앞 J1 실기 방향 수정, yaw 부호 정리, 두 걸음 Stop 복귀를 반영. |
| V6.2 | V6.1 보존 후 뒤 −125mm로 확대, 기본 주기 2초, 후진 60% 제한. 사진의 뒷다리 형상 참고. 시뮬레이터 전용. |
| V6.2.1 | V6.2 보존 후 교차 구간 감속 완화와 이동 중 J3 회수 곡선 수정. 현재 앱·시뮬레이터·실기 v63의 기본 모델. **실기 발 끌림·넘어짐 미해결.** |

모델 버전과 펌웨어 번호를 혼동하지 않는다. `v60`은 앞 J1 수정이 들어간 V6.1 실기 펌웨어였고, `v61/v62` 펌웨어 후보에는 V6.1 yaw/Stop 보완이 들어갔다. 이번 실기는 v60에서 **v63**으로 설치했다. v61/v62가 실기에 설치됐다고 가정하지 않는다.

## 유지해야 할 사용자 조건과 좌표 규약

수정 전에 [AGENTS.md](../AGENTS.md), [STS3250 위치 제어 실기 기준](STS3250-POSITION-CONTROL.md), [폴더 구조](PROJECT-LAYOUT.md)를 읽는다.

1. **S 정지 기준:** 쿠션 접지점 X가 각 R2/J2 축 중심 X에 맞는다. S에서 출발한다.
2. **대각선 교대:** FR·RL / FL·RR의 상대 X/Z와 타이밍을 각각 공유한다. 서로 반 주기 차이다. 목표 위상과 실제 접지 동기는 별도 검증한다.
3. **첫걸음:** FR·RL이 함께 내딛는다. FR만 S 대비 정상 J1 변화의 2배로 오므린다. 첫걸음의 나머지 J1은 S 유지, 다음 걸음에 정상 오므림으로 합류한다. 정상 기준은 간섭 검토 후 9°, 첫 FR 18°로 제한했으며 동적 보정은 별도다.
4. **교차 시 대기 금지:** 발이 교차하는 곳에서 제자리에 기다리는 동작을 재도입하지 않는다.
5. **회수:** 앞으로 옮기는 동안 자연스럽게 무릎을 접었다 펴야 한다. 목표점에 도달한 뒤 펴거나, 먼저 들어 올리고 정지한 다음 앞으로 옮기는 식으로 분리하지 않는다.
6. **정지:** 신호가 오면 제자리에서 대각선 한 쌍씩 “하나, 둘” 내딛으며 J1까지 한 번에 S 목표로 맞춘다. 정지 후 바닥에 댄 채 천천히 벌리는 별도 단계를 넣지 않는다. [Stop 구현과 접지 한계](S-NATIVE-STOP-PLACEMENT-2026-09-15.md).
7. 사진의 2D 투영각을 실제 J2/J3 각도로 취급하지 않는다. J3 서보각과 링크 사이 안쪽 각도도 구분한다.
8. 앞 J1의 CAD→실기 변환은 `s_native_servo.h`에 있다. FL ID1 감소 / FR ID4 증가가 오므림이다. 이를 다시 반전하거나 저장된 영점·방향을 변경하지 않는다.
9. J2(STS3250)의 누적 목표 틱과 0–4095 센서 피드백을 구분한다. 목표를 무조건 `%4096`으로 만들지 않는다. 기존 Stow/Landing 좌표와 안전 한계를 보존한다.
10. 새 모델은 기존 이름을 덮어쓰지 않고 버전으로 추가한다. 지원 시 앱 맨 위·기본, 시뮬레이터 기본도 갱신한다. 기존 펌웨어 프로필 인덱스는 변경하지 않는다. 현재 V6.1=16, V6.2.1=17이다. 다음 버전 이름은 아직 정하지 않았다.

총중량은 배터리·쿠션 포함 **2.754kg**, 쿠션은 외경 **37.3mm**, 전체 길이 **27mm**다. 질량 분포·접촉 물성·서보 응답은 추정값을 포함한다. 설정은 `simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json`을 사용한다. 실제 한계/추종 특성을 simulator/mock에도 반영해야 한다.

뷰어는 현재 PC에서 LG FULL HD 모니터를 사용했다. 기본 측면 시점은 앞쪽이 화면 오른쪽, azimuth=90°, elevation=0°, distance=1.05m, 추적 중심 COM−0.1m다. 새 노트북에서는 모니터 구성을 확인한다.

## V6.2.1의 실제 구현

- 명목 전진 끝점 +20mm / 후진 끝점 −125mm, 기본 주기 2초, 명목 최대 발 높이 12mm.
- 교차 감속: 뒤쪽에만 적용하던 추가 변위를 전체 왕복 구간에 분산했다. `c`를 부드러운 왕복 위상값이라 할 때 직진 X는 `0.020*linear*c - 0.105*linear*((1-c)/2)^3`이다.
- 회수 높이는 `12mm * sin(pi*swing)^0.25` 형태. 이전 지수 0.5에서 변경했다. 높이 최대값 자체는 늘리지 않았다. 진입 보폭 램프와 첫 발 높이는 분리한다.
- J2/J3를 함께 IK로 풀며, 이전 버전에서도 J3가 고정각인 것은 아니었다.
- 두 걸음 Stop은 1.6초(한 쌍당 0.8초), 이후 S 유지까지 완료 응답은 정상 시험에서 영상 약 12.6초에 나왔다.
- 실시간 native 경로는 기존 IMU 균형 보정이 `balance=suspended`다. 지지 예측 궤적과 heading 유지는 있지만, 앱 Balance 버튼을 켜면 Roll/Pitch 문제가 해결된다고 가정하지 않는다.
- C 이식은 기존 V6.1 표를 보존하고 V6.2.1 지지 표만 Fourier 8차 계수(7,854바이트)로 압축했다. 입력 변경 시 64위상을 복원한다.

## 검증한 것과 검증하지 못한 것

| 수준 | 결과 |
|---|---|
| 명목 궤적 | 교차 속도 75.0→222.7mm/s, 최고 속도 460.8→350.7mm/s. 몸체 전진 속도가 아니라 몸체 기준 발 목표 속도다. |
| 호스트·공용 C·Windows 프로토콜 | 관련 검사 97개 통과. 안전/Stow 좌표/Stow 동작/자세 감독/명령 복구 C 검사 5개 통과. |
| C/Python 목표 일치 | V6.2.1 최대 차이 약 0.0334°. 실제 서보 인코딩 범위도 확인. 일치는 현실 추종 성공을 뜻하지 않는다. |
| MuJoCo 보호 재생 | Python 8조건, C 6조건에서 보호 중단 없이 S 복귀. 최종 시뮬레이션 관절 오차 1.1° 미만. |
| C 전진 100%, 8초 | X 1.119m, yaw −0.275°, 최대 기울기 5.419°. 추정 물성 조건이다. |
| 실기 설치/readback | v63 설치·검증·재부팅 성공. 모델/capability/후진 제한 확인. 12개 서보 응답 정상. |
| 독립 실기 위치 유지 | 이번 v63에서 수행하지 않았다. |
| 실제 전체 보행·정지 | 사용자가 발을 끌다 넘어짐을 보고했다. 성공한 실기 Stop→S 복귀는 검증하지 못했다. |

**발 끌림은 시뮬레이터에서도 남아 있었다.** 영상 4–10초, 회수 진행률 20–90%, 50Hz, 네 발을 합친 420개 표본에서 접촉 하중 0.5N 초과 표본은 V6.2 117개→V6.2.1 57개다. 최소 쿠션 지면 간격은 V6.2 −0.920mm / V6.2.1 **−1.176mm**다. 접촉 표본 수는 줄었지만 최악 침투량까지 개선됐다는 뜻은 아니다.

기존 통과 기준은 주로 보호 중단·기울기·최종 S 오차였다. 회수 중 무접촉을 통과 조건으로 보장하지 않았다. 다음 검증에는 실제 발 높이/하중과 관절 추종을 포함해야 한다.

이미 시험한 높이 18–24mm 확대, 일부 주기 단축, 지지 변위 제한 확대, 단순 자세 피드백 후보들은 실패하거나 악화되어 채택하지 않았다. 같은 값을 다시 임의로 늘려 해결됐다고 판단하지 않는다. 실패 후보와 근거는 기존 `artifacts/s-native-v6-2/recovery-fix/` 및 V6.2.1 결과에서 확인한다.

이전 v60 로그에서는 J2 최대 목표/실측 차이 약 9–13°와 기울기 보호가 관찰됐다. 이 수치를 이번 v63의 측정값으로 쓰지 않는다. 목표 이력의 `match_age` 역시 실제 지연 시간 확정값이 아니다. [v60 분석](../artifacts/s-native-v6-1/log-analysis-2026-09-15/REPORT.md).

## 실기에 설치한 파일과 기록

- 설치 시각: **2026-09-15 18:03 KST**.
- BLE: `SpotOMG-Bridge`, Nordic UART 서비스 `6e400001-b5a3-f393-e0a9-e50e24dcca9e`.
- 설치 파일: `artifacts/s-native-v6-2-1/firmware-v63/s-native-v6-2-1-v63.bin`
- 크기: **315,816바이트**, STM32 OTA 앱 슬롯 320KiB 이내.
- SHA256: `5a02233bc67be29aebf5c957a7ebc3e58b529c1eea6d034fa9b137e05311d9d6`
- 설치 순서: 사용자 몸체 지지·앱 해제 확인 → CLI `relax` → 토크 OFF 조회 → **`--skip-landing`** OTA → 재부팅 후 상태/12개 서보 조회.
- 설치 직후: `profile=s_native_v6_2_1`, `reverse_limit=600`, `heading=on`, `mass_g=2754`, `safety=ok`, `fault_code=0`, `torque=off`.
- 12축 조회: 하드웨어 오류/읽기 실패 0, 전압 11.3–11.5V, 온도 30–40°C. 이는 넘어짐 전 관측이다.
- Windows 앱을 다시 빌드·실행했고, 사용자가 실기 BLE로 연결한 뒤 v63/V6.2.1 표시와 선택을 확인했다.
- iOS는 소스 반영만 했다. iPhone에 새 앱을 설치하려면 Mac/Xcode 빌드가 필요하다.

상세 기록: [펌웨어/설치 문서](S-NATIVE-V621-FIRMWARE.md), [설치 로그 폴더](../artifacts/s-native-v6-2-1/hardware-deployment/).

## 집 노트북에서 준비·실행

모든 명령은 저장소 루트에서 실행한다. 기존 체크아웃은 `git status`로 작업을 먼저 확인하고 보존한 뒤 `git pull --ff-only origin develop`로 갱신한다. 새 체크아웃은 다음과 같다.

```text
git clone --branch develop https://github.com/etnlwind/spot_omg.git
cd spot_omg
conda env create -f config/environment.yml
conda activate spot_omg
```

Windows:

```powershell
$env:PYTHONUTF8='1'
./apps/windows/setup.ps1
./apps/windows/run.ps1
```

앱의 프로젝트/Python 경로는 새 노트북 경로로 지정한다. MuJoCo를 앱 밖에서 띄우려면:

```powershell
python -X utf8 simulation/mujoco/virtual_robot.py --viewer --no-ble --host 127.0.0.1 --video-host 127.0.0.1 --profile s_native_v6_2_1
```

앱의 `MuJoCo 시작 + 연결`을 사용하면 별도 실행은 불필요하다. 제어 TCP 8765 / 영상 8766이다. 가상 로봇 BLE는 기존 Mac 브리지용이며 Windows 로컬 시뮬레이터는 TCP로 연결한다. 앱 연결 시 지원하는 최신 모델을 기본 선택하므로 구버전 비교에서는 앱 모델 선택도 확인한다.

Windows 실행 파일이 필요하면 앱을 닫고 `python -m pip install pyinstaller` 후 `./apps/windows/build.ps1`로 빌드한다. 배포는 `dist/SpotOMGController` 폴더 전체가 필요하다. MuJoCo/CAD/Conda 환경은 exe에 포함되지 않는다.

Mac에서 재개한다면 동일 Conda 환경을 준비하고 뷰어 실행에 `python` 대신 `mjpython`을 사용한다. Windows exe는 실행되지 않는다. Catalyst/iOS 빌드는 이전 인계 문서와 iOS 프로젝트 안내를 참고하되 보행 모델은 현재 V6.2.1로 확인한다.

현재 Windows 측 버전: Python 3.10.20, MuJoCo 3.11.0, NumPy 2.2.6, PySide6 6.11.2, Bleak 1.1.1, Pillow 12.3.0, pytest 9.0.3, PyInstaller 6.22.3. 호스트 C 검증은 Zig 0.15.2, 펌웨어 빌드는 STM32CubeIDE 2.2.0에 포함된 ARM GCC 14.3을 사용했다. 새 환경의 버전도 결과와 함께 기록한다.

## 다음 작업 순서

1. **넘어짐 증거부터 보존한다.** 현재 앱 콘솔을 저장하고, 가능하면 재부팅/새 보행 전에 로봇의 `gaitdiag`, `baldiag`, `jointtrace status`, 저장 로그를 읽는다. RAM trace는 재부팅/새 구동으로 사라질 수 있다. 이번 사건은 아직 수집하지 않았다.
2. 로봇 몸체 지지·현재 토크·앱 연결 상태를 확인한다. 이전 설치 때 받은 지지 확인을 새 시험의 준비 상태로 간주하지 않는다. BLE 제어 연결은 한 클라이언트만 사용한다.
3. 사용자가 본 어느 발/몇 번째 걸음/입력 크기/바닥/지지 조건인지 확인한다. J2/J3 목표와 실측, 실제 회수 높이, 접촉, IMU를 시간축으로 대조한다. 관절 순차 조회를 동시 표본처럼 해석하지 않는다.
4. **원인을 구분한다.** 낮은 명목 회수 높이, 몸체 기울기로 잃는 여유, 하중 중 서보 추종, 접지 교대 타이밍, 전원/센서 이상을 후보로 두되 v63 로그 없이 하나로 단정하지 않는다.
5. 실제 관측한 제약을 시뮬레이터/회귀 시험에 재현한다. 회수 진행 중 발 높이와 하중, 전진 중 끌림, 내부 간섭, 기울기, S 복귀를 함께 검사한다. 목표 궤적·C 일치만으로 통과시키지 않는다.
6. 이동 중 접었다 펴는 사용자 요구를 유지하며 수정 후보를 검증한다. 새 버전과 기존 V6.2.1을 분리하고, 이미 실패한 후보를 배포본과 혼동하지 않는다.
7. Python 변경을 C로 옮기면 지지 표·프로필 헤더·앱 지원 판별·기본 모델까지 함께 갱신한다. 원래 프로필 인덱스/영점/관절 부호/안전 한계를 보존한다.
8. 호스트/시뮬레이터 결과와 실기 설정 조회, 독립 위치 유지, 전체 보행/Stop 검증 결과를 구분해 보고한다. 새 후보 검증 전 이 v63을 다시 설치할 필요는 없다.

### 읽기 전용 실기 진단 예시

앱에서 실제 로봇 연결을 해제하고, 안전하게 정지시킨 뒤 사용한다. 아래 명령은 새 보행을 시작하지 않는다. 진단 조회 자체도 로그에 남을 수 있으므로 사건 시각을 구분한다. 결과 폴더는 새 이름을 사용한다.

```powershell
$caseDir='artifacts/s-native-v6-2-1/home-fall-investigation'
New-Item -ItemType Directory -Force -Path $caseDir | Out-Null
python -X utf8 -m servo.cli --via ble --no-app-control --log "$caseDir/state.log" console send syncstate
python -X utf8 -m servo.cli --via ble --no-app-control --log "$caseDir/gaitdiag.log" gaitdiag
python -X utf8 -m servo.cli --via ble --no-app-control --log "$caseDir/baldiag.log" baldiag
python -X utf8 -m servo.cli --via ble --no-app-control --log "$caseDir/jointtrace-status.log" console send jointtrace status
python -X utf8 -m servo.cli --via ble --no-app-control logs --count 512 --output "$caseDir/flight-log.txt"
```

대량 로그는 행 수·레코드 연속 번호·출력 종료까지 확인한다. `jointtrace`가 있으면 `tools/servo_tool/servo/joint_trace.py`의 페이지 단위 `download()`를 사용한다. 새 기록 시작·삭제는 기존 사건 로그를 회수한 뒤에 한다.

OTA의 `--skip-landing`은 토크 OFF를 확인하고 자세 이동을 생략한다. 생략하면 도구가 Landing을 요청한다. 다음 설치도 실기 지지 상태와 토크 OFF를 확인한 뒤 진행한다. PC 앱의 Relax는 현재 프로토콜에서 Landing을 먼저 요청하므로 CLI의 `console send relax`와 같은 동작으로 간주하지 않는다.

## 코드와 근거 위치

| 목적 | 파일/폴더 |
|---|---|
| Python 궤적·진입·Stop·지지 예측 | `simulation/mujoco/runtime/s_native_gait.py` |
| CAD 쿠션 기하와 S/IK | `simulation/mujoco/runtime/standing_pose.py` |
| 시뮬레이터 제어·보호·프로필 | `simulation/mujoco/runtime/virtual_robot.py` |
| 물리/서보/접촉 모델 | `simulation/mujoco/runtime/cad_physics.py` |
| STM32 native 상태/수식 | `firmware/stm32-learning/Inc/s_native.h`, `s_native_impl.h` |
| 실기 틱 변환 | `firmware/stm32-learning/Inc/s_native_servo.h` |
| V6.1 보존 표 / V6.2.1 압축 표 | `firmware/stm32-learning/Inc/s_native_data.h`, `s_native_v621_data.h` |
| C 제어 통합·진단 | `firmware/stm32-learning/Src/robot.c`, `app_console.c` |
| 프로필 정의/생성 | `config/locomotion_profiles.json`, `tools/generate_locomotion_profiles.py` |
| Windows 모델·입력·화면 | `apps/windows/spot_controller/protocol.py`, `ui.py`, `connection.py` |
| iOS 모델/선택 | `apps/ios/SpotOMGController/SpotOMGController/Models/ConnectionState.swift`, `BLE/RobotBluetoothManager.swift` |
| C/Python 검증·물리 재생 | `simulation/mujoco/scripts/validation/validate_s_native_firmware.py` |
| V6.2.1 C/물리 회귀 | `simulation/mujoco/tests/test_s_native_v621_firmware.py` |
| 보행 형상/접지 검증 | `simulation/mujoco/scripts/validation/validate_s_native_v62.py` (이름은 V62지만 V621도 지정 가능) |
| 4방향 영상 | `artifacts/s-native-v6-2-1/video/four-views.mp4` |
| 물리/형상 원시 기록 | `artifacts/s-native-v6-2-1/validation/`, `artifacts/s-native-v6-2-1/video/trajectory.json` |
| 접촉/교차 비교 | `artifacts/s-native-v6-2-1/recovery-comparison.json` |
| 호스트 검증 | `artifacts/s-native-v6-2-1/firmware-validation/`, `artifacts/s-native-v6-2-1/firmware-host-units/` |
| 기존 모델 보존 확인 | `artifacts/s-native-v6-2-1/preservation.json`, `artifacts/s-native-v6-2-1/firmware-validation/preservation.json` |

### 오프라인 검증 재실행

아래 명령은 실기에 연결하지 않는다. 필요한 변경 후 해당 검사를 실행하고, 기존 결과를 덮어쓰지 않는다.

```powershell
python -X utf8 -m pytest simulation/mujoco/tests/test_s_native_v621_firmware.py simulation/mujoco/tests/test_s_native_firmware.py simulation/mujoco/tests/test_shared_locomotion.py apps/windows/tests/test_protocol.py -q
python -X utf8 simulation/mujoco/scripts/validation/check_firmware_windows.py --output artifacts/s-native-v6-2-1/home-host-check
python -X utf8 simulation/mujoco/scripts/validation/validate_s_native_v62.py --profile s_native_v6_2_1 --output artifacts/s-native-v6-2-1/home-sim-baseline
python tools/generate_locomotion_profiles.py --check
python tools/generate_gait_speed_labels.py --check
```

기존 영상은 **2초 정지＋8초 보행＋4초 Stop/유지**이며, 앞·뒤·위·옆 동시 화면에 질량·발 하중·높이·위상·기울기 등을 포함한다. 재생성하려면 `ffmpeg`를 준비한다.

```powershell
python -X utf8 simulation/mujoco/scripts/analysis/capture_s_native_balance.py --profile s_native_v6_2_1 --command 1000 --walk-seconds 8 --settle-seconds 4 --fourth-view side --output artifacts/s-native-v6-2-1/home-video
```

## 다음 작업에 그대로 전달할 요청

> 이 인계 문서와 AGENTS.md를 읽고 이어서 진행해 줘. 실기에는 V6.2.1(v63)이 설치됐지만 발을 끌다가 넘어졌다. 넘어짐 로그 분석과 수정은 아직 안 했다. 기존 모델·관절 부호·안전 장치를 보존하고, 실제 추종/접지 문제를 재현해 이동 중 무릎 회수를 개선해 줘. 목표 궤적과 C/Python 일치, 시뮬레이터 성공, 실기 성공을 구분해서 검증해 줘.
