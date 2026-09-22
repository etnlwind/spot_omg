# Spot OMG! · Windows V92-R2

앱 버전/빌드는 Apple과 동일한 **92.2.0 / 66**이며, 화면은 `V92-R2 (66) - Release`
(소스 실행은 Debug)로 표시합니다. 수정 번호가 생기면 Apple처럼 `V92-R1`로
표시합니다. Windows 빌드 시 Apple 프로젝트와 버전/빌드 일치를 검사합니다.
기본 연결 대상은 실제 로봇 BLE이며 연결은 사용자가 시작합니다.

Mac/iPhone과 같은 로봇 펌웨어 기준 버전 **92.2.0**입니다. Python/PySide6/Bleak 기반 Windows x64 앱이며, 실행 파일 이름은 `SpotOMGController.exe`, 표시 이름은 **Spot OMG!**입니다.

앞다리 간격 부호 수정은 [V92-R2 원인·검증](../../docs/FOOT-WIDTH-FRONT-SIGN-V92-R2.md)을 참고하세요.

## 현재 기능

- 기본 다크 화면, 읽기 쉬운 체크박스·보조 글자, Mac과 동일한 주황색 아이콘.
- **모델 보행 / 직접 설정 보행** 탭. 다른 모드의 설정은 숨깁니다. 모델 목록은 탭 안에 한글 이름으로 표시하며 지원 연결에서 `IMU 자세 안정화 V6`를 맨 위·기본값으로 선택합니다. 이전 모델과 구 펌웨어 지원도 유지합니다.
- **발 위치 보정** 팝업: 실제 MuJoCo CAD의 주황색 윗모습, FL/FR/RL/RR 추가 들림과 좌우 간격(mm). 간격은 0 기본, 음수 안쪽, 양수 바깥쪽이며 V92-R1 펌웨어의 `footwidth` 지원이 필요합니다. `footlift save` 및 새 readback으로 STM32 영구 저장을 확인합니다. Windows·Mac·iPhone이 같은 로봇의 값을 공유합니다. 팝업을 여는 것만으로 로봇 값을 덮어쓰지 않습니다.
- 전/후/좌/우 각 축의 ±20° 입력을 직진·후진·제자리 회전으로 맞춥니다. 나머지는 이동과 회전의 혼합이며 자동 복귀/입력 유지, WASD/방향키 조종을 제공합니다.
- Landing, Stow, Stand, Stand11, Recover, Relax, 수평 보정, 직진 유지, **IMU 복구**. 수동 IMU 복구는 완료 응답과 새 상태를 확인한 뒤 스틱을 놓고 다시 조작해야 합니다. 보호를 해제하거나 이전 보행 입력을 재생하지 않습니다.
- BLE 제어 응답과 상세 진단 로그를 분리합니다. `@D`·`@S` 등 조종 명령이 로그 표시·파일 저장을 기다리지 않습니다. 구 브리지와 시뮬레이터의 기존 통신도 지원합니다.
- 상세 진단은 백그라운드에서 `%LOCALAPPDATA%/SpotOMG/RobotConnection/`의 `current.jsonl`, `previous.jsonl` 두 파일로 순환 기록합니다(각 512KiB). **진단 로그 저장**으로 별도 JSONL 파일을 보존합니다. 기록 큐가 가득 차면 로그 유실을 기록하며 조종을 지연시키지 않습니다. 화면에는 명령 결과·오류·상태와 복구 알림을 표시합니다. **화면 로그 저장**은 보이는 텍스트만 저장합니다.
- 전압 표시·충전 경고는 정지 후 새 `read 1` 응답으로 측정합니다. 정지 후 3초 안정화와 1초 간격 3개 표본을 사용하며, 과거 보행 로그의 최저 전압이나 `$BATTERY` 진단 출력으로 경고를 만들지 않습니다.

V6는 실험 보행입니다. [구현과 검증 한계](../../docs/ATTITUDE-V6-RECOVERY.md)를 참고하세요. 앱 업데이트와 실제 보행 검증은 별개입니다.

## 실행 및 빌드

Windows x64, Python 3.10 이상에서 실행합니다. 기존 `spot_omg` Conda 환경을 사용합니다.

```powershell
conda activate spot_omg
python -m pip install -r ./apps/windows/requirements.txt 'pyinstaller>=6,<7'
./apps/windows/run.ps1
./apps/windows/build.ps1
```

Python을 직접 지정할 수도 있습니다.

```powershell
./apps/windows/build.ps1 -Python C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe
```

빌드 결과는 `apps/windows/dist/SpotOMGController/SpotOMGController.exe`입니다. **같은 폴더의 `_internal`을 포함한 폴더 전체**가 필요합니다. 빌드에는 아이콘·로봇 이미지·버전 정보가 포함됩니다. 구 앱을 종료하고 기존 설치 폴더를 백업한 뒤 새 폴더로 교체하세요. 사용자 설정과 진단 기록은 실행 파일 폴더 밖에 보관됩니다.

Windows V92-R2 (65) 실행 파일을 빌드했습니다. Apple 앱의 빌드 및 기기 설치는 별도 Mac 환경에서 필요합니다.

## 로봇 연결과 조작

**실제 로봇 · BLE**를 선택하고 연결합니다. 다른 Mac·iPhone·Windows 앱이 로봇에 연결되어 있다면 먼저 해제하세요. `SpotOMG-Bridge`의 BLE 서비스를 사용하며 Bluetooth SPP COM 포트를 사용하지 않습니다. 연결 후 새 상태를 확인하고 조작을 활성화합니다. 앱 실행만으로 연결하거나 움직이지 않습니다.

- 스틱 또는 키를 놓으면 정상 감속 정지를 요청합니다. 자동 복귀를 끄면 스틱 입력을 유지합니다.
- Stop/Space는 보행 정지와 대기 자세 복귀, Esc는 긴급 중단입니다. 입력창에서는 Space와 조종 키를 텍스트로 처리합니다.
- 창 비활성화·GUI heartbeat 유실·연결 오류 시 이전 입력을 계속 보내지 않습니다. 오류 후 수동으로 다시 연결합니다.
- Relax는 기존 절차대로 Landing 완료 및 새 자세 readback을 확인한 뒤 토크를 해제합니다. 실패한 Landing을 생략하지 않습니다. Stow에서는 Landing으로 먼저 펼칩니다.
- 직접 설정 보행은 V6.2.5를 사용하는 별도 파라미터 시험입니다. 설정 적용·조회와 Stand 확인 후 조이스틱 전진 또는 시험 시작으로 실행합니다.

## 로컬 MuJoCo

로컬 시뮬레이터에는 저장소 전체와 별도 Python/MuJoCo 환경이 필요합니다. 독립 앱 폴더에는 CAD 모델이나 MuJoCo 환경을 포함하지 않습니다.

```powershell
conda activate spot_omg
./apps/windows/setup.ps1
```

`setup.ps1`은 공용 servo 도구·MuJoCo와 SHA-256을 확인한 Zig 도구를 설치합니다. 앱에서 프로젝트/Python 경로를 지정하고 **MuJoCo 시작 + 연결**을 선택하세요. TCP 제어 8765, 영상 8766이 기본입니다. **별도 3D 뷰어도 열기**로 카메라를 조작할 수 있습니다. 원격 시뮬레이터는 주소와 영상 포트를 지정합니다.

영상은 실제 로봇 카메라가 아닙니다. 기존 2.754kg 모델의 질량 분포·접촉 물성은 추정값입니다. 앱이 시작한 시뮬레이터만 종료하며, 앱 heartbeat가 끊기면 해당 프로세스도 종료합니다.

## 검증

```powershell
python -X utf8 -m pytest apps/windows/tests -q -p no:cacheprovider
python -X utf8 apps/windows/tests/qa_desktop.py
./apps/windows/dist/SpotOMGController/SpotOMGController.exe --smoke-test ./artifacts/windows-v92/packaged.png
```

단위·UI 시험은 실제 BLE 장치를 검색하거나 움직이지 않습니다. `qa_desktop.py`는 별도의 MuJoCo 제어/영상 포트에서 조종·정지·프로세스 종료를 검사합니다. 소프트웨어 시험, 실기 설정 readback, 실제 위치 유지, 전체 보행 시험을 구분합니다.

시작 오류는 `%LOCALAPPDATA%/SpotOMGController/error.log`에 기록됩니다. 과거 Windows 검증은 [VALIDATION.md](VALIDATION.md), 이번 변경은 [V92 맥 기능 반영 기록](../../docs/WINDOWS-V92-PARITY.md)을 참고하세요.
