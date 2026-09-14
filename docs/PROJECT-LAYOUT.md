# 폴더 구조와 실행 경로

2026-09-15: 실행 코드·테스트·실험 도구·설정·결과물을 분리했다. 파일 이동 목록은 `file-moves-2026-09-15.json`에 기록했다. 과거 JSON/CSV 결과의 해시와 당시 경로 표기는 원본 증거로 보존했다.

| 위치 | 저장할 내용 |
|---|---|
| `apps/` | 제어 앱 소스와 앱별 실행 도구 |
| `firmware/` | 임베디드 소스, 보드별 빌드 설정 |
| `hardware/` | 모델·회로·배선 자료 |
| `config/` | 공용 정책·개발 환경 설정 |
| `docs/`, `docs/references/` | 문서와 외부 참고 PDF |
| `simulation/mujoco/runtime/` | 실행 중 사용하는 Python/C 모듈 |
| `simulation/mujoco/tests/` | 자동 검사 |
| `simulation/mujoco/scripts/{analysis,tuning,validation,visualization}/` | 목적별 분석·실험·검증·시각화 도구 |
| `simulation/mujoco/config/` | 쿠션·실험 프로필·선택 의존성 |
| `simulation/mujoco/models/`, `cad_300mm/` | 실행 모델·CAD 자산 |
| `artifacts/simulation/mujoco/` | 이전에 시뮬레이터 소스 폴더에 섞여 있던 결과와 진단 기록 |
| `.cache/` | 저장소 수준 시험 캐시 |

프로젝트 루트의 `platformio.ini`는 PlatformIO의 프로젝트 검색과 기존 빌드 명령을 위해 유지한다. `pytest.ini`는 공용 모듈 검색 경로와 시험 캐시를 정의한다. 자동 생성된 `__pycache__`와 `.pio`는 Git에서 제외한다.

## 실행

아래 명령은 저장소 루트에서 실행한다.

```bash
conda env create -f config/environment.yml
conda activate spot_omg
mjpython simulation/mujoco/virtual_robot.py --viewer
python simulation/mujoco/scripts/validation/validate_s_native.py --command 600
python -m pytest simulation/mujoco/tests -q
```

`virtual_robot.py`와 `walk.py`의 기존 경로는 얇은 실행 진입점으로 유지한다. 실제 코드는 `runtime/`에 있다. Windows 앱이 가상 로봇을 찾고 시작하는 기존 진입점도 유지한다. 새 모듈 import는 `from simulation.mujoco.runtime.virtual_robot import RobotController`와 같은 패키지 경로를 쓴다.

Conda는 환경 파일이 있는 폴더를 기준으로 editable package를 설치하므로 `config/environment.yml`의 설치 경로는 `../tools/servo_tool`이다.

선택적인 상세 삼각형 충돌 검사에는 `simulation/mujoco/config/requirements-stow.txt`가 필요하다. 라이브 가상 로봇 실행에는 필수가 아니다.

## 이동 후 확인

- 시뮬레이터 자동 검사: 이동 전·후 모두 568개 통과, 기존 최적화 보행 검사 1개 실패. 해당 실패의 yaw 값도 전·후 동일한 12.4856586635°로, 요구치 10°를 넘는다. 상세 삼각형 충돌 검사는 선택 의존성 python-fcl이 없어 이 비교에서 제외했다.
- 파일 경로 관련 추가 검사: 18개 및 하위 검사 3개 통과.
- Python 구문 검사와 앱 속도 표시 생성 검사 통과.
- 이동한 Conda 환경 파일 위치에서 editable 설치 dry-run이 원래 tools/servo_tool을 찾는 것을 확인했다.
- 실제 macOS MuJoCo 창 재실행, LG 모니터 위치, 맥 앱의 TCP 연결과 상태 갱신, 영상 응답을 확인했다.

폴더 정리는 보행 품질 개선과 별개다. 현재 S V1의 잔여 대기와 동기 문제를 해결했다고 보지 않는다.
