# V77-T1 runtime experiment parameters

Firmware: `s-native-v6-2-7-v77-t1-param`. One initial installation is required; subsequent changes below use console commands without rebuilding or flashing. This is a bounded diagnostic based on V625, not a replacement registered gait model.

## Console protocol

```
probeconfig show
probeconfig set 20 344 4000 all
probeconfig show
walkprobe
```

`set` order: total peak lift in mm, forward drive input in per mille, drive duration in milliseconds, legs (`all`, `rl`, `rr`).

| Parameter | Range | Default |
| --- | --- | --- |
| lift_mm | 12–40 | 20 |
| linear | 1–1000 (input, not measured speed) | 344 |
| duration_ms | 500–30000 | 4000 |
| legs | all / rl / rr | all |

Response example:

```
$PROBECONFIG lift_mm=20 linear=344 duration_ms=4000 legs=all storage=ram
```

`probeconfig reset` restores defaults. Settings are RAM only and reset on firmware reboot. Set/show/reset do not command motion. Invalid or incomplete input preserves the previous configuration. Changes are rejected while driving, stowing or gait diagnostics are active.

`walkprobe` requires idle selected V625 and executes the configured probe. Normal `drive` retains the selected original model. With rl/rr only that rear leg follows the gait; other three final targets are S. With all, normal four-leg coordination runs. Lift modifies swing Z after baseline J1 computation; horizontal path and phase shape stay V625. This interface does not adjust body lean, J1 adduction, phase offset, servo limits or safety thresholds.

Duration begins inside the drive loop, after preparation. Stop is requested at duration; return to S takes additional time. Abort bound is duration + 4 seconds. Watchdog and IMU/tracking protection remain active. The accepted range is not a claim that every combination is physically validated.

## Host execution with readback and trace capture

From repository root, using the project's Python environment:

```
python -m scripts.hardware.capture_v77_t1_params --execute-authorized --lift-mm 20 --linear 344 --duration-ms 4000 --legs all --output artifacts/v77-t1/param-walk20-01
```

For a single RL trial use `--legs rl`. Use a new output directory for every trial. This command actually moves the robot: it checks state/voltage, preserves old traces, sends settings, checks returned settings, reaches S, arms traces, executes and sends matching drive heartbeats until stopping. Configuration, timestamps, joint and IMU traces are saved in the output directory. Do not manually issue walkprobe without the normal drive heartbeat/stop protocol: the existing 800 ms watchdog will stop it.

## Validation and deployment status

- C configuration tests: defaults, bounds, malformed/missing/huge input, atomic rejection.
- C gait tests: RL/RR/all lift geometry, unchanged baseline J1/X, stop completion.
- Firmware build: 319492 bytes, SHA256 b00caa6223a5eefb19f7ba4d05da90b7d4fe411810f3998d8d0736b4ed15a3c6.
- Installed and read back `s-native-v6-2-7-v77-t1-param`. User confirmed level body support; recover held current joint positions, supervised Landing completed, all 12 torque-off readbacks passed, OTA verified/rebooted, post-install Landing and persistent servo settings verification passed. Runtime setting changed to 28/250/2000/rl and read back, then restored to 20/344/4000/all and read back without motion. Immediately after installation: Landing, torque OFF. Later configuration-only readback: custom pose, error 361 ticks, torque OFF, safety OK, fault 0, selected V625; the posture changed while torque was off. No corrective motion was issued. No parameterized gait motion was executed during installation. Evidence: `artifacts/v77-t1/param-install/`.

## First parameterized full-walk trial

User authorized Stand and test. Initial Stand was rejected from Stow; trace pre-download then refused while Stow was paused. A separate supervised Landing completed (error26 ticks), then `param-walk20-03` reached Stand and executed 20/344/4000/all. Tilt protection stopped motion at2857ms. IMU roll -3.8..+12.9 degrees, pitch -1.4..+3.2. J3 peak sampled errors FL/FR/RL/RR 8.88/9.58/8.61/8.17 degrees. User observed feet did NOT lift sufficiently. This differs from the earlier supported RL clearance observation; nominal20mm is not guaranteed floor clearance. Final state custom, torqueON, tilt fault14; no automatic recovery or retry. Evidence: `artifacts/v77-t1/param-walk20-03`.

## 28 mm comparison

User requested Stand and next test. Recover held current positions; supervised Stand completed. Config readback 28/344/4000/all; no firmware update. Trial `artifacts/v77-t1/param-walk28-01` stopped on tilt at1097ms (20mm comparison2857ms). User observed a fall to the right; foot clearance was not confirmed. IMU roll -6.1..+16.0 degrees, pitch -1.8..+1.1; initial roll -2.7 degrees. Joint and IMU trace downloads validated. Final recorded state custom, torque ON, tilt fault14. No further motion. Increasing lift alone did not yield stable walking; different initial/support conditions prevent attributing the shorter time solely to lift height. Review support transfer and roll onset before further height increases.

## FR J1 calibration revision

Installed `s-native-v6-2-7-v77-t1-param-j1`: FR J1 center2073 (was2089), user-selected manual zero. `targets` readback ID4=2073. Landing completed; torqueOFF. Persistent servo registers unchanged. Runtime capture default revision updated accordingly; use --revision s-native-v6-2-7-v77-t1-param only with the previous firmware. No gait after calibration.

## 수직 기준 간격과 FR 출발 옵션

`v77-t1-width` 시험 펌웨어의 `walkprobe` 전용 설정입니다. 일반 보행 모델은 변경하지 않습니다.

- 한쪽 발 기준 **0mm = 수직 기준**, 음수 = 안쪽, 양수 = 바깥쪽. 범위 −40~+20mm, 새 기본값 0mm.
- 기존 정상 오므림은 수직 기준 약 −37.8mm/발입니다. 기존 첫걸음 비대칭과 보정 때문에 순간값은 다릅니다.
- **출발 시 FR만 추가 오므림** 체크박스: 기본 해제. 해제하면 좌우 공통 간격을 적용합니다.
- 체크하면 음수 간격에서 첫걸음 FR만 기구 J1 변화량을 2배 적용하고 다른 다리는 S의 J1을 유지합니다. 다음 걸음에 공통 간격으로 전환합니다. 0mm/양수에서는 추가 오므림을 적용하지 않습니다.
- 설정은 RAM에 저장합니다. 단일 뒷다리 시험은 선택된 다리만 움직이며 나머지는 S를 유지합니다. 실제 위치와 접지는 별도로 확인해야 합니다.

```text
probeconfig set 28 344 4000 all -20 0
probeconfig show
```

마지막 값은 FR 옵션(0=해제, 1=체크)입니다. Mac/iPhone/Windows의 시험 패널에서 간격과 체크박스를 설정하고 **설정 적용 + 조회** 후 적용값을 확인합니다. 새 펌웨어가 설치돼야 옵션을 사용할 수 있습니다.

### 시험 시간 로그

호스트 시험 도구 `scripts/hardware/capture_v77_t1_params.py`는 `--width-mm -20 --fr-extra`로 같은 설정을 전달합니다(`--fr-extra` 생략 시 해제).

- `events.jsonl`: 한국시간 ISO 8601 `timestamp`(+09:00), 시험 도구 시작 후 `elapsed_ms`, 송수신 명령 및 응답.
- `summary.json`: 시작/종료 `started_at`/`ended_at`, 총 `elapsed_ms`, 적용 파라미터와 FR 옵션, 정지 결과 및 오류.
- 센서 자체 시간은 원본 trace에 보존합니다. 호스트 수신 시간은 측정 순간과 같지 않으므로 영상과 자동 동기화됐다고 간주하지 않습니다.
- 빌드·호스트 테스트 기록: `artifacts/v77-t1/width-validation/tests.jsonl`. 실기 동작과 구분해 기록합니다.

## 앱 저장과 보행 모드

- **기본 보행**: 초기 선택. 기존 조이스틱 보행을 사용하며 파라미터 메뉴는 비활성화됩니다.
- **파라미터 보행**: 저장/불러오기/로봇 적용 메뉴 활성화. V6.2.5·Stand·설정 적용 및 조회 완료 후 조이스틱을 앞으로 밀면 적용값으로 시작합니다. 전체 다리(all)를 선택해야 합니다.
- **파라미터 저장**은 해당 앱의 로컬 저장소에 저장합니다. 앱을 다시 열면 불러옵니다. 기기 간 자동 동기화나 로봇 영구 저장은 아닙니다.
- 편집/불러오기 후 **설정 적용 + 조회**를 눌러야 로봇에 반영됩니다. 적용값 표시는 로봇 readback입니다.
- 파라미터 보행은 전진 전용이며 전진 입력은 설정값을 사용합니다. 조이스틱 해제/중립 또는 설정 시간 도달 시 정지합니다. 자동 반복하지 않으며 다시 조작해야 시작합니다.
- 기본 모드로 전환하려면 먼저 정지합니다. 새 연결은 기본 모드로 시작합니다. 현재 `v77-t1-width` 펌웨어에서 사용할 수 있어 펌웨어 재설치는 필요 없습니다.

### 조이스틱 자동 복귀

조이스틱 왼쪽 상단의 **자동 복귀 ON/OFF** 버튼을 사용합니다. 기본 ON이며 OFF는 손을 떼어도 입력을 유지합니다. ON으로 전환하거나 Stop/중앙 입력으로 정지할 수 있습니다. 앱 비활성화·연결 해제 시 입력을 해제합니다. 파라미터 보행의 설정 시간 제한은 그대로 적용되며 자동 반복하지 않습니다.
