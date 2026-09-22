# 발 위치 보정 · V92-R1 (65)

## 후속 실기·Windows 설치 완료

2026-09-22 사용자 설치 요청으로 실기 OTA 및 Windows 설치를 완료했다.
설치 전 Landing 완료(오차 24틱) → 12축 토크 OFF → 이미지 검증·OTA·재부팅 →
새 펌웨어 Landing 완료(오차 24틱)를 확인했다. 종료 상태는 Landing/토크 ON/safety OK다.
서보 12개 영구 레지스터 0..39가 설치 전과 동일하다. 저장된 추가 들림은 네 발 모두
10mm로 보존됐고 새 좌우 간격은 모두 0mm다. 실제 간격 변경 보행/위치 유지 시험은 아직 하지 않았다.

설치 증거: `artifacts/foot-position/install-01/prepare.json`, `ota.json`, `post-verify.json`.
Windows 설치 경로: `%LOCALAPPDATA%/Programs/SpotOMG/V92-R1-65/SpotOMGController.exe`.
바탕화면 `Spot OMG.lnk`를 새 버전으로 변경하고 Release 실행·메뉴 표시를 확인했다.
아래 미설치 문구는 구현 직후의 검증 단계 기록이다.

Windows의 기존 대시보드 배치를 유지하면서 Windows·Mac·iPhone의 `발 높이 설정`을 `발 위치 보정`으로 변경했다. 각 발 FL/FR/RL/RR에 추가 들림과 좌우 간격(mm)을 입력한다.

- 간격 0은 해당 보행의 기본 궤적 그대로, 음수는 몸체 안쪽, 양수는 바깥쪽이다. 양쪽 다리에서 같은 의미다.
- 간격은 J1 각도 자체가 아닌 발끝 좌우 위치의 보정량이다. 기본 발끝 X/Z를 유지하도록 J1/J2/J3를 함께 계산하며 기존 서보 부호·원점 변환을 유지한다.
- 보행의 지지·스윙 구간에 적용하고 기존 시작 활동량에 따라 반영한다. 정적 Landing/Stow/Stand는 바꾸지 않는다. Native 정지 시 기존 발 옮김으로 S에 복귀한다.
- 저장하지 않고 닫았다가 열면 로봇에서 확인한 적용값을 다시 표시한다. 저장은 8개 값을 한 번에 전송하고 새로운 일치 readback까지 확인한다.
- 들림은 0..2147483647, 간격은 -2147483647..2147483647 정수 입력을 허용한다. 입력 가능 범위가 기구적 도달 가능 범위는 아니다. 도달 불가 IK는 실패로 반환하며 계산 중 일부 다리만 반영하지 않는다. 실제 안정성은 별도 실기 검증 대상이다.

## 프로토콜 및 저장

펌웨어 `attitudepd-v6-v92-r1`의 `footwidth` capability로 지원 여부를 구분한다. 구 펌웨어에서는 간격 입력을 비활성화한다.

```
footlift save LFL LFR LRL LRR WFL WFR WRL WRR
footlift show
```

기존 4개 들림만 보내는 명령은 저장된 간격을 유지한다. `$SPOTSTATE`에 `width_fl`, `width_fr`, `width_rl`, `width_rr`를 추가했다. 기존 Flash 설정 레코드의 payload를 16→32바이트로 확장했고 구 16바이트 기록의 간격은 0으로 읽는다. 쓰기 중단 시 이전 유효 기록을 유지한다. 시뮬레이터 JSON도 두 설정을 함께 원자적으로 저장한다.

8개 최대 길이 정수를 수용하도록 콘솔 명령 버퍼를 128바이트로 늘렸다. 앱의 확장 길이 허용은 `footlift save`에 한정된다. STM32 realtime 프레임 144바이트 이내다.

## 검증과 배포 상태

- Windows 및 관련 firmware/simulator 호스트 시험: 285개 통과, 기존 제어 로그 시험 1개 실패. 실패는 `test_stop_ready_without_waiting_for_logs_and_old_reply_cannot_complete_new_command`의 `control-tx @D` 로그 기대이며 변경 전 HEAD에서도 재현했다.
- 추가 명령 길이 변경 후 관련 시험 19개 통과, 같은 기존 로그 시험 1개 실패.
- Flash C 호스트 시험: 재시작, 구 명령의 간격 보존, 쓰기 중단, 레코드 순환·초기화·교정 저장 보존 확인.
- 배포 보행의 ±간격 방향, X/Z 보존, 0 무변경, 도달 불가 시 원본 보존 확인. Native 7종은 펌웨어/Python 궤적 일치와 정지 후 S 복귀 확인.
- Windows 실행 파일 빌드 및 offscreen 시작 시험 성공. 편집 화면은 모의 readback으로 렌더링 확인.
- 최종 펌웨어 326220바이트, SHA256 `29ee86040e2fad63d3afc240c5f99c9340ac226ea0a7c528acd6980017b208a0`.
- Apple 소스·프로젝트 버전은 변경했으나 이 Windows에서 Xcode 빌드/기기 설치는 하지 않았다.
- **실기 펌웨어 설치, 실기 설정 readback, 실제 위치 유지, 전체 보행 시험은 아직 수행하지 않았다.** 실기 설치 시 Landing 도착 확인 → 토크 OFF → 설치 → Landing 확인 절차를 따른다.

Windows 결과: `apps/windows/dist/v92-r1-build65/SpotOMGController/SpotOMGController.exe` (동일 폴더의 `_internal` 필요).
펌웨어 결과: `artifacts/foot-position/v92-r1-final/attitudepd-v6-v92-r1.bin`.
