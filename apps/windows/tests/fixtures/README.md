# V80 Stop 응답 재현

`v80-stop-diagnostics.txt`는 사용자가 2026-09-18 첨부한 실제 Windows 콘솔의
첫 `$SPOTDRIVE stopped reason=ok`부터 앱의 `> ^C` 직전까지를 발췌했다.
ID12의 `peak_ctx` 중간에서 끊긴 상태도 그대로 보존했다. 전체 원문은
`artifacts/attitudepd-v4/windows-stop-disconnect/user-console.txt`에 있다.

원문에는 수신 타임스탬프가 없다. 테스트의 패킷 크기/간격은 지연 조건을
재현하기 위한 설정이며, 실제 BLE 처리량 측정값이 아니다. 테스트에서 붙이는
줄 끝과 `# `는 진단 출력이 끝나는 경우를 검증하는 합성 입력이다.
# 추가 기록: v80-stop-tail-missing.txt

2026-09-18 사용자 `spot-omg-console.txt`에서 정상 Stop 응답부터 앱 `> ^C`
직전까지 추출했다. RR J2 peak_ctx 중간에서 끝나며, 누락된 출력·프롬프트를
보충하지 않았다. 재현 시험은 이 누락을 유지한 채 새 syncstate 조회에 대한
가짜 장치 응답으로 복구를 검증한다. 원본에는 수신 시각이 없으므로 120바이트
분할과 재생 간격은 실제 당시 BLE 타이밍의 복원이 아니다.
