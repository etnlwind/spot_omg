# Mac 조종 중 Stand 전도 보고: 초기 진단

사용자 보고: Mac 앱에서 조종하면 Stand로 일어서다가 전도. Stand 단독인지 조이스틱 자동Stand/전환 구간인지 확인 요청 중.

Mac 실행 앱 V0.5.0 (43), 실제 로봇 V45/arcsupport 연결은 UI에서 확인. 마지막 수신 상태 Stand, error77, torque on, safety ok, ID1 voltage11100mV/hw0. 이후 사용자의 Landing 시작 출력이 `Starting slow synchronized `에서 끊김. 진단 중 Stop 요청했으나 응답이 없었고 앱은 BLE timeout으로 연결 끊김. 직접 BLE 로그 조회도 SpotOMG-Bridge not found로 실패했다. 새로운 자세/보행 명령은 실행하지 않았다.

이 기록은 전도 순간 IMU/서보 추종/제어시간 로그가 아니다. 전원·통신 단절이 전도 원인인지, 이후 전원 차단의 결과인지는 아직 알 수 없다. 84MHz 설정이 전도 원인이라고 단정하지 않는다. 로봇 재접속 후 저장 로그와 balance trace를 우선 확보해야 한다.
