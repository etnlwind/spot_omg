"""User-visible pause reasons, independent of console visibility."""
def motion_warning(reason):
    if not reason:return ('', '')
    text=reason.lower()
    if 'tilt' in text or 'unstable' in text:
        cause='기울기 보호로 일시정지'
    elif 'imu' in text or 'attitude' in text:
        cause='IMU 상태 이상으로 일시정지'
    elif 'watchdog' in text:
        cause='제어 명령 수신 지연으로 일시정지'
    elif 'voltage' in text:
        cause='전압 이상으로 일시정지'
    else:
        cause='동작이 중단되었습니다'
    return cause, ('주변과 로봇 상태를 확인하세요. 계속할지는 사용자가 결정합니다. '
                   '스틱을 놓고 다시 조작하거나 자세 명령을 선택하세요. 자동 재개하지 않습니다.\n'
                   '원인: '+reason)
