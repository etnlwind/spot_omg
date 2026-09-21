"""3S LiPo warning policy; no radio, GUI or motor side effects."""
import re

CHARGE_MV = 11_000
CRITICAL_MV = 10_500
RECOVERED_MV = 11_400


def reading(line):
    if line.startswith('ID 1 '):
        pattern, historical = r'\bvoltage=(\d+)mV\b', False
    elif line.startswith('$BATTERY '):
        pattern, historical = r'\bmv=(\d+)(?=\s|$)', False
    elif line.startswith('Gait diagnostics:'):
        pattern, historical = r'\bmin_voltage=(\d+)mV\b', True
    else:
        return None
    match = re.search(pattern, line)
    if match and 0 < int(match[1]) <= 60000:
        return int(match[1]), historical
    return None


class BatteryWarning:
    def __init__(self):
        self.level = 0
        self.detected_mv = None
        self.recovery_started = self.recovery_last = None
        self.recovery_count = 0

    def observe(self, mv, now, historical=False):
        if historical or not 0 < mv <= 60000:
            return
        incoming = 2 if mv <= CRITICAL_MV else 1 if mv <= CHARGE_MV else 0
        if incoming:
            if incoming >= self.level:
                self.detected_mv = min(self.detected_mv or mv, mv)
            self.level = max(self.level, incoming)
        if not self.level or mv < RECOVERED_MV:
            self.recovery_started = self.recovery_last = None
            self.recovery_count = 0
            return
        if self.recovery_last is not None and now - self.recovery_last < 1:
            return
        if self.recovery_last is not None and now - self.recovery_last > 15:
            self.recovery_started = None
            self.recovery_count = 0
        if self.recovery_started is None:
            self.recovery_started = now
        self.recovery_last = now
        self.recovery_count += 1
        if self.recovery_count >= 3 and now - self.recovery_started >= 5:
            self.__init__()

    def snapshot(self):
        return dict(level=self.level, detected_mv=self.detected_mv,
                    title='즉시 사용 중단 · 배터리 충전' if self.level == 2 else '배터리 부족 · 지금 충전하세요',
                    message=(f'정지 후 안정 전압 {self.detected_mv / 1000:.1f}V. ' if self.detected_mv else '') +
                    '보행을 멈추고 몸체를 지지한 뒤 전원 스위치를 끄고 충전하세요.')
