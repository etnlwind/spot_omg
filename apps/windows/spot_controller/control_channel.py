"""Framed, correlated replies on the dedicated BLE control characteristic.

Console notifications never enter this parser. A console prompt, diagnostic
line, or reply for a previous request cannot complete the active command.
"""
class ControlStream:
    def __init__(self):
        self.buffer = bytearray()
        self.active = False

    def feed(self, data):
        events = []
        for byte in data:
            if byte == 0x1e:
                self.buffer.clear()
                self.active = True
            elif byte == 0x1f and self.active:
                self.active = False
                try:
                    sequence, kind, payload = bytes(self.buffer).split(b' ', 2)
                    sequence = int(sequence)
                    if not 0 < sequence <= 0xffffffff or kind not in (b'DATA', b'DONE'):
                        raise ValueError('invalid control envelope')
                    if kind == b'DONE' and payload:
                        raise ValueError('invalid completion payload')
                    events.append((sequence, kind.decode(), payload))
                except (ValueError, UnicodeError) as exc:
                    raise ConnectionError('Invalid control reply') from exc
            elif self.active:
                self.buffer.append(byte)
                if len(self.buffer) > 766:
                    raise ConnectionError('Oversized control reply')
        return events


def request(sequence, data):
    if not 0 < sequence <= 0xffffffff:
        raise ValueError('Invalid control sequence')
    command = data.rstrip(b'\r\n')
    limit = 128 if command.startswith(b'footlift save ') else 96
    if not command or len(command) >= limit or any(b < 32 or b >= 127 for b in command):
        raise ValueError('제어 명령은 ASCII 한 줄, 95바이트 이내여야 합니다.')
    return b'@C ' + str(sequence).encode() + b' ' + command + b'\n'
