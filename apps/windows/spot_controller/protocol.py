"""Deterministic controller. No GUI, radio, clock, or robot side effects."""
from collections import deque
import codecs
import math
import re
from .battery import BatteryWarning, reading as battery_reading

NATIVE_PROFILES = ("s_native_v6_2_7", "s_native_v6_2_6", "s_native_v6_2_5", "s_native_v6_2_4", "s_native_v6_2_3", "s_native_v6_2_2", "s_native_v6_2_1", "s_native_v6_2", "s_native_v6_1") + tuple(f"s_native_v{version}" for version in range(6, 0, -1))
SIMULATOR_NATIVE_PROFILES = frozenset(NATIVE_PROFILES) - {"s_native_v6_1", "s_native_v6_2_1", "s_native_v6_2_2", "s_native_v6_2_3", "s_native_v6_2_4", "s_native_v6_2_5", "s_native_v6_2_6", "s_native_v6_2_7"}
PROFILES = ("attitudepd_v2",) + NATIVE_PROFILES + (
    "attitudepd", "centerpivot", "arcsupport", "arcturn", "legacy", "crawl",
    "cruise", "trot", "highstep", "lift", "imu", "level", "level15", "joint",
    "jointfast", "jointsport", "cushion_reach", "cushion_j2lift", "cushion_wbc",
    "cushion_forward", "cushion_support_shift", "cushion_support_shift_v2",
    "cushion_v2_push", "cushion_diagonal_sync_wide80",
)
POSTURES = {"landing", "stow", "stand", "stand11"}
READ_ONLY = {"syncstate", "targets", "gaitdiag", "baldiag", "imudiag",
             "locomotiondiag", "read 1", "identity", "log show 64", "stabilize status"}


class ConsoleStream:
    def __init__(self):
        self.buffer = ""
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")

    def feed(self, data):
        self.buffer += self.decoder.decode(data)
        events = []
        while self.buffer:
            if self.buffer.startswith("# "):
                self.buffer = self.buffer[2:]
                events.append(("prompt", ""))
                continue
            match = re.search(r"[\r\n]", self.buffer)
            if not match:
                break
            line = self.buffer[:match.start()].strip()
            self.buffer = self.buffer[match.end():]
            if line:
                events.append(("line", line))
        if len(self.buffer) > 8192:
            raise ValueError("Console input exceeded 8192 bytes without a line ending")
        return events


def drive_vector(x, y):
    """Match iOS RobotDriveVector including axis corridors and Swift rounding."""
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    distance = math.hypot(x, y)
    if distance > 1:
        x, y = x / distance, y / distance
    magnitude = min(1., distance)
    if magnitude < .15:
        return (0, 0)
    scale = (.30 + .70 * (magnitude - .15) / .85) * 1000 / magnitude
    def axis(value, other):
        dead = .10 * abs(other)
        mapped = math.copysign(max(0., abs(value) - dead) / (1 - dead), value) * scale
        return int(math.copysign(math.floor(abs(mapped) + .5), mapped))
    return axis(y, x), axis(x, y)


class Controller:
    def __init__(self, simulator=False):
        self.simulator = simulator
        self.stream = ConsoleStream()
        self.connected = False
        self.synced = False
        self.state = {}
        self.caps = set()
        self.phase = "offline"
        self.error = ""
        self.outbox = deque()
        self.sequence = 0
        self.vector = None
        self.requires_release = True
        self.next_heartbeat = 0.
        self.deadline = 0.
        self.refresh_at = None
        self.pending = None
        self.command = None
        self.command_ok = False
        self.command_state_seen = False
        self.command_error = False
        self.confirm_pose = None
        self.relax_pending = False
        self.voltage = None
        self.voltage_at = None
        self.battery = BatteryWarning()
        self.next_voltage_poll = 5.
        self.parameter_walking = False
        self.probe_from_joystick = False
        self.state_at = None
        self.fatal = False
        self.disconnect_requested = False
        self.default_profile_pending = True
        self.probe_config = None
        self.probe_expected = None
        self.probe_running = False
        self.probe_stop_at = None

    @property
    def supports_probe(self):
        return not self.simulator and self.state.get('rev') in {
            's-native-v6-2-7-v77-t1-param', 's-native-v6-2-7-v77-t1-param-j1', 's-native-v6-2-7-v77-t1-width', 'attitudepd-v2-v78'}

    @staticmethod
    def parse_probe(values):
        if len(values) not in {4,5,6}:
            raise ValueError('들림, 전진 입력, 시간, 대상 다리가 필요합니다.')
        lift, linear, duration = map(int, values[:3])
        leg = values[3]
        if not (12 <= lift <= 40 and 1 <= linear <= 1000 and 500 <= duration <= 30000 and leg in {'all','rl','rr'}):
            raise ValueError('시험 파라미터 범위를 확인하십시오.')
        width = int(values[4]) if len(values)>=5 else None
        if width is not None and not -40 <= width <= 20:raise ValueError("간격은 -40..20mm입니다.")
        fr=int(values[5]) if len(values)==6 else 0
        if fr not in (0,1):raise ValueError("FR 옵션은 0 또는 1입니다.")
        return (lift, linear, duration, leg) if width is None else (lift,linear,duration,leg,width,fr)

    def start_probe(self, now):
        if not (self.supports_probe and self.can_drive and self.phase == 'idle'
                and self.state.get('pose') == 'stand' and self.state.get('profile') == 's_native_v6_2_5'
                and self.probe_config is not None and self.probe_expected is None):
            raise ValueError('V6.2.5 · Stand · 설정 조회 완료 후 시험을 시작하십시오.')
        self.probe_running = True
        self.probe_stop_at = now + self.probe_config[2] / 1000 + 3
        self.vector = (self.probe_config[1], 0)
        self.phase = 'drive'
        self.refresh_at = None
        self.next_heartbeat = now + .2
        self.packet('walkprobe\n')

    def packet(self, text, kind="command"):
        data = text.encode("utf-8") if isinstance(text, str) else text
        if kind in {"update", "stop", "interrupt"}:
            self.outbox = deque((k, d) for k, d in self.outbox if k != "update")
        if kind == "interrupt":
            self.outbox.clear()
        if len(self.outbox) >= 16:
            raise RuntimeError("Transmission queue overflow")
        self.outbox.append((kind, data))

    def next_sequence(self):
        self.sequence = (self.sequence + 1) & 0xffffffff
        return self.sequence

    def opened(self, now):
        self.connected = True
        self.phase = "idle"
        self._console("syncstate", now)

    @property
    def stowed(self):
        return self.state.get("pose") in {"stow", "stow-paused"}

    @property
    def can_drive(self):
        return (self.connected and self.synced and not self.fatal and not self.disconnect_requested
                and self.phase in {"idle", "drive"} and not self.stowed
                and self.state.get("safety") == "ok")

    @property
    def motion_active(self):
        return self.phase in {"drive", "stopping", "draining"} or (
            self.phase == "busy" and self.command not in READ_ONLY and not (self.command or '').startswith('log time '))

    def validate(self, line):
        if not line or len(line.encode("utf-8")) > 240 or any(ord(c) < 32 for c in line):
            raise ValueError("명령은 240바이트 이내의 한 줄이어야 합니다.")
        words = line.split()
        if words[0] in {'walkprobe','rearprobe'}:
            raise ValueError('시험 시작 버튼을 사용하십시오.')
        if words[0] == 'probeconfig':
            if not self.supports_probe:raise ValueError('파라미터 펌웨어가 필요합니다.')
            if words[1:] not in [['show'], ['reset']]:
                if len(words) not in {6,7,8} or words[1] != 'set':raise ValueError('잘못된 파라미터 명령')
                self.parse_probe(words[2:])
                if len(words)>=7 and self.state.get("rev") not in ("s-native-v6-2-7-v77-t1-width", "attitudepd-v2-v78"):raise ValueError("간격 지원 펌웨어가 필요합니다.")
        first = words[0]
        if first in POSTURES | {'relax', 'recover', 'hold'} and len(words) != 1:
            raise ValueError('자세 명령에는 추가 인자를 사용할 수 없습니다.')
        if first in {"drive", "@D", "@S", "simwalk"} or first.startswith("@"):
            raise ValueError("연속 보행은 조이스틱으로 조작하십시오.")
        if self.stowed and line not in READ_ONLY and line != "landing" and not (
                self.state.get("pose") == "stow-paused" and line == "stow"):
            raise ValueError("Stow 상태: Landing으로 먼저 펼쳐 주십시오.")
        required = {"stow": {"stow", "simstow"} if self.simulator else {"stow"},
                    "trot5": {"trot5"}, "heading": {"headinghold"},
                    "balance": {"balancecontrol"}, "simbalance": {"simbalance"},
                    "gaitprofile": {"gaitprofiles"}, "simprofile": {"simprofiles"}}
        if first in required and not (self.caps & required[first]):
            raise ValueError("연결된 제어기가 지원하지 않는 기능입니다: " + first)
        if first in {"gaitprofile", "simprofile"}:
            if len(words) != 2 or words[1] not in PROFILES:
                raise ValueError("지원 목록의 보행 정책을 선택하십시오.")
            profile = words[1]
            if (profile.startswith("cushion_") or profile in SIMULATOR_NATIVE_PROFILES) and not self.simulator:
                raise ValueError("이 보행 정책은 시뮬레이터 전용입니다.")
            if profile in {*NATIVE_PROFILES, "attitudepd_v2", "attitudepd", "centerpivot", "arcsupport"} and profile not in self.caps:
                raise ValueError("제어기가 선택한 실험 정책을 지원하지 않습니다.")

    def request(self, line, now):
        line = line.strip()
        if line in {'app_parameter_mode 0','app_parameter_mode 1'}:
            if self.phase != 'idle':raise ValueError('정지 후 모드를 변경하십시오.')
            self.parameter_walking=line.endswith('1');return
        if line == 'app_probe_start':
            self.start_probe(now);return
        if line == "hold" or line == "\x03":
            self.interrupt(now)
            return
        if not self.synced or self.fatal or self.disconnect_requested:
            raise ValueError("연결 및 상태 동기화를 먼저 완료하십시오.")
        self.validate(line)
        if self.phase == "drive":
            if line in READ_ONLY:
                return  # State reads must not terminate or congest a held drive.
            self.pending = line
            self.release(now)
            return
        if self.phase != "idle":
            raise ValueError("이전 명령의 완료 응답을 기다리는 중입니다.")
        if line == "relax":
            self.relax_pending = True
            self._console("landing", now)
        else:
            self._console(line, now)

    def _console(self, line, now):
        if line.startswith('probeconfig set '):
            self.probe_config = None
            self.probe_expected = self.parse_probe(line.split()[2:])
            if len(self.probe_expected)==4 and self.state.get('rev') in ("s-native-v6-2-7-v77-t1-width", "attitudepd-v2-v78"):self.probe_expected += (0,0)
        elif line == 'probeconfig reset':
            self.probe_config = None
            self.probe_expected = (20,344,4000,'all') + ((0,0) if self.state.get('rev') in ("s-native-v6-2-7-v77-t1-width", "attitudepd-v2-v78") else ())
        self.command = line
        self.command_ok = self.command_error = False
        self.command_state_seen = False
        self.phase = "busy"
        self.deadline = now + (75 if line in POSTURES else 30 if line.startswith(("trot", "crab", "turn")) else 10)
        self.refresh_at = None
        self.packet(line + "\n")

    def update(self, x, y, now):
        vector = drive_vector(x, y)
        if self.parameter_walking:
            forward=vector is not None and vector[0]>0 and abs(vector[1])<100
            if not forward:
                if self.probe_from_joystick:self.release(now)
                self.requires_release=False
                return
            if self.probe_running or self.requires_release:return
            if self.phase!='idle' or self.probe_config is None or self.probe_config[3]!='all':return
            try:self.start_probe(now)
            except ValueError as exc:self.error=str(exc);return
            self.probe_from_joystick=True;self.requires_release=True
            return
        if self.probe_running:return
        if vector is None:
            self.release(now)
            return
        if vector == (0, 0):
            self.requires_release = False
        if not self.can_drive or self.requires_release:
            return
        if self.phase == "idle" and vector == (0, 0):
            return
        self.vector = vector
        if self.phase == "idle":
            self.phase = "drive"
            self.refresh_at = None
            self.packet(f"drive {vector[0]} {vector[1]} {self.next_sequence()}\n")
            self.next_heartbeat = now + .2

    def release(self, now):
        self.probe_from_joystick=False
        self.probe_running = False
        self.probe_stop_at = None
        self.requires_release = False
        self.vector = None
        if self.phase == "drive":
            self.packet(f"@S {self.next_sequence()}\n", "stop")
            self.phase = "stopping"
            self.deadline = now + 5

    def stop(self, now):
        """Normal walking STOP waits for the robot's return-to-stand completion."""
        self.pending = None
        self.relax_pending = False
        if self.phase in {"drive", "stopping", "draining"}:
            self.release(now)
            self.requires_release = True
        else:
            self.interrupt(now)

    def interrupt(self, now):
        self.probe_running = False
        self.probe_stop_at = None
        self.probe_expected = None
        if not self.connected:
            return
        if self.phase == 'idle':
            self.requires_release = True
            self._console('hold', now)
            return
        self.relax_pending = False
        self.confirm_pose = None
        self.pending = None
        self.vector = None
        self.requires_release = True
        self.command = None
        self.packet(b"\x03", "interrupt")
        self.phase = "stopping"
        self.deadline = now + 5

    def disconnect(self, now):
        self.disconnect_requested = True
        if self.phase == "idle" or not self.connected:
            self.fatal = True
        elif self.motion_active:
            self.interrupt(now)

    def tick(self, now):
        if self.probe_running and self.probe_stop_at is not None and now >= self.probe_stop_at:
            self.stop(now)
        if self.phase == "drive" and self.vector is not None and now >= self.next_heartbeat:
            a, b = self.vector
            self.packet(f"@D {self.next_sequence()} {a} {b}\n", "update")
            self.next_heartbeat = now + .2
        if self.phase in {"busy", "stopping", "draining"} and now >= self.deadline:
            self.interrupt(now)
            self.error = "완료 응답 시간 초과. 정지 요청 후 연결을 해제합니다."
            self.fatal = True
        if self.phase == "idle" and self.refresh_at is not None and now >= self.refresh_at:
            self._console("syncstate", now)
        if (not self.simulator and self.phase == "idle" and self.synced and self.connected and not self.fatal
                and not self.disconnect_requested and now >= self.next_voltage_poll
                and (self.voltage_at is None or now - self.voltage_at >= 5)):
            self.next_voltage_poll = now + 5
            self._console("read 1", now)

    def feed(self, data, now):
        for kind, line in self.stream.feed(data):
            if kind == "prompt":
                self._prompt(now)
                continue
            if line.startswith("$SIMLINK disconnected"):
                self.error = "가상 로봇 연결 종료"
                self.fatal = True
            if line.startswith("$SPOTSTATE "):
                self.state = dict(word.split("=", 1) for word in line.split()[1:] if "=" in word)
                self.caps = set(self.state.get("caps", "").split(","))
                self.state_at = now
                self.synced = True
                if self.command == 'syncstate':
                    self.command_state_seen = True
            if line.startswith('$PROBECONFIG '):
                try:
                    fields = dict(v.split('=',1) for v in line.split()[1:] if '=' in v)
                    parsed = self.parse_probe([fields[k] for k in ['lift_mm','linear','duration_ms','legs']] + ([fields['width_mm'],fields.get('fr_extra','0')] if 'width_mm' in fields else []))
                    if self.command == 'probeconfig show':self.probe_config = parsed
                except (ValueError,KeyError):
                    self.command_error = True
                    self.probe_config = None
            value = battery_reading(line)
            if value and self.connected and not self.simulator:
                mv, historical = value
                self.battery.observe(mv, now, historical=historical)
                if not historical:
                    self.voltage = mv / 1000
                    self.voltage_at = now
            if line == "OK" or line == "OK " + (self.command or ""):
                self.command_ok = True
            failed = line.startswith("ERROR:") or line == "unknown command; type help"
            stopped = line.startswith(("$SPOTDRIVE stopped ", "STOPPED:"))
            if failed:
                self.error = line
                self.command_error = True
                self.relax_pending = False
                self.confirm_pose = None
            if failed or stopped:
                self.probe_running = False
                self.probe_stop_at = None
                self.relax_pending = False
                if self.phase in {"drive", "stopping", "draining"}:
                    self.vector = None
                    self.outbox = deque((k, d) for k, d in self.outbox if k != "update")
                    self.requires_release = True
                    if failed or any(s in line.lower() for s in ("reason=tilt", "reason=imu", "reason=safety")):
                        self.pending = None
                        if stopped:
                            self.state['safety'] = 'fault'
                    self.phase = "draining"
                    self.deadline = now + 5
                elif stopped and self.phase == "busy":
                    self.command_error = True

    def _prompt(self, now):
        if self.phase == "draining":
            self.phase = "idle"
            if self.disconnect_requested:
                self.fatal = True
            elif self.pending:
                pending, self.pending = self.pending, None
                try:
                    self.request(pending, now)
                except ValueError as exc:
                    self.error = str(exc)
            else:
                self.refresh_at = now + .1
            return
        if self.phase != "busy":
            return
        command = self.command
        self.command = None
        self.phase = "idle"
        if self.disconnect_requested:
            self.fatal = True
            return
        if self.command_error:
            self.probe_expected = None
            self.probe_config = None
            self.confirm_pose = None
            if command == 'syncstate':
                self.fatal = True
            else:
                self._console('syncstate', now)
            return
        if command and command.startswith(('probeconfig set ', 'probeconfig reset')):
            self._console('probeconfig show',now)
            return
        if command == 'probeconfig show':
            if self.probe_config is None or (self.probe_expected is not None and self.probe_config != self.probe_expected):
                self.probe_config = None
                self.error = '시험 설정 조회 불일치'
            self.probe_expected = None
            return
        if command in POSTURES:
            if not self.command_ok:
                self.error = "자세 완료 ACK가 없어 동작을 확정하지 않았습니다."
                self.relax_pending = False
            else:
                self.confirm_pose = command
            self._console("syncstate", now)
        elif command == "syncstate":
            # A bare OK from real STM32 requires a *subsequent* pose readback.
            if self.confirm_pose:
                confirmed = self.state.get("pose") == self.confirm_pose and self.command_state_seen
                landing = self.confirm_pose == "landing"
                self.confirm_pose = None
                if not confirmed:
                    self.error = "자세 readback 불일치. 후속 동작을 취소했습니다."
                    self.relax_pending = False
                elif landing and self.relax_pending and self.state.get("safety") == "ok":
                    self.relax_pending = False
                    self._console("relax", now)
                    return
                else:
                    self.relax_pending = False
            if not self.command_state_seen:
                self.error = "BLE/TCP는 연결되었지만 로봇 상태 응답이 없습니다."
                self.fatal = True
            else:
                self._console("read 1", now)
        elif command == "read 1" and self.default_profile_pending and self.synced and self.state.get('safety') == 'ok' and self.state.get('pose') in {'stand', 'stand11', 'landing'}:
            self.default_profile_pending = False
            profile = next((p for p in ("attitudepd_v2",) + NATIVE_PROFILES if p in self.caps and
                            (self.simulator or p not in SIMULATOR_NATIVE_PROFILES)), None)
            if profile and self.state.get('profile') != profile and self.caps & {'gaitprofiles', 'simprofiles'}:
                prefix = 'gaitprofile' if 'gaitprofiles' in self.caps else 'simprofile'
                self.request(prefix + ' ' + profile, now)
        elif command != "read 1" and command not in READ_ONLY:
            self.refresh_at = now + .1

    def snapshot(self):
        return dict(connected=self.connected and not self.fatal, synced=self.synced,
                    phase=self.phase, state=dict(self.state), caps=sorted(self.caps),
                    error=self.error, can_drive=self.can_drive, voltage=self.voltage if self.connected and not self.simulator else None,
                    voltage_at=self.voltage_at, state_at=self.state_at,
                    battery_warning=self.battery.snapshot() if self.connected and not self.simulator else {},
                    simulator=self.simulator, vector=self.vector, motion_active=self.motion_active,
                    parameter_walking=self.parameter_walking, supports_probe=self.supports_probe, probe_config=self.probe_config, probe_running=self.probe_running)
