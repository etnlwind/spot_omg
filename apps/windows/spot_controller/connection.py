"""One asynchronous connection owner, independent of Qt's GUI thread."""
import asyncio
from collections import deque
import threading
import time
import secrets

from PySide6.QtCore import QObject, Signal
from .protocol import Controller, ConsoleStream
from .control_channel import ControlStream, request as control_request


class TcpLink:
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.reader = self.writer = None

    async def open(self):
        self.reader, self.writer = await asyncio.wait_for(asyncio.open_connection(self.host, self.port), 5)
        self.writer.write(b"identity\n")
        await self.writer.drain()
        stream = ConsoleStream()
        identified = False
        async def identify():
            nonlocal identified
            while True:
                data = await self.reader.read(4096)
                if not data:
                    raise ConnectionError("시뮬레이터가 연결을 종료했습니다.")
                for kind, line in stream.feed(data):
                    if kind == "line" and line.startswith("$SPOTBACKEND "):
                        identified = {"backend=sim", "protocol=1"} <= set(line.split())
                    if kind == "prompt":
                        if not identified:
                            raise ConnectionError("Spot OMG 시뮬레이터 identity 검증 실패")
                        return
        await asyncio.wait_for(identify(), 5)

    async def read(self):
        data = await self.reader.read(4096)
        if not data:
            raise ConnectionError("TCP 연결이 종료되었습니다.")
        return data

    async def write(self, data):
        self.writer.write(data)
        await self.writer.drain()

    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()


class BleLink:
    def __init__(self, simulator=False, address=""):
        prefix = "6e4001" if simulator else "6e4000"
        suffix = "-b5a3-f393-e0a9-e50e24dcca9e"
        self.service, self.rx, self.tx = [prefix + n + suffix for n in ("01", "02", "03")]
        self.name = "SpotOMG-Sim" if simulator else "SpotOMG-Bridge"
        self.address = address.strip()
        self.client = None
        self.received = asyncio.Queue(maxsize=256)
        self.simulator = simulator
        self.control_characteristic = None
        self.control_received = asyncio.Queue(maxsize=32)
        self.control_mode = False
        self.log_dropped = 0

    def received_control(self, _, data):
        try:
            self.control_received.put_nowait(bytes(data))
        except asyncio.QueueFull:
            asyncio.create_task(self.client.disconnect())

    async def read_control(self):
        return await self.control_received.get()

    async def write_control(self, data):
        await asyncio.wait_for(self.client.write_gatt_char(
            self.control_characteristic, data, response=True), 2)

    def received_data(self, _, data):
        try:
            self.received.put_nowait(bytes(data))
        except asyncio.QueueFull:
            # Closing the physical connection stops notification flooding too.
            if self.control_mode:
                self.log_dropped += len(data)
            else:
                asyncio.create_task(self.client.disconnect())

    def disconnected(self, _):
        while not self.received.empty():
            self.received.get_nowait()
        self.received.put_nowait(None)

    async def open(self):
        from bleak import BleakClient, BleakScanner
        devices = await BleakScanner.discover(timeout=5., return_adv=True)
        matches = [d for d, a in devices.values()
                   if self.service in [u.lower() for u in a.service_uuids]
                   and ((self.address and d.address.lower() == self.address.lower())
                        or (not self.address and (a.local_name or d.name) == self.name))]
        if len(matches) != 1:
            addresses = ", ".join(d.address for d in matches)
            raise ConnectionError(f"{self.name}: 일치 장치 {len(matches)}개. 로봇 전원·Bluetooth 및 iPhone 연결 해제를 확인하십시오. "
                                  + ("장치 주소를 지정하십시오: " + addresses if matches else ""))
        self.client = BleakClient(matches[0], disconnected_callback=self.disconnected, timeout=10,
                                 winrt={"use_cached_services": False})
        await self.client.connect()
        self.characteristic = self.client.services.get_characteristic(self.rx)
        if self.characteristic is None:
            raise ConnectionError("BLE UART RX characteristic 누락")
        self.response = "write" in self.characteristic.properties
        if not self.response and "write-without-response" not in self.characteristic.properties:
            raise ConnectionError("BLE UART 쓰기 미지원")
        await self.client.start_notify(self.tx, self.received_data)
        if not self.simulator:
            control_tx = '6e400006-b5a3-f393-e0a9-e50e24dcca9e'
            control_rx = self.client.services.get_characteristic('6e400005-b5a3-f393-e0a9-e50e24dcca9e')
            if control_rx and self.client.services.get_characteristic(control_tx):
                await self.client.start_notify(control_tx, self.received_control)
                self.control_characteristic = control_rx
        if self.simulator:
            await self.write(b"identity\n")
            stream = ConsoleStream()
            identified = False
            async def identify():
                nonlocal identified
                while True:
                    for kind, line in stream.feed(await self.read()):
                        if kind == "line" and line.startswith("$SPOTBACKEND "):
                            identified = {"backend=sim", "protocol=1"} <= set(line.split())
                        if kind == "prompt":
                            if not identified:
                                raise ConnectionError("가상 BLE identity 검증 실패")
                            return
            await asyncio.wait_for(identify(), 5)

    async def read(self):
        data = await self.received.get()
        if data is None:
            raise ConnectionError("BLE 연결이 끊겼습니다. 재연결 후 새로 조작하십시오.")
        if self.log_dropped:
            notice = f'\n[BLE LOG overflow: {self.log_dropped} bytes lost]\n'.encode()
            self.log_dropped = 0
            data = notice + data
        return data

    async def write(self, data):
        maximum = 180 if self.response else self.characteristic.max_write_without_response_size
        for offset in range(0, len(data), maximum):
            await asyncio.wait_for(self.client.write_gatt_char(
                self.characteristic, data[offset:offset + maximum], response=self.response), 2)

    async def close(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()


class Connection(QObject):
    changed = Signal(dict)
    log = Signal(str)
    finished = Signal()

    def __init__(self):
        super().__init__()
        self.thread = None
        self.lock = threading.Lock()
        self.intents = deque()
        self.latest = None
        self.pulse_at = time.monotonic()
        self.emergency = False
        self.walk_stop = False
        self.closing = False

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def connect_to(self, target, host, port, address=""):
        if self.running:
            return
        with self.lock:
            self.intents.clear()
            self.latest = None
            self.closing = self.emergency = False
            self.walk_stop = False
            self.pulse_at = time.monotonic()
        self.thread = threading.Thread(target=self._run, args=(target, host, port, address), daemon=True)
        self.thread.start()

    def pulse(self, vector):
        with self.lock:
            self.latest = vector
            self.pulse_at = time.monotonic()

    def trace(self, message):
        self.log.emit(f"\n[앱 {time.strftime('%H:%M:%S')} +{time.monotonic():.3f}] {message}\n")

    def command(self, line):
        with self.lock:
            if len(self.intents) < 16:
                self.intents.append((line, time.monotonic()))

    def stop(self, graceful=False):
        with self.lock:
            if graceful:
                self.walk_stop = True
            else:
                self.emergency = True
            self.latest = None
            self.intents.clear()

    def disconnect(self):
        with self.lock:
            self.closing = True
            self.latest = None
            self.intents.clear()

    def _run(self, target, host, port, address):
        try:
            asyncio.run(self._session(target, host, port, address))
        finally:
            self.finished.emit()

    async def _session(self, target, host, port, address):
        model = Controller(simulator=target != "robot")
        link = TcpLink(host, port) if target == "tcp" else BleLink(target == "simble", address)
        read_task = None
        control_task = None
        control_stream = ControlStream()
        control_payload = ConsoleStream()
        control_mode = False
        control_sequence = secrets.randbelow(0xfffffffe) + 1
        active_control = None
        log_stream = ConsoleStream()
        self.changed.emit(dict(phase="connecting", connected=False, state={}, caps=[], error="", can_drive=False))
        try:
            opening = asyncio.create_task(link.open())
            started = time.monotonic()
            while not opening.done():
                await asyncio.sleep(.05)
                if self.closing or time.monotonic() - started > 25:
                    opening.cancel()
                    try:
                        await opening
                    except asyncio.CancelledError:
                        pass
                    if self.closing:
                        return
                    raise TimeoutError("연결 시간 초과")
            await opening
            model.opened(time.monotonic())
            read_task = asyncio.create_task(link.read())
            next_snapshot = 0.
            last_released = False
            previous_phase = model.phase
            last_drive_data = None
            last_rx = time.monotonic()
            while True:
                now = time.monotonic()
                if read_task.done():
                    data = read_task.result()
                    last_rx = now
                    self.log.emit(data.decode("utf-8", errors="replace"))
                    if not control_mode:
                        model.feed(data, now)
                    else:
                        try:
                            log_events = log_stream.feed(data)
                        except ValueError:
                            log_stream = ConsoleStream()
                            log_events = []
                            self.trace('콘솔의 미완성 줄이 너무 길어 배터리 파서만 초기화했습니다.')
                        for kind, line in log_events:
                            if kind == 'line' and line.startswith('$BATTERY '):
                                model.feed((line+'\n').encode(), now)
                    read_task = asyncio.create_task(link.read())
                if (not control_mode and model.synced and model.command != 'syncstate'
                        and 'controlv1' in model.caps and getattr(link, 'control_characteristic', None)):
                    control_mode = True
                    link.control_mode = True
                    model.separate_control = True
                    control_task = asyncio.create_task(link.read_control())
                    self.trace('전용 제어 채널 활성화 · 콘솔 로그와 완료 응답 분리')
                if control_task is not None and control_task.done():
                    for sequence, kind, data in control_stream.feed(control_task.result()):
                        if sequence != active_control:
                            continue
                        if kind == 'DATA':
                            for event, line in control_payload.feed(data):
                                if event == 'line':
                                    model.feed((line+'\n').encode(), now)
                        else:
                            active_control = None
                            model.feed(b'\r\n# ', now)
                    control_task = asyncio.create_task(link.read_control())
                with self.lock:
                    vector, pulse, closing = self.latest, self.pulse_at, self.closing
                    emergency, self.emergency = self.emergency, False
                    walk_stop, self.walk_stop = self.walk_stop, False
                    intents = list(self.intents)
                    self.intents.clear()
                if closing and not model.disconnect_requested:
                    model.disconnect(now)
                if emergency:
                    self.trace("정지 원인: 긴급 정지/앱 비활성화 요청")
                    model.interrupt(now)
                    intents = []
                elif walk_stop:
                    self.trace("정지 원인: Stop 버튼")
                    model.stop(now)
                    intents = []
                if now - pulse > .6:
                    vector = None
                    if model.phase in {"drive", "busy"} and model.motion_active:
                        self.trace(f"정지 원인: UI heartbeat 지연 {now-pulse:.3f}s")
                        model.interrupt(now)
                        model.error = "화면 응답 지연으로 정지했습니다. 스틱을 놓고 다시 조작하십시오."
                if vector is None:
                    if not last_released:
                        if model.phase == 'drive':
                            self.trace("정지 원인: UI 입력 해제(None)")
                        model.release(now)
                    last_released = True
                else:
                    if last_released and model.phase == 'idle':
                        model.release(now)
                    last_released = False
                    model.update(*vector, now)
                for line, issued in intents:
                    if now - issued > .5 or emergency or closing:
                        continue
                    try:
                        model.request(line, now)
                    except ValueError as exc:
                        model.error = str(exc)
                        self.log.emit("[거부] " + str(exc) + "\n")
                model.tick(now)
                if model.phase != previous_phase:
                    self.trace(f"상태 {previous_phase} → {model.phase}; 마지막 수신 {now-last_rx:.3f}s 전")
                    previous_phase = model.phase
                if model.fatal and model.error:
                    self.trace(model.error)
                if model.outbox:
                    kind, data = model.outbox.popleft()
                    if kind != "update":
                        self.log.emit("> " + data.decode("utf-8").replace("\x03", "^C") + ("\n" if data == b"\x03" else ""))
                    else:
                        axes = data.split()[2:]
                        if axes != last_drive_data:
                            self.trace("보행 입력: " + data.decode().strip())
                            last_drive_data = axes
                    if control_mode:
                        if kind == 'command':
                            control_sequence = control_sequence % 0xffffffff + 1
                            active_control = control_sequence
                            control_payload = ConsoleStream()
                            data = control_request(active_control, data)
                        await asyncio.wait_for(link.write_control(data), 2)
                    else:
                        await asyncio.wait_for(link.write(data), 2)
                if now >= next_snapshot:
                    self.changed.emit(model.snapshot())
                    next_snapshot = now + .1
                if model.fatal:
                    break
                await asyncio.sleep(.01)
        except Exception as exc:
            model.error = str(exc) or type(exc).__name__
            self.log.emit("[연결 오류] " + model.error + "\n")
        finally:
            if control_task:
                control_task.cancel()
                try:
                    await control_task
                except (asyncio.CancelledError, Exception):
                    pass
            if read_task:
                read_task.cancel()
                try:
                    await read_task
                except (asyncio.CancelledError, Exception):
                    pass
            try:
                await asyncio.wait_for(link.close(), 3)
            except Exception:
                pass
            model.connected = model.synced = False
            model.state = {}
            model.caps.clear()
            model.voltage = model.voltage_at = None
            model.phase = "offline"
            self.changed.emit(model.snapshot())
