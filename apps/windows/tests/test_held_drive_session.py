"""Exercise Qt input, worker loop, and a lost real diagnostic tail together.

Only the BLE endpoint is substituted. No physical robot is connected/moved.
"""
import asyncio
from pathlib import Path
import time

from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from spot_controller import connection
from spot_controller.ui import Window

STATE = b'$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v4-v80 caps=gaitprofiles,attitudepd_v4 profile=attitudepd_v4\r\n# '


def test_held_half_to_full_drive_then_lost_tail_recovery(monkeypatch, tmp_path):
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.show()
    app.processEvents()
    sent = []
    recovered = False
    phases = []
    window.connection.changed.connect(lambda s: phases.append(s['phase']))
    fixture = (Path(__file__).parent/'fixtures/v80-stop-tail-missing.txt').read_bytes()

    class Radio:
        def __init__(self, *args):
            self.queue = asyncio.Queue()

        async def open(self):
            pass

        async def read(self):
            return await self.queue.get()

        async def write(self, data):
            nonlocal recovered
            sent.append(data)
            if data == b'syncstate\n':
                self.queue.put_nowait(STATE)
            elif data == b'read 1\n':
                self.queue.put_nowait(b'ID 1 voltage=12000mV\r\n# ')
            elif data.startswith(b'@S '):
                for offset in range(0, len(fixture), 120):
                    self.queue.put_nowait(fixture[offset:offset+120])
                # Deliberately no tail, newline, or prompt after ID11.
            elif data == b'\nsyncstate\n':
                recovered = True
                self.queue.put_nowait(b'\r\n# ')  # Must not unlock here.
                self.queue.put_nowait(STATE)

        async def close(self):
            pass

    monkeypatch.setattr(connection, 'BleLink', Radio)

    async def scenario():
        task = asyncio.create_task(window.connection._session('robot', '', 0, ''))

        async def until(predicate, timeout=3):
            deadline = time.monotonic() + timeout
            while not predicate():
                app.processEvents()
                if task.done():
                    await task
                    raise AssertionError('connection ended before expected state\n' + window.console.toPlainText())
                assert time.monotonic() < deadline, window.console.toPlainText()
                await asyncio.sleep(.01)

        try:
            await until(lambda: window.snapshot.get('phase') == 'idle')
            stick = window.joystick
            cx, cy = stick.width()/2, stick.height()/2
            travel = (min(cx, cy)-17)*.64
            QTest.mousePress(stick, Qt.MouseButton.LeftButton,
                             pos=QPoint(round(cx), round(cy-travel*.5)))
            await until(lambda: any(d.startswith(b'drive ') for d in sent))
            started = time.monotonic()
            full = False
            while time.monotonic() - started < 11:
                app.processEvents()
                if not full and time.monotonic() - started > 1:
                    QTest.mouseMove(stick, QPoint(round(cx), round(cy-travel)))
                    full = True
                assert stick.held and window.input_vector is not None
                assert not any(d.startswith(b'@S ') or d == b'\x03' for d in sent)
                await asyncio.sleep(.02)
            updates = [d.split() for d in sent if d.startswith(b'@D ')]
            assert any(570 <= int(d[2]) <= 610 for d in updates)
            assert any(int(d[2]) >= 990 for d in updates)
            QTest.mouseRelease(stick, Qt.MouseButton.LeftButton,
                               pos=QPoint(round(cx), round(cy-travel)))
            await until(lambda: recovered and window.snapshot.get('phase') == 'idle', 8)
            assert '→ resync;' in window.console.toPlainText()
            assert window.snapshot['connected'] and window.snapshot['synced']
            assert b'\x03' not in sent
            assert sent.count(b'\nsyncstate\n') == 1
            assert sum(d.startswith(b'drive ') for d in sent) == 1
            assert 'mouseReleaseEvent' in window.console.toPlainText()
        finally:
            window.connection.disconnect()
            await asyncio.wait_for(task, 2)

    try:
        asyncio.run(scenario())
    finally:
        (tmp_path/'session-console.txt').write_text(window.console.toPlainText(), encoding='utf-8')
        window.close()
        app.processEvents()
