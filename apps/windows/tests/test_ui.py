from PySide6.QtCore import Qt, QEvent
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication
from spot_controller.ui import Window, configure_app

def test_v7_displays_side_pivot_and_reverse_intent():
    app=QApplication.instance() or QApplication([]);configure_app(app)
    window=Window()
    for vector,title in [((0,588),'오른쪽 옆걸음'),((0,-588),'왼쪽 옆걸음'),
                         ((-600,400),'제자리 우회전'),((-600,-400),'제자리 좌회전'),
                         ((-588,0),'후진 · 보행 속도 50%')]:
        window.on_state(dict(connected=True,synced=True,phase='drive',
            state=dict(pose='stand',torque='on',safety='ok',profile='attitudepd_v7'),
            vector=vector,caps=['attitudepd_v7'],can_drive=True,controls_enabled=True,requires_release=False,error=''))
        assert title in window.drive_status.text()
    window.close()

def test_pause_banner_keeps_controls_available(tmp_path):
    app=QApplication.instance() or QApplication([]);configure_app(app)
    window=Window();window.resize(1240,1100);window.show()
    state=dict(connected=True,synced=True,phase='idle',state=dict(pose='custom',torque='on',safety='tilt'),
               caps=[],can_drive=True,controls_enabled=True,requires_release=True,
               pause_reason='$SPOTDRIVE stopped reason=tilt',error='')
    window.on_state(state);app.processEvents()
    assert window.motion_panel.isVisible() and window.joystick.isEnabled()
    assert '기울기' in window.motion_title.text()
    assert '사용자가 결정' in window.motion_message.text()
    assert window.grab().save(str(tmp_path/'pause-banner.png'))
    state.update(pause_reason='');state['state']['safety']='ok'
    window.on_state(state)
    assert not window.motion_panel.isVisible()
    window.close();app.processEvents()

def test_window_layout_and_idle_start(tmp_path):
    app=QApplication.instance() or QApplication([])
    configure_app(app)
    window=Window();window.show();app.processEvents()
    assert not window.connection.running
    assert not window.joystick.isEnabled()
    assert window.target.currentData()=='robot'
    assert window.process.processId()==0
    assert window.grab().save(str(tmp_path/'desktop.png'))
    window.close();app.processEvents()

def test_keyboard_updates_joystick_and_release(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window()
    monkeypatch.setattr(window, 'isActiveWindow', lambda: True)
    window.snapshot['can_drive'] = True
    window.keyboard.setChecked(True)

    def key(kind, code):
        window.eventFilter(window, QKeyEvent(kind, code, Qt.KeyboardModifier.NoModifier))

    try:
        key(QEvent.Type.KeyPress, Qt.Key.Key_W)
        assert window.joystick.vector == (0, 1)
        key(QEvent.Type.KeyPress, Qt.Key.Key_D)
        assert window.joystick.vector == window.input_vector
        assert abs(window.joystick.vector[0] - 2**-.5) < 1e-9
        key(QEvent.Type.KeyRelease, Qt.Key.Key_W)
        assert window.joystick.vector == (1, 0)
        key(QEvent.Type.KeyRelease, Qt.Key.Key_D)
        assert window.joystick.vector == (0, 0)
        assert window.input_vector is None
        key(QEvent.Type.KeyPress, Qt.Key.Key_Left)
        assert window.joystick.vector == (-1, 0)
        for kind in (QEvent.Type.ShortcutOverride, QEvent.Type.KeyRelease, QEvent.Type.KeyPress):
            repeat = QKeyEvent(kind, Qt.Key.Key_Left, Qt.KeyboardModifier.NoModifier, '', True)
            assert window.eventFilter(window, repeat)
            assert window.joystick.vector == (-1, 0)
            assert Qt.Key.Key_Left in window.keys
        window.application_state(Qt.ApplicationState.ApplicationInactive)
        assert window.joystick.vector == (0, 0)
        assert window.input_vector is None
    finally:
        window.close()
        app.processEvents()

def test_joystick_release_and_focus_loss_clear_input():
    app=QApplication.instance() or QApplication([])
    window=Window();window.set_vector((0,1));window.keys.add(Qt.Key.Key_W)
    window.application_state(Qt.ApplicationState.ApplicationInactive)
    assert window.input_vector is None and not window.keys
    assert window.connection.latest is None
    window.close();app.processEvents()


def test_battery_banner_persists_after_ack_and_escalates(monkeypatch, tmp_path):
    from spot_controller.battery import BatteryWarning
    app = QApplication.instance() or QApplication([])
    calls = []
    monkeypatch.setattr(QApplication, 'beep', lambda: calls.append('sound'))
    monkeypatch.setattr(QApplication, 'alert', lambda *args: None)
    window = Window(); window.show(); app.processEvents()
    b = BatteryWarning(); b.observe(10900, 0)
    try:
        window.snapshot['connected'] = True
        window.show_battery_warning(b.snapshot())
        assert window.battery_panel.isVisible()
        assert '지금 충전' in window.battery_title.text()
        assert '10.9V' in window.battery_message.text()
        assert calls == ['sound']
        window.battery_confirm.click()
        assert window.battery_panel.isVisible()
        assert not window.battery_message.isVisible()
        window.show_battery_warning(b.snapshot())
        assert calls == ['sound']
        b.observe(10300, 1); window.show_battery_warning(b.snapshot())
        assert window.battery_message.isVisible()
        assert '즉시 사용 중단' in window.battery_title.text()
        assert calls == ['sound', 'sound']
        assert window.grab().save(str(tmp_path / 'battery-critical.png'))
        assert not window.connection.running
        for t in [2, 4, 7]: b.observe(12000, t)
        window.show_battery_warning(b.snapshot())
        assert not window.battery_panel.isVisible()
    finally:
        window.close(); app.processEvents()


def test_joystick_latch_keeps_vector_until_explicit_release():
    from PySide6.QtTest import QTest
    from PySide6.QtCore import QPoint
    from spot_controller.ui import Joystick
    app=QApplication.instance() or QApplication([])
    stick=Joystick();stick.resize(200,200);stick.show();app.processEvents()
    values=[];stick.vectorChanged.connect(values.append)
    stick.auto_return=False
    QTest.mousePress(stick,Qt.MouseButton.LeftButton,pos=QPoint(100,40))
    QTest.mouseRelease(stick,Qt.MouseButton.LeftButton,pos=QPoint(100,40))
    assert stick.vector[1]>0 and values[-1] is not None
    stick.release();assert stick.vector==(0.,0.) and values[-1] is None
    stick.auto_return=True
    QTest.mousePress(stick,Qt.MouseButton.LeftButton,pos=QPoint(100,40))
    QTest.mouseRelease(stick,Qt.MouseButton.LeftButton,pos=QPoint(100,40))
    assert values[-1] is None
    stick.close()
