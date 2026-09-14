from PySide6.QtCore import Qt, QEvent
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication
from spot_controller.ui import Window, configure_app

def test_window_layout_and_idle_start(tmp_path):
    app=QApplication.instance() or QApplication([])
    configure_app(app)
    window=Window();window.show();app.processEvents()
    assert not window.connection.running
    assert not window.joystick.isEnabled()
    assert window.target.currentData()=='tcp'
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
