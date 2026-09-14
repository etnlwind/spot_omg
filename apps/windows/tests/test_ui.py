from PySide6.QtCore import Qt
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

def test_joystick_release_and_focus_loss_clear_input():
    app=QApplication.instance() or QApplication([])
    window=Window();window.set_vector((0,1));window.keys.add(Qt.Key.Key_W)
    window.application_state(Qt.ApplicationState.ApplicationInactive)
    assert window.input_vector is None and not window.keys
    assert window.connection.latest is None
    window.close();app.processEvents()
