"""Exercise mode visibility and wire commands; never connect to hardware."""
from pathlib import Path
import re

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from spot_controller.profile_names import PROFILE_TITLES
from spot_controller.protocol import PROFILES
from spot_controller.ui import Window, configure_app


def ready_state(parameter=False):
    return dict(connected=True, synced=True, phase='idle', supports_probe=True,
                controls_enabled=True, can_drive=True, parameter_walking=parameter,
                caps=['gaitprofiles', 'attitudepd_v6', 'attitudepd_v5', 'footliftpersist'],
                state=dict(pose='stand', safety='ok', torque='on', rev='attitudepd-v6-v92',
                           profile='attitudepd_v6', lift_fl='0', lift_fr='0', lift_rl='0', lift_rr='0'))


def test_model_names_match_apple_without_changing_wire_keys():
    source = (Path(__file__).resolve().parents[2] / 'ios/SpotOMGController/SpotOMGController/Models/ConnectionState.swift').read_text()
    for key in PROFILES:
        assert PROFILE_TITLES[key] == re.search(r'case \.' + key + r': return "([^"]+)"', source)[1]


def test_modes_hide_other_settings_and_send_original_model_key():
    app = QApplication.instance() or QApplication([])
    configure_app(app)
    window = Window()
    sent = []
    window.send = sent.append
    try:
        window.show()
        window.on_state(ready_state())
        app.processEvents()
        assert window.profiles.isVisible()
        assert not window.probe_lift.isVisible()
        assert window.profiles.currentItem().text().startswith('IMU 자세 안정화 V6')
        assert 'IMU 자세 안정화 V6' in window.active_model.text()
        window.profiles.setCurrentRow(PROFILES.index('attitudepd_v5'))
        window.profile_button.click()
        assert sent[-1] == 'gaitprofile attitudepd_v5'
        window.probe_mode.setCurrentIndex(1)
        assert sent[-1] == 'app_parameter_mode 1'
        window.on_state(ready_state(parameter=True))
        app.processEvents()
        assert window.probe_lift.isVisible()
        assert not window.profiles.isVisible()
        assert window.probe_apply.isEnabled()
        count = len(sent)
        window.on_state(dict(phase='offline', state={}, caps=[], connected=False))
        assert len(sent) == count  # Disconnect readback must not issue mode commands.
        assert not window.connection.running
    finally:
        window.close()
        app.processEvents()


def test_capability_gating_and_imu_recovery_feedback():
    app = QApplication.instance() or QApplication([])
    window = Window()
    sent = []
    window.send = sent.append
    try:
        state = ready_state()
        state.update(can_recover_imu=True, imu_recovery_message='복구 가능')
        window.on_state(state)
        window.set_vector((0, 1))
        window.imu_recovery_button.click()
        assert sent == ['imurecover']
        assert window.input_vector is None
        state.update(imu_recovery_pending=True, can_recover_imu=False,
                     controls_enabled=False, imu_recovery_message='IMU 복구 중')
        window.on_state(state)
        assert not window.imu_recovery_button.isEnabled()
        assert not window.joystick.isEnabled()
        assert window.imu_recovery_status.text() == 'IMU 복구 중'
        state['caps'].remove('attitudepd_v6')
        window.on_state(state)
        assert not (window.profiles.item(0).flags() & Qt.ItemFlag.ItemIsEnabled)
    finally:
        window.close()
        app.processEvents()


def test_diagnostic_export_ui_remains_asynchronous():
    app = QApplication.instance() or QApplication([])
    window = Window()
    try:
        window.connection.diagnostics_exporting.emit(True)
        assert not window.diagnostic_export_button.isEnabled()
        window.connection.diagnostics_exporting.emit(False)
        window.connection.diagnostics_exported.emit('/tmp/spot-log.jsonl')
        assert window.diagnostic_export_button.isEnabled()
        assert '저장 완료' in window.diagnostic_status.text()
        assert window.diagnostic_status.toolTip() == '/tmp/spot-log.jsonl'
        assert not window.connection.running
    finally:
        window.close()
        app.processEvents()


def test_keyboard_navigation_in_model_list_never_drives(monkeypatch):
    from PySide6.QtTest import QTest
    app = QApplication.instance() or QApplication([])
    window = Window()
    monkeypatch.setattr(window, 'isActiveWindow', lambda: True)
    sent = []
    window.send = sent.append
    try:
        window.show()
        window.on_state(ready_state())
        window.keyboard.setChecked(True)
        window.profiles.setFocus()
        app.processEvents()
        assert QApplication.focusWidget() is window.profiles
        initial = window.profiles.currentRow()
        QTest.keyClick(window.profiles, Qt.Key.Key_Down)
        assert window.profiles.currentRow() == initial + 1
        assert window.input_vector is None and window.connection.latest is None
        assert not sent
        QTest.keyClick(window.profiles, Qt.Key.Key_Return)
        assert sent == ['gaitprofile attitudepd_v5']
        window.set_vector((0, 1))
        window.keys.add(Qt.Key.Key_W)
        window.pulse()
        assert window.input_vector is None and not window.keys
    finally:
        window.close()
        app.processEvents()
