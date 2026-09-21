import pytest
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QSettings
from spot_controller.foot_lift import FootLiftEditor
from spot_controller.protocol import Controller

def test_editor_saves_only_fresh_matching_readback(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'settings.ini'),QSettings.IniFormat)
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift'],state_at=1,
        state=dict(lift_fl='0',lift_fr='0',lift_rl='0',lift_rr='0'))
    editor.update_state(state)
    editor.inputs[0].setValue(300);editor.submit()
    assert sent==['footlift set 300 0 0 0']
    editor.update_state(state)
    assert settings.value('footLiftMm') is None
    state['state_at']=2;state['state']['lift_fl']='300';editor.update_state(state)
    assert settings.value('footLiftMm')=='[300, 0, 0, 0]'
    state['caps']=[];editor.update_state(state);assert not editor.apply.isEnabled()
    editor.close()

def test_controller_rejects_unsupported_or_moving_lift():
    c=Controller();c.phase='idle';c.caps=set()
    with pytest.raises(ValueError):c.validate('footlift set 10 0 0 0')
    c.caps={'footlift'}
    c.validate('footlift set 30 300 0 2147483647')
    for command in ('footlift set 2147483648 0 0 0','footlift set -1 0 0 0','footlift set 1 2 3','footlift set 1 2 3 4 extra'):
        with pytest.raises(ValueError):c.validate(command)
    c.phase='drive'
    with pytest.raises(ValueError):c.validate('footlift set 0 0 0 0')


def test_saved_values_auto_apply_once_per_connection_when_idle(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'auto.ini'),QSettings.IniFormat)
    settings.setValue('footLiftMm','[20, 20, 0, 0]')
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='drive',caps=['footlift'],state_at=1,
        state=dict(lift_fl='0',lift_fr='0',lift_rl='0',lift_rr='0'))
    editor.update_state(state);assert not sent
    state['phase']='idle';editor.update_state(state)
    assert sent==['footlift set 20 20 0 0']
    editor.update_state(state);assert len(sent)==1 and editor.pending is not None
    state['state_at']=2;state['state'].update(lift_fl='20',lift_fr='20')
    editor.update_state(state);assert editor.pending is None
    editor.update_state(dict(connected=False))
    state['state'].update(lift_fl='0',lift_fr='0');state['state_at']=3
    editor.update_state(state);assert len(sent)==2
    editor.close()


def test_matching_or_invalid_saved_values_are_not_sent(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'equal.ini'),QSettings.IniFormat)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift'],state_at=1,
        state=dict(lift_fl='0',lift_fr='0',lift_rl='0',lift_rr='0'))
    for saved in ('[0,0,0,0]','null','[-1,0,0,0]'):
        settings.setValue('footLiftMm',saved)
        editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
        editor.update_state(state);assert not sent;editor.close()


def test_reopen_discards_unsent_edits_and_shows_robot_values(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'reopen.ini'),QSettings.IniFormat)
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift'],state_at=1,
        state=dict(lift_fl='10',lift_fr='20',lift_rl='0',lift_rr='0'))
    editor.update_state(state);editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[10,20,0,0]
    editor.inputs[0].setValue(100)
    assert '아직 반영되지 않음' in editor.status.text()
    editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[10,20,0,0] and not sent
    settings.setValue('footLiftMm','[5,6,0,0]')
    editor.update_state(dict(connected=False));editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[5,6,0,0]
    assert '연결 후' in editor.status.text()
    editor.close()
