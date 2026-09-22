import pytest
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QSettings
from spot_controller.foot_lift import FootLiftEditor
from spot_controller.protocol import Controller


def test_packaged_artwork_matches_apple_and_has_windows_icon():
    from pathlib import Path
    from PIL import Image
    from spot_controller.assets import resource_path
    assets=Path(__file__).resolve().parents[3]/'apps/ios/SpotOMGController/SpotOMGController/Resources/Assets.xcassets'
    assert resource_path('robot-top.png').read_bytes()==(assets/'RobotTopView.imageset/robot-top.png').read_bytes()
    assert resource_path('app-icon.png').read_bytes()==(assets/'AppIcon.appiconset/AppIcon-1024.png').read_bytes()
    with Image.open(resource_path('SpotOMG.ico')) as icon:
        assert icon.format=='ICO'
        assert {(16,16),(32,32),(48,48),(256,256)}<=icon.ico.sizes()


def test_dark_foot_editor_fits_640_pixel_dialog_and_preserves_leg_sides(tmp_path):
    from PySide6.QtWidgets import QDialog,QVBoxLayout
    from spot_controller.foot_lift import RobotTop
    app=QApplication.instance() or QApplication([])
    dialog=QDialog();dialog.resize(640,570)
    editor=FootLiftEditor(QSettings(str(tmp_path/'layout.ini'),QSettings.IniFormat))
    QVBoxLayout(dialog).addWidget(editor)
    dialog.show();app.processEvents()
    try:
        assert dialog.width()==640
        top=editor.findChild(RobotTop)
        assert not top.pixmap.isNull() and top.height()>=310
        assert editor.inputs[0].x()<top.x()<editor.inputs[1].x()
        assert editor.inputs[2].x()<top.x()<editor.inputs[3].x()
        assert editor.inputs[0].y()<editor.inputs[2].y()
        for field in editor.inputs:
            assert editor.rect().contains(field.geometry())
        for button in (editor.apply,editor.load,editor.zero):
            assert editor.rect().contains(button.geometry())
        shot=editor.grab().toImage()
        assert shot.pixelColor(0,0).name()=='#171c23'
        assert editor.grab().save(str(tmp_path/'foot-lift-dark.png'))
    finally:
        dialog.close();app.processEvents()

def test_editor_saves_only_fresh_matching_readback(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'settings.ini'),QSettings.IniFormat)
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift','footliftpersist'],state_at=1,
        state=dict(lift_fl='0',lift_fr='0',lift_rl='0',lift_rr='0'))
    editor.update_state(state)
    editor.inputs[0].setValue(300);editor.submit()
    assert sent==['footlift save 300 0 0 0']
    editor.update_state(state)
    assert settings.value('footLiftMm') is None
    state['state_at']=2;state['state']['lift_fl']='300';state['foot_lift_result']=dict(ok=True,values=[300,0,0,0]);editor.update_state(state)
    assert settings.value('footLiftMm') is None
    assert editor.pending is None and editor.actual==[300,0,0,0]
    state['caps']=[];editor.update_state(state);assert not editor.apply.isEnabled()
    editor.close()

def test_controller_rejects_unsupported_or_moving_lift():
    c=Controller();c.phase='idle';c.caps=set()
    with pytest.raises(ValueError):c.validate('footlift save 10 0 0 0')
    c.caps={'footlift','footliftpersist'}
    c.validate('footlift save 30 300 0 2147483647')
    for command in ('footlift save 2147483648 0 0 0','footlift save -1 0 0 0','footlift save 1 2 3','footlift save 1 2 3 4 extra'):
        with pytest.raises(ValueError):c.validate(command)
    c.phase='drive'
    with pytest.raises(ValueError):c.validate('footlift save 0 0 0 0')


def test_legacy_local_values_never_overwrite_robot(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'auto.ini'),QSettings.IniFormat)
    settings.setValue('footLiftMm','[20, 20, 0, 0]')
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift','footliftpersist'],state_at=1,
        state=dict(lift_fl='3',lift_fr='4',lift_rl='5',lift_rr='6'))
    editor.update_state(state)
    assert not sent and [v.value() for v in editor.inputs]==[3,4,5,6]
    editor.update_state(dict(connected=False));editor.update_state(state)
    assert not sent
    # A change made by another client becomes the clean editor's current value.
    state['state_at']=2;state['state']['lift_fl']='10';editor.update_state(state)
    assert editor.inputs[0].value()==10
    editor.close()


def test_matching_or_invalid_saved_values_are_not_sent(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'equal.ini'),QSettings.IniFormat)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift','footliftpersist'],state_at=1,
        state=dict(lift_fl='0',lift_fr='0',lift_rl='0',lift_rr='0'))
    for saved in ('[0,0,0,0]','null','[-1,0,0,0]'):
        settings.setValue('footLiftMm',saved)
        editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
        editor.update_state(state);assert not sent;editor.close()


def test_reopen_discards_unsent_edits_and_shows_robot_values(tmp_path):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'reopen.ini'),QSettings.IniFormat)
    editor=FootLiftEditor(settings);sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift','footliftpersist'],state_at=1,
        state=dict(lift_fl='10',lift_fr='20',lift_rl='0',lift_rr='0'))
    editor.update_state(state);editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[10,20,0,0]
    editor.inputs[0].setValue(100)
    assert '아직 반영되지 않음' in editor.status.text()
    editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[10,20,0,0] and sent==['footlift show','footlift show']
    settings.setValue('footLiftMm','[5,6,0,0]')
    editor.update_state(dict(connected=False));editor.prepare_open()
    assert [v.value() for v in editor.inputs]==[0,0,0,0]
    assert '연결 후' in editor.status.text()
    editor.close()


def test_protocol_requires_saved_ack_and_fresh_readback():
    c=Controller();c.connected=c.synced=True;c.phase='idle';c.caps={'footlift','footliftpersist'}
    state=b'$SPOTSTATE pose=stand torque=on safety=ok caps=footlift,footliftpersist lift_fl=10 lift_fr=20 lift_rl=0 lift_rr=0\n'
    c.request('footlift save 10 20 0 0',0)
    c.feed(b'OK footlift saved\n'+state+b'# ',.1)
    assert c.command=='footlift show' and c.foot_lift_result is None
    c.feed(state+b'# ',.2)
    assert c.foot_lift_result==dict(ok=True,values=[10,20,0,0])
    c.request('footlift save 10 20 0 0',.3)
    c.feed(b'ERROR: flash failed\n# ',.4)
    c.feed(state+b'# ',.5)
    assert c.foot_lift_result==dict(ok=False,values=[10,20,0,0])
    c.phase='idle';c.request('footlift save 10 20 0 0',.6)
    c.feed(b'OK footlift saved\n# ',.7)
    c.feed(b'# ',.8) # Old matching state does not count as new readback.
    assert c.foot_lift_result['ok'] is False


def test_width_editor_per_leg_restore_and_atomic_readback(tmp_path):
    app=QApplication.instance() or QApplication([])
    editor=FootLiftEditor(QSettings(str(tmp_path/'width.ini'),QSettings.IniFormat))
    sent=[];editor.applyRequested.connect(sent.append)
    state=dict(connected=True,synced=True,phase='idle',caps=['footlift','footliftpersist','footwidth'],state_at=1,
               state=dict(lift_fl='1',lift_fr='2',lift_rl='3',lift_rr='4',width_fl='-5',width_fr='6',width_rl='-7',width_rr='8'))
    editor.update_state(state)
    assert [f.value() for f in editor.width_inputs]==[-5,6,-7,8]
    editor.width_inputs[0].setValue(-12);editor.prepare_open()
    assert editor.width_inputs[0].value()==-5
    editor.width_inputs[1].setValue(-9);editor.submit()
    assert sent[-1]=='footlift save 1 2 3 4 -5 -9 -7 8'
    editor.update_state(state);assert editor.pending is not None
    state['state']['width_fr']='-9';state['state_at']=2
    state['foot_lift_result']=dict(ok=True,values=[1,2,3,4,-5,-9,-7,8])
    editor.update_state(state);assert editor.pending is None
    assert 'FR -9mm' in editor.status.text()
    state['caps'].remove('footwidth');editor.update_state(state)
    assert all(not f.isEnabled() for f in editor.width_inputs)
    editor.close()


def test_width_protocol_requires_all_eight_fresh_values():
    c=Controller();c.connected=c.synced=True;c.phase='idle';c.caps={'footlift','footliftpersist'}
    command='footlift save 1 2 3 4 -5 6 -7 8'
    with pytest.raises(ValueError):c.validate(command)
    c.caps.add('footwidth');c.validate(command)
    for invalid in ('footlift save 1 2 3 4 -5 6 7','footlift save 1 2 3 4 -2147483648 0 0 0'):
        with pytest.raises(ValueError):c.validate(invalid)
    c.request(command,0)
    c.feed(b'OK footlift saved\n# ',.1)
    assert c.command=='footlift show'
    c.feed(b'$SPOTSTATE pose=stand torque=on safety=ok caps=footlift,footliftpersist,footwidth lift_fl=1 lift_fr=2 lift_rl=3 lift_rr=4 width_fl=-5 width_fr=6 width_rl=-7 width_rr=8\n# ',.2)
    assert c.foot_lift_result==dict(ok=True,values=[1,2,3,4,-5,6,-7,8])


def test_full_integer_foot_settings_fit_dedicated_channel():
    from spot_controller.control_channel import request
    command='footlift save '+' '.join(['2147483647']*4+['-2147483647']*4)
    c=Controller();c.phase='idle';c.caps={'footlift','footliftpersist','footwidth'}
    c.validate(command)
    frame=request(4294967295,command.encode())
    assert len(command)<128 and len(frame)<144
