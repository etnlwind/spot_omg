"""Real Windows MuJoCo + desktop acceptance. No BLE; no hardware commands.

Run from repo root: python -X utf8 apps/windows/tests/qa_desktop.py
"""
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/windows'))
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QProcess
from spot_controller.ui import Window, configure_app


def main():
    output=ROOT/'apps/windows/test-output'
    output.mkdir(exist_ok=True)
    app=QApplication([]);configure_app(app)
    window=Window();window.repo.setText(str(ROOT));window.python.setText(sys.executable)
    window.resize(1240,980)
    window.viewer.setChecked('--viewer' in sys.argv)
    window.process.finished.connect(lambda code,status: print('SIM EXIT',code,status.name,window.sim_console.toPlainText()[-1500:],flush=True))
    window.connection.log.connect(lambda text: print(text,end='',flush=True))
    window.port.setValue(18875);window.video_port.setValue(18876)
    window.show()
    checks=[]
    def wait(predicate, timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():return
            time.sleep(.015)
        raise AssertionError('Timeout: '+json.dumps(window.snapshot,ensure_ascii=False)+'\n'+window.sim_console.toPlainText()[-3000:])
    def idle():return window.snapshot.get('phase')=='idle'
    def step(name):checks.append(name);print('PASS',name,flush=True)
    try:
        window.toggle_sim()
        wait(lambda:window.sim_ready and idle(),60)
        assert window.snapshot['state']['backend']=='sim'
        step('App launches Windows MuJoCo and verifies simulator identity/readback')
        wait(lambda:window.last_frame is not None,30)
        step('Live MuJoCo JPEG rendered in desktop')
        window.send('gaitprofile attitudepd')
        wait(lambda:idle() and window.snapshot['state'].get('profile')=='attitudepd')
        step('Shared C IMU PD DLL loads and profile readback matches')
        window.grab().save(str(output/'windows-mujoco.png'))
        window.set_vector((0,.45))
        wait(lambda:window.snapshot.get('phase')=='drive')
        until=time.monotonic()+1.5
        wait(lambda:time.monotonic()>until)
        assert window.snapshot.get('phase')=='drive',window.snapshot
        step('Held joystick maintains actual simulator drive for 1.5 seconds')
        released=time.monotonic()
        window.release_input()
        wait(lambda:idle() and (window.snapshot.get('state_at') or 0)>released,15)
        assert '$SPOTDRIVE stopped' in window.console.toPlainText()
        step('Release sends stop and waits for simulator completion/prompt')
        # Start a genuinely new gesture, not a replay of the prior target.
        window.set_vector((0,-.35));wait(lambda:window.snapshot.get('phase')=='drive')
        stopped=time.monotonic()
        window.emergency_stop();wait(lambda:idle() and (window.snapshot.get('state_at') or 0)>stopped,15)
        step('Reverse restart and emergency Ctrl+C stop acknowledged')
        # Losing the GUI event loop must stop the worker heartbeat target.
        window.set_vector((0,.35));wait(lambda:window.snapshot.get('phase')=='drive')
        time.sleep(.9)
        app.processEvents()
        window.release_input();wait(idle,15)
        assert '화면 응답 지연' in window.snapshot['error']
        step('GUI stall triggers independent controller stop')
        window.toggle_sim()
        wait(lambda:not window.connection.running and window.process.state()==QProcess.ProcessState.NotRunning,20)
        step('App disconnects and gracefully shuts down its MuJoCo/video processes')
        window.toggle_sim();wait(lambda:window.sim_ready and idle(),60)
        until=time.monotonic()+8
        wait(lambda:time.monotonic()>until,10)
        assert window.process.state()!=QProcess.ProcessState.NotRunning
        step('Simulator restart and new connection work without input replay')
        window.close()
        wait(lambda:not window.connection.running and window.process.state()==QProcess.ProcessState.NotRunning,20)
        step('Window close cleans up owned simulator')
    finally:
        (output/'desktop-console.txt').write_text(window.console.toPlainText(),encoding='utf-8')
        (output/'mujoco-console.txt').write_text(window.sim_console.toPlainText(),encoding='utf-8')
        (output/'acceptance.json').write_text(json.dumps({'passed':checks},ensure_ascii=False,indent=2),encoding='utf-8')
        window.connection.disconnect()
        deadline=time.monotonic()+8
        while window.connection.running and time.monotonic()<deadline:
            app.processEvents();time.sleep(.02)
        window.stop_sim()
        deadline=time.monotonic()+10
        while window.process.state()!=QProcess.ProcessState.NotRunning and time.monotonic()<deadline:
            app.processEvents();time.sleep(.02)
        window.close();app.processEvents()

if __name__=='__main__':main()
