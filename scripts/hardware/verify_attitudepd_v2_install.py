"""Verify V2 install from mechanically reached Landing; does not walk."""
import json
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from scripts.hardware.measure_rr_j3_response import registers
OUT=Path(__file__).resolve().parents[2]/'artifacts/attitudepd-v2/hardware-eight-steps'
rows=[]
prior=json.loads((OUT/'pre-update-registers.json').read_text())
def save(): (OUT/'installation-readback.json').write_text(json.dumps(rows,indent=2))
with Stm32Console('SpotOMG-Bridge',transport=BleTransport('SpotOMG-Bridge')) as c:
    def send(cmd,timeout=15):
        r=c.send(cmd,timeout=timeout,on_line=lambda s:print(s,flush=True));rows.append(dict(command=cmd,ok=r.ok,text=r.text));save()
        if not r.ok:raise RuntimeError(r.text)
        return r.text
    state=send('syncstate')
    assert 'rev=attitudepd-v2-v78' in state.split(),state
    assert 'attitudepd_v2' in state and 'safety=ok' in state.split(),state
    send('landing',75)
    state=send('syncstate')
    assert {'pose=landing','torque=on','safety=ok','fault_code=0'}.issubset(state.split()),state
    for i in range(1,13):
        raw=registers(send(f'servoconfig {i}'),i)
        assert raw[:40]==prior[str(i)][:40],f'Servo {i} permanent configuration differs'
        assert raw[40]==1,f'Servo {i} torque not enabled at Landing'
    send('targets')
    send('status')
print('V2 installed; Landing completed; all permanent servo registers preserved.')
