"""No hardware devices: exercise simulator identity on loopback only."""
import socket
import threading
import pytest
from servo.transport import SimulatorTransport
from servo.cli import parse_args


@pytest.mark.parametrize('identity,accepted', [
    (b'$SPOTBACKEND backend=sim protocol=1\r\n# ', True),
    (b'$SPOTBACKEND backend=robot protocol=1\r\n# ', False),
    (b'$SPOTBACKEND backend=sim protocol=2\r\n# ', False),
])
def test_endpoint_identity_before_commands(identity, accepted):
    listener=socket.socket()
    listener.bind(('127.0.0.1',0)); listener.listen(1)
    received=[]
    def serve():
        connection,_=listener.accept()
        with connection:
            received.append(connection.recv(1024))
            connection.sendall(identity)
    worker=threading.Thread(target=serve); worker.start()
    transport=SimulatorTransport('127.0.0.1',listener.getsockname()[1])
    try:
        if accepted:
            transport.open()
        else:
            with pytest.raises(ConnectionError): transport.open()
            assert not transport.is_open
    finally:
        transport.close(); worker.join(3); listener.close()
    assert received == [b'identity\n']


def test_simulator_selection_rejects_conflicting_hardware(monkeypatch):
    for key in ('SPOT_TCP_HOST','SPOT_SERVO_PORT','SPOT_STM32_PORT','SPOT_TRANSPORT'):
        monkeypatch.delenv(key,raising=False)
    args=parse_args(['--sim-host','127.0.0.1','stand'])
    assert args.host == '127.0.0.1'
    with pytest.raises(SystemExit):
        parse_args(['--sim-host','127.0.0.1','--via','ble','stand'])
