"""Loopback TCP integration; never opens Bluetooth or physical servo hardware."""
import socket
import time
import pytest
from run_calculated_placement import parameters
from cad_physics import Simulation
from virtual_robot import RobotController,ConsoleServer

def test_tcp_policy_runtime_toggle_and_stop():
    robot=RobotController(Simulation(parameters()))
    server=ConsoleServer(robot,'127.0.0.1',0)
    client=socket.create_connection(server.listener.getsockname());client.setblocking(False)
    now=0.
    def exchange(command,expected=None):
        client.sendall((command+'\n').encode());result=b''
        for _ in range(100):
            server.poll(now)
            try:result+=client.recv(8192)
            except BlockingIOError:pass
            if result.endswith(b'# ') and (expected is None or expected.encode() in result):return result.decode()
            time.sleep(.001)
        return result.decode()
    try:
        assert 'attitudepd' in exchange('syncstate')
        assert 'OK profile=attitudepd' in exchange('gaitprofile attitudepd')
        assert 'enabled=0' in exchange('@B 0')
        assert 'balance=off' in exchange('syncstate')
        # The app's existing leveling switch addresses this policy's controller.
        exchange('balance on');assert robot.body_stabilizer.enabled
        robot.command('drive 1000 0 1',now)
        for _ in range(100):
            robot.command(f'@D {2+round(now*1000)} 1000 0',now)
            robot.tick(now);now+=.02
        assert robot.motion is not None
        assert 'enabled=0' in exchange('@B 0')  # accepted while drive owns console
        assert 'enabled=1' in exchange('@B 1')
        assert 'enabled=1' in exchange('@B 2')
        assert 'ERROR:' in exchange('@B 2 junk')
        exchange('@S 100000')
        for _ in range(150):robot.tick(now);now+=.02
        assert robot.motion is None and robot.torque
        assert 'OK profile=centerpivot' in exchange('gaitprofile centerpivot',expected='OK profile=centerpivot')
    finally:
        client.close();server.close()
