"""App STOP must let MuJoCo finish the physical return to S."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from spot_controller.protocol import Controller
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args


def test_app_stop_waits_for_v6_1_physical_return():
    robot = RobotController(Simulation(load_parameters(parse_args([]))))
    robot.select_profile('s_native_v6_1')
    app = Controller(simulator=True)
    app.default_profile_pending = False
    app.opened(0)
    sent = []
    completion_at = None
    for i in range(700):
        now = i * .02
        if i == 100:
            assert app.can_drive
            app.release(now)
            app.update(0, 1, now)
        if i in (500, 575):
            app.stop(now)  # Repeated STOP during final blend must not interrupt it.
        app.tick(now)
        while app.outbox:
            _, data = app.outbox.popleft()
            sent.append(data)
            for line in data.decode().splitlines():
                robot.command(line, now)
        robot.tick(now)
        reply = robot.drain()
        if b'$SPOTDRIVE stopped' in reply:
            completion_at = now
            assert not robot.motion and not robot.transition
            assert np.max(abs(robot.command_target-robot.stand_target)) < .01
        app.feed(reply, now)
    assert sum(packet.startswith(b'@S ') for packet in sent) == 1, sent
    assert b'\x03' not in sent
    assert completion_at is not None and 10 < completion_at < 14
    assert not app.fatal and app.phase == 'idle'
    assert np.max(abs(np.degrees(robot.plant.data.qpos[robot.plant.q])-robot.stand_target)) < 1.1
