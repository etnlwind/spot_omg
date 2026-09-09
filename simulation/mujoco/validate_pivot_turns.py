"""Measure achieved yaw, drift and stability, not just accepted drive packets."""
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from search_gait_profiles import physics
from virtual_robot import RobotController
from cad_physics import Simulation


def run(case):
    profile, scenario, mode, direction = case
    p, model = physics(scenario)
    robot = RobotController(Simulation(p, model))
    robot.select_profile(profile)
    headings = []
    peak = 0.
    contact = False
    start = None
    for i in range(1700):
        now = i * .02
        if i == 100:
            start = robot.plant.data.qpos[:2].copy()
            robot.command('drive 0 0 1', now)
        if 100 <= i < 1600 and i % 10 == 0:
            linear, yaw = 0, direction * 1000
            if mode == 'arc': linear, yaw = 700, direction * 700
            elif mode == 'low_arc': linear, yaw = 300, direction * 500
            elif mode == 'reverse_arc': linear, yaw = -300, direction * 500
            elif mode == 'turn_forward' and (i - 100) % 500 >= 250: linear, yaw = 1000, 0
            robot.command(f'@D {i} {linear} {yaw}', now)
        if i == 1600: robot.command('@S 1700', now)
        robot.tick(now)
        rotation = robot.plant.data.xmat[model.body('robot').id].reshape(3, 3)
        headings.append(math.atan2(rotation[1, 0], rotation[0, 0]))
        row = robot.plant.row()
        peak = max(peak, abs(row['roll_deg']), abs(row['pitch_deg']))
        contact |= any(not name.endswith('_foot') for name in row['contacts'])
        robot.drain()
    angle = math.degrees(np.unwrap(headings)[-1] - headings[100])
    drift = float(np.linalg.norm(robot.plant.data.qpos[:2] - start))
    # Positive protocol yaw is clockwise (negative physical heading).
    passed = robot.safety == 'ok' and not contact and robot.motion is None and robot.transition is None
    if mode == 'pivot': passed &= angle * direction < -60 and drift < .30
    return dict(profile=profile, scenario=scenario, mode=mode, direction=direction,
                yaw_deg=angle, displacement_m=drift, peak_tilt_deg=peak,
                safety=robot.safety, nonfoot_contact=contact, passed=bool(passed))


if __name__ == '__main__':
    cases = [(p, 'nominal', mode, d) for p in ('trot', 'cruise', 'crawl', 'highstep')
             for mode in ('pivot', 'arc', 'turn_forward', 'low_arc', 'reverse_arc') for d in (-1, 1)]
    cases += [('trot', scenario, 'pivot', d) for scenario in ('heavy_slippery', 'com_offset') for d in (-1, 1)]
    rows = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run, cases):
            rows.append(row)
            print(json.dumps(row), flush=True)
            Path(__file__).with_name('pivot_turn_validation.json').write_text(json.dumps(rows, indent=2) + '\n')
    assert all(row['passed'] for row in rows)
