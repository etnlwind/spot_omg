"""Interactive baseline/optimized gait comparison with replay in the same window."""
import collections
import json
import math
import threading
import time
import mujoco
import numpy as np
from cad_physics import Simulation
from optimize_cad_gait import RESULTS, make_scenario, shared_targets as targets, smooth, NAMES


def reset_simulation(parameters, model, viewer_data):
    """Reset physics and controller history without replacing the viewer data."""
    fresh=Simulation(parameters,model)
    mujoco.mj_copyData(viewer_data,model,fresh.data)
    fresh.data=viewer_data
    return fresh


def main():
    import mujoco.viewer as mv
    selected=json.loads((RESULTS/'selected.json').read_text())
    params=np.array([selected['params'][k] for k in NAMES])
    p,model=make_scenario('nominal')
    sim=Simulation(p,model);data=sim.data
    events=collections.deque();lock=threading.Lock()
    def keypress(key):
        with lock:events.append(key)
    mode='optimized';paused=False;elapsed=0.;done=False;start=None;neutral=None;distance=0.
    with mv.launch_passive(model,data,key_callback=keypress) as viewer:
        viewer.opt.geomgroup[3]=0
        viewer.cam.distance=1.2;viewer.cam.azimuth=135;viewer.cam.elevation=-20
        while viewer.is_running():
            tick=time.monotonic();restart=False
            with lock:
                while events:
                    key=events.popleft()
                    if key==82:restart=True
                    elif key==49:mode='baseline';restart=True
                    elif key==50:mode='optimized';restart=True
                    elif key in (32,80):paused=not paused
            if restart:
                with viewer.lock():sim=reset_simulation(p,model,data)
                elapsed=0.;done=False;paused=False;start=None;neutral=None;distance=0.
            if not paused and not done:
                with viewer.lock():
                    if elapsed<2.:
                        sim.step(balance=False)
                    elif elapsed<3.:
                        if neutral is None:neutral=sim.desired*180/math.pi
                        if mode=='optimized':
                            target=neutral+(targets(params,0,0)-neutral)*smooth(elapsed-2)
                            sim.step(balance=False,targets_deg=target)
                        else:sim.step(balance=False)
                    else:
                        if start is None:start=data.subtree_com[model.body('robot').id].copy()
                        t=elapsed-3.
                        if mode=='optimized':sim.step(balance=False,targets_deg=targets(params,t,float(smooth(t))))
                        else:sim.step(1,balance=False,startup=t)
                        distance=float(data.subtree_com[model.body('robot').id,0]-start[0])
                        rotation=data.xmat[model.body('robot').id].reshape(3,3)
                        if rotation[2,2]<math.cos(math.radians(40)) or data.subtree_com[model.body('robot').id,2]<.12:done=True
                        if t>=30:done=True
                    elapsed+=.02
            row=sim.row()
            stage='FINISHED - R to replay' if done else ('PAUSED' if paused else ('SETTLING' if elapsed<3 else 'WALKING'))
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                             'Gait\nState\nTime / distance\nSpeed\nRoll / pitch\nReplay\nCompare\nPause\nModel',
                             f'{mode.upper()}\n{stage}\n{max(0,elapsed-3):.1f}s / {distance:.2f}m\n{data.qvel[0]:.3f} m/s\n{row["roll_deg"]:.1f} / {row["pitch_deg"]:.1f} deg\nR\n1 baseline / 2 optimized\nP or Space\n11.1V 3S; estimated physics'))
            viewer.cam.lookat[:]=row['com_m'];viewer.sync()
            time.sleep(max(0,.02-(time.monotonic()-tick)))


if __name__=='__main__':main()
