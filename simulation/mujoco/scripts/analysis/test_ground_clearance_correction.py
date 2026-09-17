"""Offline software experiment at unchanged gait speed and floor physics.

Uses delayed firmware attitude observations, never simulator truth, for control.
Not a registered gait or deployable hardware implementation.
"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import json
import numpy as np
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding,physical_replay

def correction(gain,cap,stance_gain=0):
    def apply(robot,gait,phase,q):
        if gait.stop_progress is not None or robot.imu_reading is None or robot.imu_reading['age_ms']>100 or robot.attitude_filter.failures:
            return q
        roll,pitch=np.radians(np.asarray(robot.attitude_filter.filtered)/10)
        kin=gait.kin;kin.set_angles(q)
        points=np.array([kin.foot(i) for i in range(4)])
        swing=((phase+np.array([.5,0,0,.5]))%1-.5)*2
        u=np.clip(np.minimum(swing,1-swing)/.25,0,1)
        weight=u**3*(10+u*(-15+6*u))
        # Raise only the threatened swinging feet. Preserve propulsion, J1,
        # timing and stopping. This does not assert body stabilization.
        dz=np.clip(gain*(-roll*points[:,1]+pitch*points[:,0]),0,cap)*weight
        # A planted foot must extend on the low body side to raise that side;
        # the world-clearance sign for an airborne foot is the opposite.
        local=(phase+np.array([.5,0,0,.5]))%1
        stance_u=np.clip(np.minimum(local,.5-local)/.1,0,1)
        stance_weight=stance_u**3*(10+stance_u*(-15+6*stance_u))
        dz+=np.clip(stance_gain*(roll*points[:,1]-pitch*points[:,0]),-.004,.004)*stance_weight
        points[:,2]+=dz
        result,error=kin.solve_xz(points,q,q[::3])
        if error>.0002:raise ValueError('Candidate clearance target unreachable')
        return result
    return apply

def main():
    out=ROOT/'artifacts/imu-trace-v77/simulator-audit/software-candidates';out.mkdir(parents=True,exist_ok=True)
    lib=load_binding(out)
    for gain,cap,stance_gain in [(0,0,.15),(0,0,.3),(.25,.004,.15),(.25,.004,.3)]:
        name=f'gain-{gain}-cap-{cap}-stance-{stance_gain}'
        try:
            report,rows=physical_replay(lib,command=(344,0),profile='s_native_v6_2_7',walk_seconds=8,voltage=11.1,target_adjustment=correction(gain,cap,stance_gain))
            (out/(name+'.json')).write_text(json.dumps(dict(summary=report,rows=rows)))
            print(name,json.dumps(report),flush=True)
        except Exception as exc:
            (out/(name+'.json')).write_text(json.dumps(dict(failed=True,error=repr(exc))))
            print(name,'FAILED',repr(exc),flush=True)

if __name__=='__main__':main()
