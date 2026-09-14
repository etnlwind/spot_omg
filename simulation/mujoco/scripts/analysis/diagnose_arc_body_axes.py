"""Independent free-body response to small CAD foot-height changes.

Runs MuJoCo only, with no hardware/RobotController commands. The C kinematics
header is compiled behind private wrappers, so no production source is changed.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import collections
import csv
import ctypes
import json
import math
from pathlib import Path
import subprocess
import tempfile

import mujoco
import numpy as np

from simulation.mujoco.runtime.bno055_emulator import BNO055Config, BNO055Emulator, FirmwareAttitudeFilter
from simulation.mujoco.runtime.cad_physics import Simulation, build, foot_clearance
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics


ROOT = REPO_ROOT
OUT = ROOT / 'artifacts/audits/arc-body-axis-diagnosis'
LEGS = ('FL', 'FR', 'RL', 'RR')


class PrivateKinematics:
    def __init__(self):
        self.temp = tempfile.TemporaryDirectory(prefix='arc-body-axes-')
        source = Path(self.temp.name) / 'axes.c'
        source.write_text('''#include "arc_turn.h"
int axes_neutral(float out[12]) {
 GaitPolicyLegTarget q[4];
 if(!arc_turn_targets_configured(0,1,0,0,.02,.5,0,0,q))return 0;
 for(int i=0;i<4;i++){out[3*i]=q[i].j1_deg;out[3*i+1]=q[i].j2_deg;out[3*i+2]=q[i].j3_deg;}return 1;
}
int axes_offset(const float source[12],const float dz[4],float out[12]) {
 for(int i=0;i<4;i++){
  float q[3]={source[3*i],source[3*i+1],source[3*i+2]},p[3];arc_foot(i,q,p,0);p[2]+=dz[i];
  if(!arc_ik(i,p,q))return 0;for(int j=0;j<3;j++)out[3*i+j]=q[j];
 }return 1;
}
void axes_feet(const float source[12],float out[12]) {
 for(int i=0;i<4;i++)arc_foot(i,source+3*i,out+3*i,0);
}
''')
        library = Path(self.temp.name) / 'axes.dylib'
        subprocess.run(['cc', '-shared', '-fPIC', '-O2', '-ffp-contract=off', '-I',
                        str(ROOT/'firmware/stm32-learning/Inc'), str(source), '-o', str(library)], check=True)
        self.lib = ctypes.CDLL(str(library))
        fp = ctypes.POINTER(ctypes.c_float)
        self.lib.axes_neutral.argtypes = (fp,)
        self.lib.axes_offset.argtypes = (fp, fp, fp)
        self.lib.axes_feet.argtypes = (fp, fp)
        neutral = (ctypes.c_float*12)()
        assert self.lib.axes_neutral(neutral)
        self.neutral = np.array(neutral)

    def offset(self, dz):
        out = (ctypes.c_float*12)()
        assert self.lib.axes_offset((ctypes.c_float*12)(*self.neutral), (ctypes.c_float*4)(*dz), out)
        return np.array(out)

    def feet(self, q):
        out = (ctypes.c_float*12)()
        self.lib.axes_feet((ctypes.c_float*12)(*q), out)
        return np.array(out).reshape(4,3)


def make_plant(neutral):
    p, _ = physics()
    p['foot_cushion'] = json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    xml, p = build(p, write_scene=False)
    plant = Simulation(p, mujoco.MjModel.from_xml_string(xml))
    plant.data.qpos[plant.q] = np.radians(neutral)
    mujoco.mj_forward(plant.model, plant.data)
    feet = [plant.model.geom(leg.lower()+'_foot').id for leg in LEGS]
    plant.data.qpos[2] += .001-min(foot_clearance(plant.model,plant.data,g) for g in feet)
    mujoco.mj_forward(plant.model, plant.data)
    plant.desired = plant.filtered = np.radians(neutral).copy()
    plant.target_velocity[:] = 0
    plant.delay = collections.deque([plant.desired.copy() for _ in range(round(p['command_delay_s']/.02))])
    sensor = BNO055Emulator(BNO055Config(noise_std_deg=0, yaw_drift_deg_s=0))
    filt = FirmwareAttitudeFilter()
    def observe(model, data):
        rotation = data.xmat[model.body('robot').id].reshape(3,3)
        sensor.advance(float(data.time), math.degrees(math.atan2(rotation[2,1],rotation[2,2])),
                       math.degrees(math.asin(float(np.clip(-rotation[2,0],-1,1)))),
                       math.degrees(math.atan2(rotation[1,0],rotation[0,0])))
    plant.sensor_observer = observe
    return plant, sensor, filt


def run_case(kinematics, support, pattern, amplitude):
    plant, sensor, filt = make_plant(kinematics.neutral)
    rows = []
    for frame in range(250):
        now = frame*.02
        dz = np.zeros(4)
        if len(support) < 4:
            lift = .02 * min(1., max(0., (now-3.)/.4))
            for leg in range(4):
                if leg not in support:
                    dz[leg] += lift
        # All-four trials settle 3s; diagonal trials settle four-legged first,
        # then lift the unused feet before applying the paired perturbation.
        begin = 3. if len(support) == 4 else 3.6
        ramp = min(1., max(0., (now-begin)/.2))
        for leg in support:
            dz[leg] += amplitude*pattern[leg]*ramp
        target = kinematics.offset(dz)
        plant.step(targets_deg=target, balance=False)
        reading = sensor.read(float(plant.data.time))
        filt.update(reading)
        row = plant.row()
        forces = [0.]*4
        floor = plant.model.geom('floor').id
        foot_ids = [plant.model.geom(leg.lower()+'_foot').id for leg in LEGS]
        for index, contact in enumerate(plant.data.contact):
            if floor in (contact.geom1,contact.geom2):
                other = contact.geom2 if contact.geom1 == floor else contact.geom1
                if other in foot_ids:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(plant.model, plant.data, index, force)
                    forces[foot_ids.index(other)] += max(0.,float(force[0]))
        rows.append(dict(time_s=now, roll_deg=row['roll_deg'], pitch_deg=row['pitch_deg'],
            imu_roll_deg=reading['roll_tenths']/10 if reading else None,
            imu_pitch_deg=reading['pitch_tenths']/10 if reading else None,
            filtered_roll_deg=filt.filtered[0]/10, filtered_pitch_deg=filt.filtered[1]/10,
            forces_n=forces, clearance_mm=[1000*foot_clearance(plant.model,plant.data,g) for g in foot_ids],
            command_dz_mm=(dz*1000).tolist(), actual_deg=row['actual_deg'], target_deg=target.tolist()))
    return rows


def means(rows, start, end):
    window = [r for r in rows if start <= r['time_s'] < end]
    result = {key:float(np.mean([r[key] for r in window])) for key in
              ('roll_deg','pitch_deg','imu_roll_deg','imu_pitch_deg','filtered_roll_deg','filtered_pitch_deg')}
    result['contact_count_mean'] = float(np.mean([sum(force>.2 for force in r['forces_n']) for r in window]))
    result['forces_n'] = np.mean([r['forces_n'] for r in window],axis=0).tolist()
    result['clearance_mm'] = np.mean([r['clearance_mm'] for r in window],axis=0).tolist()
    return result


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    kin = PrivateKinematics()
    result = dict(estimated_physics=True, hardware_commands=False, sensor_noise_std_deg=0.,
                  neutral_deg=kin.neutral.tolist(), neutral_feet_cad_m=kin.feet(kin.neutral).tolist(), cases={})
    for name, support in [('all4',(0,1,2,3)), ('FL_RR',(0,3)), ('FR_RL',(1,2))]:
        for pattern_name, pattern in [('zero',[0,0,0,0]), ('roll',[1,-1,1,-1]), ('pitch',[-1,-1,1,1])]:
            for sign in ([0] if pattern_name=='zero' else [1,-1]):
                key = f'{name}-{pattern_name}-{sign:+d}'
                rows = run_case(kin,support,np.array(pattern),sign*.001)
                (OUT/(key+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
                periods = [('before',2.7,3.),('early',3.24,3.44),('settled',4.5,5.)] if name=='all4' else [
                    ('before',3.46,3.6),('pure_two_contact_window',3.72,3.8),
                    ('early',3.84,4.04),('settled',4.5,5.)]
                item = {period:means(rows,start,end) for period,start,end in periods}
                item['command_pattern_mm'] = (sign*np.array(pattern)).tolist()
                result['cases'][key] = item
                print(key, item, flush=True)
                (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__ == '__main__':
    main()
