"""Evaluate stance propulsion vs swing: world contact force and J2/J3 power.

These ground-truth signals are deliberately outside the live controller.
Positive force is robot-forward; positive joint power is mechanical output.
"""
import csv
import json
from pathlib import Path
import mujoco
import numpy as np
from diagnose_turn_clearance import run

ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'


def main():
    profiles=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    results=[]
    for name in ('cushion_forward','cushion_support_shift'):
        rows=[]
        def record(now,robot):
            if now<5:return
            plant=robot.plant;m=plant.model;d=plant.data
            forward=d.xmat[m.body('robot').id].reshape(3,3)[:,0].copy()
            forward[2]=0;forward/=np.linalg.norm(forward)
            forces={m.geom(leg+'_foot').id:np.zeros(3) for leg in ('fl','fr','rl','rr')}
            floor=m.geom('floor').id
            for i,contact in enumerate(d.contact):
                if floor not in (contact.geom1,contact.geom2):continue
                foot=contact.geom2 if contact.geom1==floor else contact.geom1
                if foot not in forces:continue
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                # MuJoCo contact-frame wrench acts on geom2; reverse for geom1.
                forces[foot]+=(1 if contact.geom2==foot else -1)*(contact.frame.reshape(3,3).T@force[:3])
            phase=(robot.phase-.02/profiles[name]['params'][0])%1
            duty=profiles[name]['params'][1]
            for i,leg in enumerate(('fl','fr','rl','rr')):
                q=(phase+(0,.5,.5,0)[i])%1
                force=forces[m.geom(leg+'_foot').id]
                segment=('early_stance' if q<duty/2 else 'late_stance') if q<duty else 'swing'
                rows.append(dict(time_s=now,leg=leg.upper(),phase=q,segment=segment,
                    forward_force_n=float(force@forward),vertical_force_n=float(force[2]),
                    j2_torque_nm=float(d.ctrl[3*i+1]),j3_torque_nm=float(d.ctrl[3*i+2]),
                    j2_power_w=float(d.ctrl[3*i+1]*d.qvel[plant.v[3*i+1]]),
                    j3_power_w=float(d.ctrl[3*i+2]*d.qvel[plant.v[3*i+2]])))
        run(1000,0,24,profile=name,override=profiles[name],cushion=pad,observer=record)
        with (OUT/f'support-propulsion-{name}.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
        summary={'profile':name,'measurement_seconds':19.,'segments':{}}
        for segment in ('early_stance','late_stance','swing'):
            samples=[r for r in rows if r['segment']==segment]
            summary['segments'][segment]=dict(
                mean_forward_force_n=float(np.mean([r['forward_force_n'] for r in samples])),
                positive_forward_impulse_ns=.02*sum(max(0,r['forward_force_n']) for r in samples),
                negative_forward_impulse_ns=.02*sum(min(0,r['forward_force_n']) for r in samples),
                mean_j2_power_w=float(np.mean([r['j2_power_w'] for r in samples])),
                mean_j3_power_w=float(np.mean([r['j3_power_w'] for r in samples])))
        results.append(summary);print(json.dumps(summary),flush=True)
    (OUT/'support-propulsion.json').write_text(json.dumps(results,indent=2))


if __name__=='__main__':main()
