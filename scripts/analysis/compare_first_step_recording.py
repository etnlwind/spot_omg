"""Offline replay of measured targets; estimated floor physics, no robot transport."""
import argparse,csv,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    commands=list(csv.DictReader((a.source/'commands.csv').open()));samples=list(csv.DictReader((a.source/'samples.csv').open()))
    keys=[k for k in commands[0] if k.endswith('_deg')]
    ts=np.array([float(r['send_begin_ms'])/1000 for r in commands]);q=np.array([[float(r[k]) for k in keys] for r in commands]);q[:,[0,3]]*=-1
    params=load_parameters(parse_args([]));params['pack_open_circuit_voltage']=11.3
    plant=Simulation(params)
    for _ in range(50):plant.step(targets_deg=q[0],native_servo=True)
    times=[];predicted=[];tilt=[]
    for i in range(int(np.ceil(ts[-1]/.02))+2):
        t=i*.02;times.append(t);predicted.append(np.degrees(plant.data.qpos[plant.q]).copy())
        row=plant.row();tilt.append([row['roll_deg'],row['pitch_deg']])
        plant.step(targets_deg=q[max(0,np.searchsorted(ts,t,side='right')-1)],native_servo=True)
    predicted=np.array(predicted);report=[]
    for j,key in enumerate(keys):
        rows=[r for r in samples if int(r['joint'])==j and int(r['status'])==0]
        t=np.array([float(r['time_ms'])/1000 for r in rows]);actual=np.array([float(r['actual_deg']) for r in rows]);goal=np.array([float(r['target_deg']) for r in rows])
        if j in (0,3):actual*=-1;goal*=-1
        modeled=np.interp(t,times,predicted[:,j]);error=actual-goal
        report.append(dict(joint=key,samples=len(rows),measured_peak_tracking_error_deg=float(np.max(abs(error))),model_peak_tracking_error_deg=float(np.max(abs(modeled-goal))),model_vs_measured_rms_deg=float(np.sqrt(np.mean((modeled-actual)**2)))))
    result=dict(source=str(a.source),target_duration_s=float(ts[-1]),joints=report,model_peak_tilt_deg=float(np.max(abs(np.array(tilt)))),limitations=['Recorded targets replayed at 20ms; not an independent controller run.','Initial floor pose settled to first goal; measured body position/contact not available.','11.3V open-circuit and servo mechanics are estimates.','Sequential real joint reads about 120ms; no exact 10-20ms delay inference.','Model response mismatch is not identification of a single physical cause.'])
    (a.output/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
