"""Read-only substep actuator diagnostics for existing arc-wave candidates.

No plant/controller parameters or state are changed by the instrumentation.
Simulator truth is recorded for evaluation only.
"""
import csv
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from diagnose_turn_clearance import run


ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'artifacts/audits/arc-control-2026-09-12'
OUT=ROOT/'artifacts/audits/arc-actuator-timing-diagnosis'


class Instrument:
    def __init__(self):
        self.frames=[];self.plant=None

    def attach(self,plant):
        self.plant=plant;self.original=plant.sensor_observer
        self.delayed=plant.desired.copy();self.previous_filtered=plant.filtered.copy()
        self.previous_velocity=plant.target_velocity.copy()
        self.acceleration_count=np.zeros(12);self.velocity_count=np.zeros(12)
        self.torque_count=np.zeros(12);self.count=0
        self.max_filter_error=np.zeros(12);self.max_requested_acc=np.zeros(12)
        plant.sensor_observer=self.observe_substep

    def observe_substep(self,model,data):
        p=self.plant;dt=model.opt.timestep
        # This candidate uses one control-frame command delay, as in its source
        # case. The previous frame's desired command is the consumed queue item.
        desired_speed=(self.delayed-self.previous_filtered)/dt
        wanted=np.clip(desired_speed,-p.speed,p.speed)
        acceleration=(wanted-self.previous_velocity)/dt
        self.velocity_count+=abs(desired_speed)>p.speed+1e-8
        self.acceleration_count+=abs(acceleration)>p.p['target_acceleration_rad_s2']+1e-6
        raw=p.p['servo_kp']*(p.filtered-data.qpos[p.q])-p.p['servo_kd']*data.qvel[p.v]
        self.torque_count+=abs(raw)>p.limits+1e-8
        self.max_filter_error=np.maximum(self.max_filter_error,abs(self.delayed-p.filtered))
        self.max_requested_acc=np.maximum(self.max_requested_acc,abs(acceleration))
        self.count+=1
        self.previous_filtered=p.filtered.copy();self.previous_velocity=p.target_velocity.copy()
        if self.original is not None:self.original(model,data)

    def __call__(self,now,robot):
        p=robot.plant
        if self.plant is None:self.attach(p)
        state=p.row();n=max(1,self.count)
        self.frames.append(dict(time_s=now,phase=robot.phase,roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
            nominal_deg=robot.target.tolist(),command_deg=robot.command_target.tolist(),encoded_deg=np.degrees(p.desired).tolist(),
            delayed_deg=np.degrees(self.delayed).tolist(),filtered_deg=np.degrees(p.filtered).tolist(),
            filter_velocity_deg_s=np.degrees(p.target_velocity).tolist(),actual_deg=state['actual_deg'],
            actual_velocity_deg_s=np.degrees(p.data.qvel[p.v]).tolist(),torque_nm=state['torque_nm'],
            acceleration_clip_fraction=(self.acceleration_count/n).tolist(),velocity_clip_fraction=(self.velocity_count/n).tolist(),
            torque_clip_fraction=(self.torque_count/n).tolist(),max_filter_error_deg=np.degrees(self.max_filter_error).tolist(),
            max_requested_acceleration_deg_s2=np.degrees(self.max_requested_acc).tolist()))
        self.delayed=p.desired.copy();self.acceleration_count[:]=0;self.velocity_count[:]=0;self.torque_count[:]=0
        self.max_filter_error[:]=0;self.max_requested_acc[:]=0;self.count=0


def evaluate(name):
    original=json.loads((SOURCE/(name+'.json')).read_text())
    case=original['case'];instrument=Instrument()
    pad=json.loads(Path(__file__).with_name('foot_cushion_d37p3_l27mm.json').read_text())
    overrides=dict(case['parameters']);overrides['tracking_feedback_enabled']=False
    if overrides.get('command_delay_s',.02)!=.02:raise ValueError('This diagnostic assumes one-frame command delay')
    summary,rows=run(0,case['yaw'],25.02,profile='arcturn',cushion=pad,observer=instrument,
                     stop_at=25,parameter_overrides=overrides)
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/(name+'-frames.json')).write_text(json.dumps(instrument.frames,indent=2)+'\n')
    with (OUT/(name+'-feet.csv')).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    (OUT/(name+'-summary.json')).write_text(json.dumps(summary,indent=2,default=float)+'\n')
    print(name,'frames',len(instrument.frames),'maxroll',max(abs(r['roll_deg']) for r in instrument.frames if r['time_s']>=5),flush=True)


def analyze():
    report={}
    for path in sorted(OUT.glob('*-frames.json')):
        frames=json.loads(path.read_text());t=np.array([f['time_s'] for f in frames]);steady=(t>=5)&(t<25)
        fields=('command_deg','encoded_deg','delayed_deg','filtered_deg','actual_deg','filter_velocity_deg_s',
                'actual_velocity_deg_s','acceleration_clip_fraction','velocity_clip_fraction','torque_clip_fraction')
        values={key:np.array([f[key] for f in frames]) for key in fields}
        result=dict(clip_fraction={},period_difference_rms_deg={},fitted_lag={},tracking_rms_deg={})
        for key in ('acceleration_clip_fraction','velocity_clip_fraction','torque_clip_fraction'):
            result['clip_fraction'][key]=values[key][steady].mean(0).reshape(4,3).tolist()
        for key in ('command_deg','filtered_deg','actual_deg'):
            result['period_difference_rms_deg'][key]={}
            for step in (60,120):
                delta=values[key][step:]-values[key][:-step];window=(t[step:]>=15)&(t[step:]<25)
                result['period_difference_rms_deg'][key][str(step*.02)]=np.sqrt(np.mean(delta[window]**2,axis=0)).reshape(4,3).tolist()
        for response,reference in (('filtered_deg','encoded_deg'),('actual_deg','filtered_deg'),('actual_deg','encoded_deg')):
            lags=[];residual=[]
            for joint in range(12):
                response_values=values[response][steady,joint];scores=[]
                for lag in np.arange(0,.161,.001):
                    ref=np.interp(t[steady]-lag,t,values[reference][:,joint])
                    error=response_values-ref;error-=error.mean();scores.append(float(np.sqrt(np.mean(error**2))))
                index=int(np.argmin(scores));lags.append(index);residual.append(scores[index])
            result['fitted_lag'][response+'_vs_'+reference]=dict(lag_ms=np.array(lags).reshape(4,3).tolist(),
                residual_rms_deg=np.array(residual).reshape(4,3).tolist(),note='DC-offset removed, interpolated 1ms grid; not an identified causal transfer function.')
        for a,b in (('encoded_deg','filtered_deg'),('filtered_deg','actual_deg')):
            result['tracking_rms_deg'][a+'_minus_'+b]=np.sqrt(np.mean((values[a][steady]-values[b][steady])**2,axis=0)).reshape(4,3).tolist()
        torque=np.any(values['torque_clip_fraction']>0,axis=1)
        tilt=np.array([abs(f['roll_deg'])>2 for f in frames])
        result['first_torque_clip_after_5s']=float(t[steady&torque][0]) if np.any(steady&torque) else None
        result['first_roll_above_2deg_after_5s']=float(t[steady&tilt][0]) if np.any(steady&tilt) else None
        report[path.name.removesuffix('-frames.json')]=result
    (OUT/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(evaluate,['wave60_rank3_y-1000','stable3_h0.023_p0_g0.3']))
    analyze()
