"""Reproducible physics and controller-feasibility audit; never controls hardware."""
import argparse,copy,hashlib,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics
from develop_support_v2 import ROOT,OUT

SOURCE=['dynamics_wbc.py','centroidal_preview.py','support_shift.py','virtual_robot.py',
        'cad_physics.py','bno055_emulator.py','foot_cushion_10mm.json','upright_profiles.json']


def trial(job):
    name,scenario,overrides,seconds,stop=job
    cfg=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_dynamics_wbc80'])
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    if name=='cushion':pad.update(contact_time_constant_s=.04,damping_ratio=.8,friction=[.6,.005,.0001])
    rec=Recorder();heights=[]
    def observe(now,robot):
        rec(now,robot)
        heights.append(float(robot.plant.data.xipos[robot.plant.model.body('cad_base').id,2]))
    result,rows=run(1000,0,seconds,profile='cushion_dynamics_wbc80',override=cfg,scenario=scenario,
        cushion=pad,parameter_overrides=overrides,observer=observe,stop_at=stop)
    frames=[f for f in rec.frames if f['time_s']>=5]
    control=[f['support_shift']['dynamics_wbc'] for f in frames if f['moving'] and 'dynamics_wbc' in f['support_shift']]
    qp_failed=[d for d in control if not d.get('feasible',False)]
    preview_failed=[d for d in control if d.get('centroidal_preview',{}).get('status') not in ('solved','solved inaccurate')]
    elapsed=np.array([d['elapsed_ms'] for d in control])
    valid=[d for d in control if d.get('feasible')]
    torque_gap=[np.max(abs(np.array(d['adapter_torque_difference_nm']))) for d in valid]
    z=np.array(heights)[250:];target=next(f['support_shift']['body_height_target_m'] for f in frames if f['support_shift'])
    m=metrics(rec.frames,rows)
    checks=dict(no_safety_stop=not any(f['safety']!='ok' for f in rec.frames),
        tilt=m['max_roll_deg']<=3 and m['max_pitch_deg']<=3,
        rms_tilt=m['rms_roll_deg']<=1.5 and m['rms_pitch_deg']<=1.5,
        height=np.max(abs(z-target))<=.01,
        clearance=all(v['peak_clearance_mm']>=15 for v in result['legs'].values()),
        swing_contact=all(v['middle_swing_contact_fraction']<.1 for v in result['legs'].values()),
        qp_available=not qp_failed,preview_available=not preview_failed,
        servo_projection=not any(d.get('servo_map_clipped',False) for d in valid),
        joint_ranges=m['joint_ranges_ok'],
        front_touchdown=all(result['legs'][leg]['touchdown_forward_median_mm']>=30 for leg in ('FL','FR')))
    if name=='nominal':
        baseline_path=OUT/'diagonal-split-wide80-60s.json'
        baseline=json.loads(baseline_path.read_text())['metrics'] if baseline_path.exists() else None
        checks['tracking_not_worse']=baseline is not None and m['tracking_rms_deg']<=baseline['tracking_rms_deg']
        checks['stance_proxy_not_worse']=baseline is not None and m['stance_center_speed_proxy_m_s']<=baseline['stance_center_speed_proxy_m_s']
    result.update(case=name,passed=all(checks.values()),checks=checks,metrics=m,
        first_fault_s=next((f['time_s'] for f in rec.frames if f['safety']!='ok'),None),
        stop_completed=(not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']) if stop else None,
        height=dict(peak_to_peak_mm=float(np.ptp(z)*1000),rms_target_mm=float(np.sqrt(np.mean((z-target)**2))*1000),
                    mean_mm=float(z.mean()*1000),target_mm=target*1000),
        controller=dict(frames=len(control),qp_failed_frames=len(qp_failed),preview_failed_frames=len(preview_failed),
            elapsed_median_ms=float(np.median(elapsed)),elapsed_p95_ms=float(np.percentile(elapsed,95)),
            elapsed_max_ms=float(elapsed.max()),deadline_misses=int(np.sum(elapsed>20)),
            max_qp_constraint_violation=max((d['qp'].get('constraint_violation',0) for d in valid),default=None),
            max_adapter_torque_difference_nm=max(torque_gap,default=None)),
        hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in SOURCE})
    (OUT/f'dynamics-final-{name}.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    if name=='nominal':
        (OUT/'dynamics-final-nominal-trace.json').write_text(json.dumps(rec.frames,default=lambda x:x.item()))
    print(name,'passed',result['passed'],'fault',result['first_fault_s'],'stop',result['stop_completed'],result['controller'],flush=True)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--conditions',action='store_true');args=parser.parse_args()
    jobs=[('nominal','nominal',{},65.02,None)]
    if args.conditions:
        jobs=[('stop','nominal',{},24.02,18.),('com','com_offset',{},24.02,None),
              ('servo_delay','nominal',{'command_delay_s':.08},24.02,None),
              ('motor_loss','motor_loss',{},24.02,None),('cushion','nominal',{},24.02,None),
              ('imu_delay','nominal',{'bno055':{'fusion_delay_s':.06}},24.02,None)]
    with ProcessPoolExecutor(max_workers=2) as pool:results=list(pool.map(trial,jobs))
    (OUT/('dynamics-final-conditions.json' if args.conditions else 'dynamics-final-summary.json')).write_text(json.dumps(results,indent=2,default=lambda x:x.item()))

if __name__=='__main__':main()
