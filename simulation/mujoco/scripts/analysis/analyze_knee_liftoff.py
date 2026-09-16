"""Offline liftoff/tuck experiments; never modify saved or installed profiles.

J2/J3 offsets act only in swing. Positive CAD J3 folds the knee; it is not
the geometric inner knee angle. Keep stance push endpoints unchanged and
project paired offsets to shared X/Z while retaining the FR-first J1 rule.
"""
import argparse
from contextlib import contextmanager
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import numpy as np
from simulation.mujoco.scripts.analysis import analyze_s_native_hesitation as observation
from simulation.mujoco.runtime.s_native_gait import SNativeGait


@contextmanager
def tuck_experiment(rise=.3, fall=.3, height=.012,
                    tuck_j2=0.,tuck_j3=0.,tuck_peak=.2,tuck_end=1.,tuck_release=None,
                    j2_timing=None,j3_timing=None,period_s=None,tuck_projection='xz',recovery_frontload=False):
    """Temporarily apply one offline candidate to analysis or video capture."""
    if tuck_release is None:
        tuck_release=tuck_peak
    if not 0<tuck_peak<=tuck_release<tuck_end<=1:
        raise ValueError('Tuck requires 0 < peak <= release < end <= 1')
    for timing in (j2_timing,j3_timing):
        if timing is not None and not 0<=timing[0]<timing[1]<timing[2]<=1:
            raise ValueError('Joint timing requires 0 <= start < peak < end <= 1')
    if period_s is not None and not 1 <= period_s <= 3:
        raise ValueError('Offline recovery period must be 1..3 seconds')
    if tuck_projection not in ('xz','vertical'):
        raise ValueError('Tuck projection must be xz or vertical')
    from simulation.mujoco.runtime.virtual_robot import RobotController
    original_select=RobotController.select_profile
    def select_profile(robot,name):
        result=original_select(robot,name)
        if name=='s_native_v6_2_6' and period_s is not None:
            # Each controller owns its copied profiles; never change PROFILES.
            robot.profiles[name]['params']=list(robot.profiles[name]['params'])
            robot.profiles[name]['params'][0]=period_s
        return result
    original=observation.ORIGINAL_POINTS
    original_method=SNativeGait.points
    original_targets=SNativeGait.fr_entry_targets
    def early_lift(gait,phase,amplitude,linear,yaw):
        points=original(gait,phase,amplitude,linear,yaw)
        if not gait.profile.get('uniform_recovery'):
            return points
        q=(phase+np.array([.5,0.,0.,.5]))%1
        u=np.clip((q-.5)*2,0.,1.)
        def smooth(x):
            x=np.clip(x,0.,1.)
            return x**3*(10+x*(-15+6*x))
        shape=np.minimum(smooth(u/rise),smooth((1-u)/fall))
        if recovery_frontload:
            # C2 endpoints, continuous recovery, more travel during the initial
            # knee fold: beta(3,5) CDF replaces quintic only in swing.
            progress=1-(1-u)**5*(1+5*u+15*u*u)
            old=smooth(u)
            points[:,0]+=amplitude*linear*.145*(progress-old)*(q>=.5)
        activity=min(1.,(abs(linear)+abs(yaw))/.15)
        points[:,2]=gait.origin[:,2]+amplitude*height*activity*shape
        return points
    def tuck_targets(gait,phase,amplitude,linear,yaw):
        result=original_targets(gait,phase,amplitude,linear,yaw)
        if (not gait.profile.get('uniform_recovery') or gait.stop_progress is not None
            or amplitude<=1e-8 or not (tuck_j2 or tuck_j3)):
            return result
        q=(phase+np.array([.5,0.,0.,.5]))%1
        u=np.clip((q-.5)*2,0.,1.)
        def smooth(x):
            x=np.clip(x,0,1);return x**3*(10+x*(-15+6*x))
        shape=np.minimum(smooth(u/tuck_peak),smooth((tuck_end-u)/(tuck_end-tuck_release)))
        activity=min(1.,(abs(linear)+abs(yaw))/.15)
        def pulse(timing):
            if timing is None:return activity*shape
            start,peak,end=timing
            return activity*np.minimum(smooth((u-start)/(peak-start)),smooth((end-u)/(end-peak)))
        gait.kin.set_angles(result)
        feet=np.array([gait.kin.foot(i) for i in range(4)])
        folded=result.copy();folded[1::3]+=tuck_j2*pulse(j2_timing);folded[2::3]+=tuck_j3*pulse(j3_timing)
        gait.kin.set_angles(folded)
        delta=np.array([gait.kin.foot(i) for i in range(4)])-feet
        # Preserve shared diagonal X/Z trajectories despite different first-step J1.
        for pair in ([0,3],[1,2]):delta[pair]=np.mean(delta[pair],axis=0)
        if tuck_projection=='vertical':
            # Independent joint pulses can reverse X during recovery. Use their
            # lift envelope only; IK coordinates J2/J3 along the existing X path.
            # Stance/entry/STOP remain unchanged, and no dwell is inserted.
            delta[:,:2]=0.
        goal=feet+delta
        result,error=gait.kin.solve_xz(goal,folded,result[::3],iterations=60)
        if error>.0002:raise ValueError('Coupled tuck target unreachable')
        gait.previous=result.copy()
        gait.last_target_points=np.array([gait.kin.foot(i) for i in range(4)])
        return result
    observation.ORIGINAL_POINTS=early_lift
    SNativeGait.points=early_lift
    SNativeGait.fr_entry_targets=tuck_targets
    RobotController.select_profile=select_profile
    try:
        yield
    finally:
        observation.ORIGINAL_POINTS=original
        SNativeGait.points=original_method
        SNativeGait.fr_entry_targets=original_targets
        RobotController.select_profile=original_select


def run_case(rise=.3, fall=.3, height=.012, seconds=8, floor=True,
             tuck_j2=0.,tuck_j3=0.,tuck_peak=.2,tuck_end=1.,tuck_release=None,
             j2_timing=None,j3_timing=None):
    with tuck_experiment(rise,fall,height,tuck_j2,tuck_j3,tuck_peak,tuck_end,
                         tuck_release,j2_timing,j3_timing):
        summary,rows=observation.trial(profile='s_native_v6_2_6',floor=floor,
            seconds=seconds,voltage=11.1)
    if tuck_release is None:tuck_release=tuck_peak
    summary.update(rise_fraction=rise,fall_fraction=fall,lift_m=height,
        tuck_j2_deg=tuck_j2,tuck_j3_deg=tuck_j3,tuck_peak=tuck_peak,tuck_end=tuck_end,
        tuck_release=tuck_release,requested_walk_seconds=seconds,
        j2_timing=j2_timing,j3_timing=j3_timing,
        max_walk_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg']))
            for r in rows if 2<=r['time_s']<2+seconds),
        max_stop_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg']))
            for r in rows if r['time_s']>=2+seconds))
    chosen=[r for r in rows if 4<=r['time_s']<2+seconds and r['safety']=='ok' and r['motion'] and not r['stopping']]
    if len(chosen)>1:
        q=(np.array([r['phase'] for r in chosen])[:,None]+[.5,0.,0.,.5])%1
        recovery=(q[1:]>.5)&(q[1:]<1)
        moving=np.diff(np.array([r['actual_feet_m'] for r in chosen])[:,:,0],axis=0)/.02>.02
        loaded=np.array([r['force_n'] for r in chosen])[1:]>1
        summary['loaded_forward_recovery_seconds_per_leg']=(recovery&moving&loaded).sum(axis=0).tolist()
        summary['loaded_forward_recovery_seconds_per_leg']=[n*.02 for n in summary['loaded_forward_recovery_seconds_per_leg']]
        summary['recovery_seconds_per_leg']=(recovery.sum(axis=0)*.02).tolist()
    return summary,rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rise',type=float,nargs='+',default=[.3,.2,.15,.1])
    p.add_argument('--fall',type=float,default=.3)
    p.add_argument('--height',type=float,default=.012)
    p.add_argument('--seconds',type=float,default=8)
    p.add_argument('--supported',action='store_true')
    p.add_argument('--tuck-j2',type=float,default=0.)
    p.add_argument('--tuck-j3',type=float,default=0.)
    p.add_argument('--tuck-peak',type=float,default=.2)
    p.add_argument('--tuck-release',type=float)
    p.add_argument('--tuck-end',type=float,default=1.)
    p.add_argument('--j2-timing',type=float,nargs=3,metavar=('START','PEAK','END'))
    p.add_argument('--j3-timing',type=float,nargs=3,metavar=('START','PEAK','END'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if not all(0<r<=.5 for r in a.rise) or not 0<a.fall<=.5 or not 0<=a.height<=.04:
        p.error('rise/fall must be 0..0.5 and height 0..0.04m (zero allowed)')
    reports=[]
    for rise in a.rise:
        s,r=run_case(rise,a.fall,a.height,a.seconds,not a.supported,
            a.tuck_j2,a.tuck_j3,a.tuck_peak,a.tuck_end,a.tuck_release,a.j2_timing,a.j3_timing)
        reports.append(s)
        name=f'rise-{rise}-fall-{a.fall}-lift-{a.height}'
        if a.tuck_j2 or a.tuck_j3:
            name+=f'-tuck-{a.tuck_j2}-{a.tuck_j3}-peak-{a.tuck_peak}-release-{s["tuck_release"]}-end-{a.tuck_end}'
        if a.j2_timing:name+='-j2-'+ '-'.join(map(str,a.j2_timing))
        if a.j3_timing:name+='-j3-'+ '-'.join(map(str,a.j3_timing))
        with gzip.open(a.output/(name+'.json.gz'),'wt',encoding='utf-8') as f:
            json.dump(dict(summary=s,rows=r),f)
        print(json.dumps({k:v for k,v in s.items() if 'excursion' not in k}),flush=True)
    (a.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
