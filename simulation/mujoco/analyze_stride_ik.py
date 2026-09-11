"""Level-body CAD IK atlas. Kinematic feasibility is NOT dynamic validation."""
import csv
import json
from pathlib import Path
import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cad_physics import build
from search_gait_profiles import physics
from support_shift import SupportShift, OFFSETS, swing_path
from gait_profiles import foot_targets

ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'
LEGS=('FL','FR','RL','RR')


def calculate(kin,stride,height,period=4.8,samples=121):
    params=[period,.7,stride,.04,height,-.01,.75]
    seed=foot_targets(params,0,0).reshape(-1)
    kin.set_angles(seed)
    reference=np.array([kin.foot(i) for i in range(4)])
    reference[:,2]=np.mean(reference[:,2])
    com=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
    com_height=float(com[2]-reference[0,2])
    phases=np.linspace(0,1,samples)
    angles=[];targets=[];errors=[];actual=[]
    for phase in phases:
        points=reference.copy()
        for i,q in enumerate((phase+OFFSETS)%1):
            x,z=swing_path(q,.7)
            points[i,0]+=stride*x;points[i,2]+=.04*z
        seed,error=kin.solve(points,seed,iterations=32,stance=((phase+OFFSETS)%1)<.7)
        angles.append(seed.copy());targets.append(points.copy())
        actual.append(np.array([kin.foot(i) for i in range(4)]));errors.append(error)
    angles=np.array(angles);targets=np.array(targets);actual=np.array(actual)
    velocity=np.gradient(angles,period/(samples-1),axis=0)
    result=dict(stride_mm=1000*stride,height_parameter_mm=1000*height,
        neutral_com_height_mm=1000*com_height,period_s=period,
        nominal_stance_speed_m_s=stride/(period*.7),
        max_ik_error_mm=1000*max(errors),kinematic_pass=bool(max(errors)<.001),
        dynamic_pass=False,body_roll_pitch_target_deg=[0,0],
        joint_ranges_deg={f'{leg}_J{j+1}':[float(angles[:,i*3+j].min()),float(angles[:,i*3+j].max())] for i,leg in enumerate(LEGS) for j in range(3)},
        peak_joint_speed_deg_s=float(abs(velocity).max()))
    return result,phases,angles,targets,actual


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    p,_=physics();p['foot_cushion']=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    xml,_=build(p,write_scene=False);kin=SupportShift(mujoco.MjModel.from_xml_string(xml))
    results=[];selected=[]
    for stride in (.06,.10,.14):
        candidates=[]
        for height in (.22,.23,.24,.25):
            data=calculate(kin,stride,height);results.append(data[0]);candidates.append(data)
        # Prefer a reachable height near the previous 240mm parameter. This is
        # a geometric selection, not a claim of optimum physical walking.
        selected.append(min(candidates,key=lambda d:(not d[0]['kinematic_pass'],abs(d[0]['height_parameter_mm']-240),d[0]['max_ik_error_mm'])))
    (OUT/'stride-ik-atlas.json').write_text(json.dumps(dict(kind='kinematic_only',candidates=results,selected=[d[0] for d in selected]),indent=2))
    fig,axes=plt.subplots(3,3,figsize=(14,10),layout='constrained')
    for row,(info,phases,angles,targets,actual) in enumerate(selected):
        mm=round(info['stride_mm'])
        with (OUT/f'stride-{mm}mm-joint-trajectory.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['phase','time_s']+[f'{leg}_J{j}_deg' for leg in LEGS for j in (1,2,3)])
            writer.writerows([[float(ph),float(ph*4.8),*map(float,q)] for ph,q in zip(phases,angles)])
        for i,leg in enumerate(LEGS):
            for col,j in ((0,1),(1,2)):
                axes[row,col].plot(phases*4.8,angles[:,3*i+j],label=leg,linestyle='--' if i>1 else '-')
            axes[row,2].plot(1000*(targets[:,i,0]-np.mean(targets[:,i,0])),1000*(targets[:,i,2]-targets[:,i,2].min()),label=leg,linestyle='--' if i>1 else '-')
        for col in range(3):
            ax=axes[row,col];ax.grid(alpha=.25);ax.legend(ncol=4,fontsize=8)
            ax.set_title(f'{mm}mm stride | '+('J2','J3','foot path')[col])
            ax.set_xlabel('Time (s)' if col<2 else 'Fore/aft relative to path mean (mm)')
            ax.set_ylabel('Joint angle (deg)' if col<2 else 'Clearance target (mm)')
        axes[row,0].text(.02,.04,f"IK error {info['max_ik_error_mm']:.3f}mm; height parameter {info['height_parameter_mm']:.0f}mm",transform=axes[row,0].transAxes,fontsize=8)
    fig.suptitle('Level-body CAD solutions: J2 and J3 move together\n40mm common swing arch | 4.8s cycle | KINEMATICS ONLY - dynamic search failed',fontsize=14)
    fig.savefig(OUT/'stride-j2-j3-atlas.png',dpi=150)
    print(json.dumps([d[0] for d in selected],indent=2))


if __name__=='__main__':main()
