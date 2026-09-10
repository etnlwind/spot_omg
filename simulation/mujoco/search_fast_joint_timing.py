"""Evaluate faster shared-C J2/J3 gaits against an unchanged plant."""
import json,subprocess,tempfile,shutil,copy
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from servo import SharedGaitPolicy
import cad_physics,search_raised_level
from virtual_robot import RobotController
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'simulation/mujoco/diagnostics/joint-speed-timing'

def run(case):
    params,lead,path=case
    class Policy(SharedGaitPolicy):
        @classmethod
        def _build_library(cls):return Path(path)
    class Robot(RobotController):
        def __init__(self,plant):
            super().__init__(plant)
            self.profiles['joint']['params']=params
            self.deployed_profiles=copy.deepcopy(self.profiles)
    cad_physics.SharedGaitPolicy=Policy;search_raised_level.RobotController=Robot
    row=search_raised_level.run((.015,1.4,.065,.64,18.),profile='joint');row['joint_lead_s']=lead;return row

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';header=(inc/'locomotion_profiles.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-joint-speed-'));cases=[];OUT.mkdir(parents=True,exist_ok=True)
    fmt=lambda a:'{'+','.join(format(v,'.10g')+('f' if any(c in format(v,'.10g') for c in '.eE') else '.0f') for v in a)+'}'
    specs=[(p,s,.015,l) for p,s in [(1.25,.065),(1.3,.075),(1.35,.085),(1.4,.095)] for l in ([0,.04,0],[0,.02,.01])]
    for n,(period,stride,height,lead) in enumerate(specs):
        params=[period,.64,stride,height,.20175,-.035,.75]
        d=folder/str(n);shutil.copytree(inc,d)
        lines=header.splitlines();start=next(i for i,l in enumerate(lines) if l.startswith('static const float locomotion_parameters'))
        # Profile 9 is the preserved joint profile; all other constants stay unchanged.
        lines[start+10]='  {'+fmt(params)+','+fmt(params)+'},'
        (d/'locomotion_profiles.h').write_text(('\n'.join(lines)+'\n').replace('{0.0f,0.04f,0.01f}',fmt(lead)));lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-lm','-o',str(lib)],check=True)
        cases.append((params,lead,str(lib)))
    (OUT/'libraries.json').write_text(json.dumps(cases,indent=2)+'\n')
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True);(OUT/'search.json').write_text(json.dumps(rows,indent=2)+'\n')
