"""Temporary shared C joint velocity feed-forward; plant stays unchanged."""
import json,subprocess,tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from servo import SharedGaitPolicy
import cad_physics
from search_raised_level import run as evaluate
ROOT=Path(__file__).resolve().parents[2]

def run(case):
    j2,j3,height,path=case
    class CandidatePolicy(SharedGaitPolicy):
        @classmethod
        def _build_library(cls):return Path(path)
    cad_physics.SharedGaitPolicy=CandidatePolicy
    row=evaluate((.015,1.4,.065,.64,18.),profile='level15');row.update(j2_lead_s=j2,j3_lead_s=j3,height_m=height)
    return row

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(inc/'locomotion.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-joint-timing-'));cases=[]
    old='return locomotion_foot_targets(p,phase,scale,profile==1?1:0,linear,yaw,out);'
    for j2,j3,height in [(.02,0,.018),(.04,0,.018),(.04,0,.020),(.06,0,.015),(.06,0,.018),(.04,.01,.015),(.04,.01,.018),(.02,.01,.018)]:
        code=f'''if(!locomotion_foot_targets(p,phase,scale,profile==1?1:0,linear,yaw,out))return false;
    if(profile==locomotion_profile_id("level15")) {{
        GaitPolicyLegTarget a[4],b[4];
        float period=p[0]*(1.35f-.35f*fminf(1,fabsf(linear)+fabsf(yaw)));
        if(!locomotion_foot_targets(p,phase-.005f+1.f,scale,0,linear,yaw,a) ||
           !locomotion_foot_targets(p,phase+.005f,scale,0,linear,yaw,b))return false;
        for(int i=0;i<4;i++) {{
            out[i].j2_deg+=gait_policy_clampf({j2}f*(b[i].j2_deg-a[i].j2_deg)/(.01f*period),-6.f,6.f);
            out[i].j3_deg+=gait_policy_clampf({j3}f*(b[i].j3_deg-a[i].j3_deg)/(.01f*period),-6.f,6.f);
        }}
    }}
    return true;'''.replace('0f*','0.f*')
        d=folder/f'{j2}-{j3}-{height}';d.mkdir();(d/'locomotion.h').write_text(base.replace(old,code).replace('float p[7];locomotion_params(profile,linear,p);',f'float p[7];locomotion_params(profile,linear,p);if(profile==locomotion_profile_id("level15"))p[3]={height}f;'));lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True)
        cases.append((j2,j3,height,str(lib)))
    (ROOT/'simulation/mujoco/diagnostics/j3-lift/refined_libraries.json').write_text(json.dumps(cases,indent=2)+'\n')
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            (ROOT/'simulation/mujoco/diagnostics/j3-lift/refined_search.json').write_text(json.dumps(rows,indent=2)+'\n')
