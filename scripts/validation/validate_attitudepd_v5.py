"""Compare V4/V5 in the unchanged estimated plant; never connect to hardware."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from simulation.mujoco.scripts.analysis.analyze_attitudepd_direction_speed import run_case

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'artifacts/attitudepd-v5/simulation');p.add_argument('--profiles',nargs='+',default=['attitudepd_v4','attitudepd_v5']);p.add_argument('--linears',nargs='+',type=int,default=[1000,-1000]);args=p.parse_args()
    cases=[]
    for profile in args.profiles:
        for lift in ([0,0,0,0],[30,30,0,0]):
            out=args.output/profile/('front30' if lift[0] else 'zero');out.mkdir(parents=True,exist_ok=True)
            for linear in args.linears:
                summary,physics=run_case(linear,out,8.,profile=profile,foot_lift_mm=lift)
                cases.append(summary)
                (args.output/'summary.json').write_text(json.dumps(dict(physical_robot_test=False,PD='off',heading='off',physics_parameters=physics,cases=cases),indent=2)+'\n')
if __name__=='__main__':main()
