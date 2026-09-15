"""Verify protocol turn signs and heading hold using actual MuJoCo body yaw."""
if __package__ in (None, ''):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import argparse
import json
from pathlib import Path
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding, physical_replay


def validate(directory):
    directory.mkdir(parents=True,exist_ok=True)
    kernel=load_binding(directory)
    reports=[]
    for backend,lib in (('python',None),('c',kernel)):
        straight={}
        for command,heading in (((473,0),True),((473,0),False),((1000,0),True),
                                ((1000,0),False),((0,500),False),((0,-500),False)):
            report,rows=physical_replay(lib,command,heading)
            reports.append(report)
            label=f'{backend}-{command[0]}-{command[1]}-heading-{int(heading)}'
            (directory/(label+'.json')).write_text(json.dumps(dict(summary=report,rows=rows),indent=2)+'\n',encoding='utf-8')
            print(json.dumps(report),flush=True)
            assert report['max_tilt_deg']<5,report
            if command[1]:
                # Body yaw is measured independently: CCW/left is positive.
                assert report['walk_yaw_change_deg']*command[1]<0,report
                assert abs(report['walk_yaw_change_deg'])>15,report
            else:
                straight[command[0],heading]=report
                if heading:assert abs(report['walk_yaw_change_deg'])<2,report
        for linear in (473,1000):
            on=abs(straight[linear,True]['walk_yaw_change_deg'])
            off=abs(straight[linear,False]['walk_yaw_change_deg'])
            assert on<off*.5,(backend,linear,on,off)
    (directory/'summary.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')
    return reports


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    validate(parser.parse_args().output)
