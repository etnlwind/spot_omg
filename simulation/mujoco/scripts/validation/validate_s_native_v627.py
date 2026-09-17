"""Release evidence from the actual C kernel, independent CAD servo decoding."""
import json
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding,physical_replay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('artifacts/imu-trace-v77/simulator-audit/requalification'))
    directory=parser.parse_args().output
    directory.mkdir(parents=True,exist_ok=True)
    lib=load_binding(directory)
    cases=[('forward30',(1000,0),30,11.1),('forward8-low',(1000,0),8,10.9),
           ('turn-right',(0,500),8,11.1),('reverse',(-600,0),8,11.1)]
    reports={}
    for name,command,seconds,voltage in cases:
        report,rows=physical_replay(lib,command=command,profile='s_native_v6_2_7',
                                   walk_seconds=seconds,voltage=voltage)
        report.update(walk_seconds=seconds,pack_open_circuit_voltage=voltage)
        reports[name]=report
        (directory/(name+'.json')).write_text(json.dumps(dict(summary=report,rows=rows)),encoding='utf-8')
        (directory/'summary.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
        print(name,json.dumps(report),flush=True)
    passed=all(r['gait_quality']['passed'] and r['no_protection_stop'] and r['stop_pose_pass'] for r in reports.values())
    (directory/'qualification.json').write_text(json.dumps(dict(passed=passed,criteria='No protection stop, S return, and all-leg swing clearance/load checks'),indent=2))
    if not passed:raise SystemExit('V627 gait qualification FAILED; evidence saved. No-fall is not gait quality.')


if __name__=='__main__':main()
