"""Run critical existing firmware C regressions using the repository Windows Zig."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[4]
FIRMWARE=ROOT/'firmware/stm32-learning'
CASES={
    'safety':['safety'],
    'stow_servo_coordinates':['sts3215','robot_config','feetech_protocol'],
    'stow_motion':['stow_motion','command_recovery','robot_config','safety','feetech_protocol'],
    'pose_supervisor':['pose_supervisor','command_recovery','robot_config','safety'],
    'command_recovery':['command_recovery','robot_config','safety'],
    'servo_profile_registers':['sts3215','robot_config','feetech_protocol'],
    'battery_telemetry':[],
    'servo_response_probe':['servo_response_probe','robot_config','safety'],
}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True);reports=[]
    for name,sources in CASES.items():
        output=args.output/(name+'.exe')
        build=subprocess.run([str(ROOT/'.toolchain/zig/zig.exe'),'cc','-target','x86_64-windows-gnu',
            '-std=c11','-O1','-UNDEBUG','-Wall','-Wextra','-Werror','-I'+str(FIRMWARE/'Inc'),
            '-include',str(FIRMWARE/'tests/host_hal.h'),
            *[str(FIRMWARE/'Src'/(s+'.c')) for s in sources],str(FIRMWARE/'tests'/('test_'+name+'.c')),
            '-o',str(output),'-lm'],capture_output=True,text=True)
        (args.output/(name+'-build.log')).write_text(build.stdout+build.stderr,encoding='utf-8')
        if build.returncode:raise RuntimeError(build.stderr)
        run=subprocess.run([str(output.resolve())],capture_output=True,text=True,timeout=30)
        if run.returncode:raise RuntimeError(f'{name} failed {run.returncode}: {run.stdout} {run.stderr}')
        reports.append(dict(test=name,passed=True,output=run.stdout));print(name+': passed',flush=True)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')

if __name__=='__main__':main()
