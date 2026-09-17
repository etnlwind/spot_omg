"""Record an explicitly unregistered floor-balance candidate."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import argparse
import imageio_ffmpeg
import json
from simulation.mujoco.scripts.analysis.capture_s_native_balance import capture
from simulation.mujoco.scripts.analysis.analyze_large_recovery_balance import install_recovery

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--case',help='Required when the config contains more than one experiment')
    parser.add_argument('--walk-seconds',type=int,default=8)
    parser.add_argument('--output',type=Path,required=True)
    cli=parser.parse_args();cases=json.loads(cli.config.read_text(encoding='utf-8'))
    if not cases or (len(cases)>1 and not cli.case):parser.error('Select an explicit --case from the config')
    name=cli.case or next(iter(cases))
    if name not in cases:parser.error(f'Unknown case {name!r}')
    config=cases[name]
    height=f"body {config.get('crouch',0)*1000:g}mm lower" if config.get('crouch') else 'original S height'
    label=f"OFFLINE | {height} | J1 hold | recovery J2 {config.get('j2',85):g} deg"
    args=argparse.Namespace(command=344,walk_seconds=cli.walk_seconds,settle_seconds=4,profile='s_native_v6_2_7',
        allow_fall=False,fourth_view='side',ffmpeg=imageio_ffmpeg.get_ffmpeg_exe(),output=cli.output)
    capture(args,parser,experiment=dict(name=name,config=config,pack_open_circuit_voltage=11.1,
        j1_mode='fixed_j1',label=label),
        gait_setup=lambda robot:install_recovery(robot,config))

if __name__=='__main__':main()
