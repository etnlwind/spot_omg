"""Release evidence from the actual C kernel, independent CAD servo decoding."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding,physical_replay


def main():
    directory=Path('artifacts/s-native-v6-2-7/registered-v627/c-replay')
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


if __name__=='__main__':main()
