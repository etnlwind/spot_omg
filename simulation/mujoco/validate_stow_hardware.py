"""Validate the actual shared STM32 Stow encoder, without hardware transport."""
import json
from pathlib import Path
from stow_policy import LANDING,FOLDED,FOLD_SECONDS,frame,encode

def validate():
    failures=[];minimum=[10**9]*12;maximum=[-10**9]*12;largest_step=0
    for start,folded in [(LANDING,True),(FOLDED,False)]:
        previous=None
        for step in range(round(FOLD_SECONDS/.02)+1):
            try:
                _,ticks=frame(start,folded,step*.02)
            except ValueError as exc:
                failures.append(dict(folded=folded,time_s=step*.02,error=str(exc)));break
            for i,t in enumerate(ticks):minimum[i]=min(minimum[i],t);maximum[i]=max(maximum[i],t)
            if previous:largest_step=max(largest_step,max(abs(a-b) for a,b in zip(ticks,previous)))
            previous=ticks
    return dict(encoder_pass=not failures,encoder_failures=failures,
                encoder='shared stow_control.h, signed-magnitude STS absolute position',
                tick_min=minimum,tick_max=maximum,max_frame_step_ticks=largest_step,
                multi_turn_servo_ids=[2,5],physical_validation='pending',
                required_runtime_checks=['STS3250 position mode 0, resolution 1; verify limits 9/11 zero',
                    'torque-off before mode change; settings readback before torque enable',
                    'temperature/hardware-error/load/tracking checks; stop and resume',
                    'reconstruct continuous origin after power loss; translate ordinary gait targets without a one-turn jump'],
                limitations=['CAD joint axes and assembly tolerances remain provisional',
                    'mesh clearance does not model wires or all same-leg/chassis contacts',
                    'host and MuJoCo tests do not replace on-robot validation'])

if __name__=='__main__':
    result=validate()
    path=Path(__file__).parent/'diagnostics/stow/hardware-preflight.json'
    path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['encoder_pass'] else 2)
