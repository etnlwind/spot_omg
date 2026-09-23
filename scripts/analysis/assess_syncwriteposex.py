"""Wire budget and conditional MuJoCo comparison; no hardware communication.

Repeated profile writes are assumed to preserve the servo's internal reference.
The model does not identify what real firmware does on a repeated ACC write.
J3=254 is an explicit counterfactual, not a measured installed capability.
"""
import argparse
import contextlib
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import verify_v93_side_step as audit
from optimize_v10_one_step import assess


def rest_to_rest(angle,acceleration,speed):
    threshold=speed*speed/acceleration
    return 2*math.sqrt(angle/acceleration) if angle<=threshold else angle/speed+speed/acceleration


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    wire={}
    for name,item in [('positions_only',2),('position_speed_acceleration',7)]:
        count=8+12*(1+item)
        wire[name]=dict(bytes=count,wire_ms=count*10/1_000_000*1000,
                        write_only_bus_percent=count*10*50/1_000_000*100)
    ideal=[]
    for acc in (50,254):
        a=acc*100*360/4096
        t=rest_to_rest(20,a,270)
        ideal.append(dict(acc_register=acc,assumed_deg_s2=a,fold_20deg_ms=t*1000,
                          fold_and_unfold_ms=2*t*1000,
                          acceleration_only_travel_100ms_deg=.5*a*.1**2))
    original_load=audit.load_parameters;original_plant=audit.Simulation
    report=dict(wire=wire,ideal_profile=ideal,simulations=[],limitations=[
        'Wire times exclude CPU, adapters, other bus traffic, and motor processing.',
        '100 steps/s^2 per ACC unit is the existing model assumption, not identified acceleration.',
        'Repeated same-value writes preserve the model reference; real firmware behavior remains unmeasured.',
        'J3 ACC254 overrides only the offline model cap; neither firmware nor robot was changed.',
        'All contact, actuator, inertia, friction, and delay estimates remain enabled.'])
    for name,phase,cap,repeat in [
        ('short_baseline',.08,[50,254,50]*4,False),
        ('short_same_values_rewritten',.08,[50,254,50]*4,True),
        ('short_hypothetical_j3_254',.08,[50,254,254]*4,True),
        ('long_baseline',.18,[50,254,50]*4,False),
        ('long_hypothetical_j3_254',.18,[50,254,254]*4,True),
    ]:
        def load(cli):
            p=original_load(cli);p['servo_acceleration_cap_register']=cap
            p['syncwrite_experiment']=dict(name=name,rewrite_same_values=repeat,
                note='No serial packets simulated. Repeat writes preserve reference by assumption.')
            return p
        class Plant(original_plant):
            def step(self,*a,**kw):
                if repeat:self.servo_profile.set(self.servo_profile.speed.copy(),self.servo_profile.acceleration.copy())
                return super().step(*a,**kw)
        audit.load_parameters=load;audit.Simulation=Plant
        options=SimpleNamespace(output=args.output/name,profile='attitudepd_v10',input=-1000,
            seconds=4.,lift=[10]*4,width=[0]*4,no_heading=False,transfer_mm=None,
            v10_config=[.03,.025,phase,.5,.04,.2])
        with contextlib.redirect_stdout(io.StringIO()):result,*_=audit.simulate(options)
        measured=assess(result)
        measured.update(name=name,swing_ms=phase*2500,cap=cap,
                        peak_tracking_error_deg=result['summary']['walk']['tracking_peak_deg'])
        report['simulations'].append(measured)
        print(json.dumps(measured),flush=True)
    def rows(name):
        return json.loads((args.output/name/'trajectory.json').read_text(encoding='utf-8'))['rows']
    a,b=rows('short_baseline'),rows('short_same_values_rewritten')
    report['same_value_model_max_difference']={key:float(np.max(abs(np.array([r[key] for r in a])-np.array([r[key] for r in b]))))
        for key in ('actual_deg','position_mm','load_n')}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='simulations'},indent=2))


if __name__=='__main__':main()
