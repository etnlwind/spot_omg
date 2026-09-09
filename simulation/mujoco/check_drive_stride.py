"""Compare joystick fore/aft excursion scales; no hardware connection.

Shared C trajectory and estimated STEP dynamics. Balance feedback is OFF;
STM32 IMU filtering, scheduler and safety recovery are not emulated.
"""
import argparse
import json
from pathlib import Path
from optimize_cad_gait import make_scenario, evaluate


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--search',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('/private/tmp/spot-drive-stride'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    scenarios=['nominal'] if args.search else ['nominal','heavy_slippery','light_grippy','com_offset','low_friction','fine_step']
    for scenario in scenarios:
        p,m=make_scenario(scenario)
        for stride in ((1.,1.2,1.4,1.6,1.8,2.) if args.search else (1.,1.6)):
            for period in ((None,1.5,1.2) if args.search else (None,)):
                r=evaluate(None,p,m,duration=16 if args.search else 24,baseline=True,drive_stride=stride,drive_period=period)
                r['scenario']=scenario;results.append(r)
                print(json.dumps({k:r[k] for k in ['scenario','drive_stride','drive_period','speed_m_s','peak_tilt_deg','yaw_deg','slip_m_s','above_rated_fraction','peak_tracking_error_deg','cost_of_transport','fallen']}),flush=True)
    (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__': main()
