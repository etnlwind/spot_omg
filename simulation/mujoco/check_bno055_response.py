"""Deterministic sensor step response and delayed-IMU walking regression."""
import json
from pathlib import Path
from bno055_emulator import BNO055Config,BNO055Emulator,FirmwareAttitudeFilter
from check_profile_runtime import run
from search_gait_profiles import physics


def response(delay):
    s=BNO055Emulator(BNO055Config(fusion_delay_s=delay,noise_std_deg=0,startup_s=0))
    f=FirmwareAttitudeFilter(); first=half=ninety=None
    for i in range(1001):
        now=i*.001;s.advance(now,10 if now>=.2 else 0,0)
        if i%20:continue
        r=s.read(now)
        if r is None:continue
        f.update(r)
        if now>=.2:
            if first is None and r['roll_tenths']>0:first=round((now-.2)*1000)
            if half is None and f.filtered[0]>=50:half=round((now-.2)*1000)
            if ninety is None and f.filtered[0]>=90:ninety=round((now-.2)*1000)
    return dict(fusion_delay_ms=delay*1000,first_response_ms=first,filtered_50_ms=half,filtered_90_ms=ninety)


if __name__=='__main__':
    result=dict(step_response=[response(x) for x in (0,.02,.04,.06)],walking=[])
    p,m=physics('nominal')
    for delay in (.02,.06):
        p['bno055']={'fusion_delay_s':delay}
        for profile in ('legacy','crawl','cruise','trot','highstep'):
            row=run(profile,'nominal','turn_forward',p,m)
            row['fusion_delay_ms']=delay*1000
            result['walking'].append(row)
            print(profile,delay,row['safety'],row['stopped'],flush=True)
    path=Path(__file__).with_name('bno055_validation.json')
    path.write_text(json.dumps(result,indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in result['walking'])
