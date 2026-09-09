"""Unseen conditions and long run after freezing the selected gait."""
import json
import numpy as np
import mujoco
from optimize_cad_gait import RESULTS,NAMES,make_scenario,evaluate
from cad_physics import build


def main():
    selected=json.loads((RESULTS/'selected.json').read_text())
    params=np.array([selected['params'][k] for k in NAMES])
    output={}
    for name in ('nominal_60s','fine_step','low_friction','lateral_push','motor_loss'):
        scenario=name if name in ('fine_step','low_friction') else 'nominal'
        p,model=make_scenario(scenario)
        if name=='motor_loss':
            for key in ('motor_3215','motor_3250'):
                p[key]['stall_nm']*=.85;p[key]['speed_rad_s']*=.85
            p['command_delay_s']=.04
            xml,p=build(p,write_scene=False);model=mujoco.MjModel.from_xml_string(xml)
        runs={}
        for label,value in (('baseline',None),('optimized',params)):
            r=evaluate(value,p,model,duration=60 if name=='nominal_60s' else 20,baseline=label=='baseline',push=name=='lateral_push')
            runs[label]=r
            print(name,label,'FALL' if r['fallen'] else 'ok',round(r['speed_m_s'],3),'yaw',round(r['yaw_deg'],1),'load',round(r['above_rated_fraction'],3),flush=True)
        output[name]=dict(runs,physics_parameters=p)
        (RESULTS/'holdouts.json').write_text(json.dumps(dict(selected_trial=selected['trial'],conditions=output),indent=2))

if __name__=='__main__':main()
