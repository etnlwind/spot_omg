"""Two physical simulator drive-stop cycles; no hardware interfaces."""
from pathlib import Path
import json, sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'simulation/mujoco'))
from cad_physics import Simulation
from virtual_robot import RobotController
from ground_frame_preview import configure

config_path=Path(sys.argv[1])
config=json.loads(config_path.read_text())
p=json.loads((ROOT/'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
p.update(timestep_s=.0005,experimental_stow=True)
p['foot_cushion']=json.loads((ROOT/'simulation/mujoco/foot_cushion_10mm.json').read_text())
robot=RobotController(Simulation(p));robot.select_profile('centerpivot');info=configure(robot,config)
records=[];events=[];fault=None
for tick in range(1400):
    now=tick*.02
    if tick in (500,950):
        robot.command(f'drive 1000 0 {tick}',now)
        events.append(dict(t=now,event='drive-start',phase=robot.phase,custom_phase=robot.ground_evaluated_phase))
    if (500<tick<800 or 950<tick<1250) and tick%10==0:
        robot.command(f'@D {tick} 1000 0',now)
    if tick in (800,1250):robot.command(f'@S {tick}',now)
    robot.tick(now)
    message=robot.drain().decode('utf-8',errors='replace')
    if message:events.append(dict(t=now,message=message,pose=robot.pose,motion=bool(robot.motion),transition=bool(robot.transition)))
    row=robot.plant.row()
    records.append(dict(t=now,roll=row['roll_deg'],pitch=row['pitch_deg'],safety=robot.safety,
        phase=robot.phase,custom_phase=getattr(robot,'ground_evaluated_phase',None),pose=robot.pose,
        motion=bool(robot.motion),transition=bool(robot.transition),command=robot.command_target.tolist()))
    if robot.safety!='ok':fault=dict(t=now,reason=robot.safety);break
completed=[event['t'] for event in events if '$SPOTDRIVE stopped reason=requested' in event.get('message','')]
summary=dict(config_source=str(config_path),fault=fault,stop_completed_times_s=completed,
    both_stops_completed=len(completed)==2,final_pose=robot.pose,final_torque=robot.torque,
    final_command_error_from_aligned_neutral_deg=float(np.max(abs(robot.command_target-np.array(info['neutral_deg']).reshape(12)))),
    restart_phase=[event for event in events if event.get('event')=='drive-start'])
out=Path(__file__).with_name('restart-final.json')
out.write_text(json.dumps(dict(summary=summary,events=events,records=records),indent=2))
print(json.dumps(summary,indent=2),flush=True)
