"""Unconstrained floating-body physics comparison, with measured diagnostics."""
import copy,json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from cad_physics import Simulation,build,foot_clearance
from search_gait_profiles import physics
from virtual_robot import RobotController
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    profile=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift']
    robots=[];records=[]
    for height in (.24,.22):
        for stride in (.06,.10,.14):
            p,_=physics();p['foot_cushion']=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
            xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
            m.vis.global_.offwidth=480;m.vis.global_.offheight=280
            r=RobotController(Simulation(p,m));cfg=copy.deepcopy(profile)
            cfg['params'][2]=stride;cfg['params'][4]=height
            r.profiles['physics_preview']=cfg;r.select_profile('physics_preview')
            robots.append(r);records.append(dict(stride_mm=1000*stride,height_parameter_mm=1000*height,profile=cfg,frames=[]))
    renderers=[mujoco.Renderer(r.plant.model,height=280,width=480) for r in robots]
    camera=mujoco.MjvCamera();camera.distance=.85;camera.elevation=-16;camera.azimuth=70
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    font=ImageFont.truetype('/System/Library/Fonts/Menlo.ttc',16)
    video=OUT/'stride-physics-height-comparison.mp4'
    writer=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1440x840','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE)
    try:
        for frame in range(500):
            now=frame*.02
            for r,record in zip(robots,records):
                if frame==100:r.command('drive 0 0 1',now)
                if frame>=100 and frame%10==0:r.command(f'@D {frame} 1000 0',now)
                r.tick(now);r.drain()
                state=r.plant.row();d=r.plant.data;m=r.plant.model
                com=np.average(d.xipos,axis=0,weights=m.body_mass)
                diagnostic=copy.deepcopy(r.footstep_tracker.diagnostic if r.footstep_tracker else {})
                record['frames'].append(dict(time_s=now,roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
                    com_height_mm=1000*float(com[2]),position_m=state['position_m'],safety=r.safety,
                    contacts=state['contacts'],tracking_error_deg=state['max_tracking_error_deg'],
                    clearance_mm=[1000*foot_clearance(m,d,m.geom(leg+'_foot').id) for leg in ('fl','fr','rl','rr')],
                    tracker=diagnostic))
            if frame%2:continue
            canvas=Image.new('RGB',(1440,840),'#172332');draw=ImageDraw.Draw(canvas)
            draw.text((12,9),'ACTUAL MuJoCo PHYSICS | free body | gravity + contacts + servo dynamics | estimated parameters',font=font,fill='#b6ffbc')
            draw.text((12,33),f'Top: height parameter 240mm | Bottom: 220mm (-20mm) | cycle 4.8s | t={now:.2f}s',font=font,fill='white')
            for i,(r,record,renderer) in enumerate(zip(robots,records,renderers)):
                x=(i%3)*480;y=65+(i//3)*380;f=record['frames'][-1]
                camera.lookat[:]=r.plant.data.xipos[r.plant.model.body('cad_base').id];camera.lookat[2]-=.035
                renderer.update_scene(r.plant.data,camera=camera,scene_option=opt)
                canvas.paste(Image.fromarray(renderer.render()),(x,y+25))
                draw.text((x+10,y),f"Stride {record['stride_mm']:.0f}mm | h={record['height_parameter_mm']:.0f}mm",font=font,fill='white')
                draw.text((x+10,y+308),f"roll {f['roll_deg']:+.1f}  pitch {f['pitch_deg']:+.1f}  {f['safety']}",font=font,fill='#ffda75')
                draw.text((x+10,y+330),f"COM {f['com_height_mm']:.0f}mm | contacts {','.join(c.replace('_foot','').upper() for c in f['contacts'])}",font=font,fill='white')
                draw.text((x+10,y+352),'foot mm: '+' '.join(f'{v:.0f}' for v in f['clearance_mm']),font=font,fill='#9fe4ff')
            writer.stdin.write(np.asarray(canvas).tobytes())
            if frame in (198,230,260,350):canvas.save(OUT/f'stride-physics-{now:.2f}s.png')
            if frame%100==0:print(f'{now:.1f}/10s',flush=True)
    finally:
        writer.stdin.close();status=writer.wait()
        for renderer in renderers:renderer.close()
        if status:raise RuntimeError('ffmpeg failed')
    summaries=[]
    for record in records:
        frames=record['frames'];faults=[f for f in frames if f['safety']!='ok']
        first=faults[0]['time_s'] if faults else None
        before=[f for f in frames if f['time_s']>=2 and (first is None or f['time_s']<first)]
        summaries.append(dict(stride_mm=record['stride_mm'],height_parameter_mm=record['height_parameter_mm'],
            com_at_2s_mm=frames[100]['com_height_mm'],first_fault_s=first,final_safety=frames[-1]['safety'],
            pre_fault_max_ik_error_mm=max((1000*f['tracker'].get('planned_residual_m',0) for f in before),default=None),
            pre_fault_max_tilt_deg=max((max(abs(f['roll_deg']),abs(f['pitch_deg'])) for f in before),default=None)))
    (OUT/'stride-physics-height-trace.json').write_text(json.dumps(records,indent=2))
    (OUT/'stride-physics-height-summary.json').write_text(json.dumps(summaries,indent=2))
    print(json.dumps(summaries,indent=2),flush=True)
    print(video,flush=True)

if __name__=='__main__':main()
