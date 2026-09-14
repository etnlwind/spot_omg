"""Render recorded physical states, never commanded targets, in four views."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
from pathlib import Path
import argparse
import json
import subprocess
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from matplotlib.font_manager import findfont
from simulation.mujoco.runtime.cad_physics import build


def render(source, output):
    data=json.loads(source.read_text());rows=data['records']
    xml,_=build(data['parameters'],write_scene=False)
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    d.qpos[:]=rows[0]['qpos'];d.qvel[:]=rows[0]['qvel'];mujoco.mj_forward(m,d)
    m.vis.global_.offwidth=960;m.vis.global_.offheight=540
    renderer=mujoco.Renderer(m,height=540,width=960)
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    cameras=[]
    for azimuth,elevation,distance,ortho in [(90,0,1.10,1),(90,-90,1.10,1),(0,0,.90,1),(135,-22,1.15,0)]:
        c=mujoco.MjvCamera();mujoco.mjv_defaultCamera(c)
        c.azimuth=azimuth;c.elevation=elevation;c.distance=distance;c.orthographic=ortho
        cameras.append(c)
    legs=('fl','fr','rl','rr')
    anchors=[m.joint(l+'_j2').id for l in legs]
    def center():return np.mean(d.xanchor[anchors],axis=0)
    initial=center().copy()
    direction=np.mean(d.xanchor[anchors[:2]],axis=0)-np.mean(d.xanchor[anchors[2:]],axis=0)
    direction[2]=0;direction/=np.linalg.norm(direction)
    side=np.array([-direction[1],direction[0],0.]);overlay_z=initial[2]+.25
    def line(scene,a,b,color,width=2.):
        g=scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,width,a,b);scene.ngeom+=1
    def guides(scene,kind):
        if kind==0:
            for i in (0,2):
                a=d.xanchor[anchors[i]].copy();a[1]+=.045;b=a.copy();b[2]=0
                line(scene,a,b,[1.,.2,.15,1.])
        elif kind==1:
            a=initial-10*direction;b=initial+10*direction;a[2]=b[2]=overlay_z
            line(scene,a,b,[0.,.8,1.,1.],3.)
            p=center();p[2]=overlay_z+.01;g=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.007]*3),p,np.eye(3).ravel(),np.array([1.,.05,.6,1.],dtype=np.float32));scene.ngeom+=1
    fontpath=findfont('DejaVu Sans');font=ImageFont.truetype(fontpath,18);large=ImageFont.truetype(fontpath,25)
    output.parent.mkdir(parents=True,exist_ok=True)
    ff=subprocess.Popen(['/opt/homebrew/bin/ffmpeg','-hide_banner','-loglevel','error','-y',
        '-f','rawvideo','-pixel_format','rgb24','-video_size','1920x1240','-framerate','25',
        '-i','-','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    mode=data['summary']['mode'];fault=data['summary']['fault'];latest_event={};frames=0
    try:
        for i,row in enumerate(rows):
            for event in row['placement'].get('events',[]):latest_event=event
            if i%2:continue
            d.qpos[:]=row['qpos'];d.qvel[:]=row['qvel'];d.ctrl[:]=row['ctrl'];d.time=row['physics_time_s']
            mujoco.mj_forward(m,d)
            com=np.array(row['com']);com[2]-=.03
            panels=[]
            for k,cam in enumerate(cameras):
                cam.lookat[:]=com if k!=1 else initial+direction*((center()-initial)@direction)
                renderer.update_scene(d,camera=cam,scene_option=opt);guides(renderer.scene,k)
                panels.append(renderer.render().copy())
            frame=Image.new('RGB',(1920,1240),(19,25,33))
            frame.paste(Image.fromarray(np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])])),(0,0))
            draw=ImageDraw.Draw(frame)
            stopped=fault is not None and row['t']>=fault['t']
            state='IDLE' if row['t']<10 else ('SAFETY STOP' if stopped else 'FORWARD')
            for x,y,label in [(0,0,'SIDE'),(960,0,'TOP'),(0,540,'REAR'),(960,540,'QUARTER')]:
                draw.rectangle((x+6,y+6,x+948,y+64),fill=(12,16,21))
                draw.text((x+16,y+12),f'{label} | {row["t"]:05.2f}s | {mode.upper()} | {state}',font=font,fill='#ff7a70' if stopped else 'white')
                if label=='REAR':
                    note=f'ROLL {row["roll"]:+.2f} deg   PITCH {row["pitch"]:+.2f} deg'
                elif label=='TOP':
                    note=f'Cyan: initial axis | Pink: body center | lateral {(center()-initial)@side*1000:+.1f} mm'
                elif label=='SIDE':
                    note='Actual clearance FL/FR/RL/RR: '+', '.join(f'{v:.1f}' for v in row['clearance'])+' mm'
                else:
                    note='Ground load >1 N: '+', '.join(l.upper() for l,f in zip(legs,row['force']) if f>1.)
                draw.text((x+16,y+37),note,font=font,fill='#c2d2e4')
            draw.text((20,1090),'Spot OMG | MASS 2.754 kg | 11.1 V 3S | J2: STS3250 x4 | J1/J3: STS3215 x8',font=large,fill='white')
            draw.text((20,1125),'CAD + gravity + limited actuators + BNO latency | Cushion: 27 mm cap, 37.3 mm sole | 25 FPS',font=font,fill='#c2d2e4')
            gait=data.get('gait',dict(period_s=1.35,duty=.52,stride_m=.096,lift_m=.032))
            draw.text((20,1152),f'T {gait["period_s"]:.2f} s | duty {gait["duty"]:.2f} | nominal X travel {gait["stride_m"]*1000:.0f} mm | lift {gait["lift_m"]*1000:.0f} mm | heading / legacy balance OFF',font=font,fill='#c2d2e4')
            offsets=np.array(row['placement'].get('offset_body_m',np.zeros((4,3))))[:,0]*1000
            detail='Calculated X offsets FL/FR/RL/RR (mm): '+', '.join(f'{v:+.1f}' for v in offsets)
            if mode in ('inward','inward_prepared'):
                offsets=np.array(row['placement'].get('offset_body_m',np.zeros((4,3))))[:,1]*1000
                j1=np.asarray(row['command']).reshape(4,3)[:,0]
                detail='Inward Y offsets (mm): '+', '.join(f'{v:+.1f}' for v in offsets)
                detail+=' | J1 targets (deg): '+', '.join(f'{v:+.1f}' for v in j1)
                if mode=='inward_prepared':
                    detail='Prepared foot width 166.1 mm (already narrowed at t=0) | J1 targets: '+', '.join(f'{v:+.1f}' for v in j1)+' deg'
            if mode=='acceleration':
                aa=latest_event.get('planned_acceleration_m_s2',[0,0])
                detail+=f' | planned ax/ay: {aa[0]:+.2f}/{aa[1]:+.2f} m/s2'
            if mode=='underbody':
                detail='J2-relative contact targets: touchdown +20 mm | liftoff -76 mm | original foot width | coordinated J2/J3'
            if mode.startswith('body-pd-'):
                dz=row['placement'].get('applied_dz',[0]*4)
                detail='Stance PD | applied dz FL/FR/RL/RR mm: '+', '.join(f'{x*1000:+.3f}' for x in dz)
                detail+=f' | {row["placement"].get("status","idle")} | 50Hz | cap 5mm'
            draw.text((20,1179),detail,font=font,fill='#9fdbfa')
            verdict=f'FAILED at {fault["t"]:.2f}s: {fault["reason"]}' if stopped else 'EXPERIMENT: recorded physics; stability/real-robot equivalence not certified'
            draw.text((20,1207),verdict,font=font,fill='#ff7a70' if stopped else '#ffc974')
            ff.stdin.write(np.asarray(frame).tobytes());frames+=1
            if frames%125==0:print(mode,'frames',frames,flush=True)
        ff.stdin.close();assert ff.wait()==0
    finally:
        renderer.close()
        if ff.poll() is None:ff.terminate()
    assert frames==(len(rows)+1)//2
    print(output,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();render(args.source,args.output)
