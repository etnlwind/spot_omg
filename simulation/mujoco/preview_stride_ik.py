"""Explicitly kinematic level-body preview; not a physics playback."""
import csv,json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from cad_physics import build
from search_gait_profiles import physics
from support_shift import SupportShift
from gait_profiles import foot_targets
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'
p,_=physics();p['foot_cushion']=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
xml,_=build(p,write_scene=False);model=mujoco.MjModel.from_xml_string(xml)
model.vis.global_.offwidth=480;model.vis.global_.offheight=300
renderer=mujoco.Renderer(model,height=300,width=480)
kin=SupportShift(model);camera=mujoco.MjvCamera();camera.distance=1.05;camera.elevation=-10
option=mujoco.MjvOption();option.geomgroup[3]=0
font=ImageFont.truetype('/System/Library/Fonts/Menlo.ttc',18)
selected=json.loads((OUT/'stride-ik-atlas.json').read_text())['selected'];paths=[];roots=[]
for info in selected:
    mm=round(info['stride_mm'])
    with (OUT/f'stride-{mm}mm-joint-trajectory.csv').open() as f:paths.append(np.array([[float(x) for x in r] for r in list(csv.reader(f))[1:]]))
    q=foot_targets([4.8,.7,mm/1000,.04,info['height_parameter_mm']/1000,-.01,.75],0,0).reshape(-1)
    kin.set_angles(q);roots.append(-np.mean([kin.foot(i)[2] for i in range(4)]))
writer=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1440x740','-framerate','12.5','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'stride-ik-preview.mp4')],stdin=subprocess.PIPE)
try:
    for frame in range(len(paths[0])):
        canvas=Image.new('RGB',(1440,740),'#172332');draw=ImageDraw.Draw(canvas)
        draw.text((15,10),'KINEMATIC PREVIEW ONLY | torso held level | no gravity / no contact dynamics',font=font,fill='#ffda75')
        draw.text((15,36),'Calculated J2 + J3 motion | 40mm lift | 4.8s cycle shown at 0.5x speed',font=font,fill='white')
        for col,(info,trajectory,z) in enumerate(zip(selected,paths,roots)):
            kin.set_angles(trajectory[frame,2:]);kin.data.qpos[2]=z
            mujoco.mj_forward(model,kin.data)
            camera.lookat[:]=kin.data.xipos[model.body('cad_base').id];camera.lookat[2]-=.06
            draw.text((col*480+15,65),f"Stride {info['stride_mm']:.0f}mm | t={trajectory[frame,1]:.2f}s",font=font,fill='white')
            for row,azimuth in enumerate((90,0)):
                camera.azimuth=azimuth
                renderer.update_scene(kin.data,camera=camera,scene_option=option)
                canvas.paste(Image.fromarray(renderer.render()),(col*480,95+row*300))
                draw.text((col*480+10,100+row*300),('SIDE','FRONT')[row],font=font,fill='black')
        draw.text((15,710),'Physical tests failed: this preview shows target geometry, not validated walking.',font=font,fill='#ffda75')
        writer.stdin.write(np.asarray(canvas).tobytes())
        if frame==42:canvas.save(OUT/'stride-ik-preview.png')
finally:
    writer.stdin.close();code=writer.wait();renderer.close()
    if code:raise RuntimeError('ffmpeg failed')
print(OUT/'stride-ik-preview.mp4')
