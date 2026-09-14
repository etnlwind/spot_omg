"""Render the actual A/B MuJoCo trajectories; no physical robot commands."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from simulation.mujoco.runtime.cad_physics import Simulation, build, foot_clearance
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics
from simulation.mujoco.runtime.virtual_robot import RobotController

ROOT=SIM_ROOT
OUT=ROOT.parents[1]/'artifacts/audits/tracking-v42'
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    robots=[];renderers=[]
    for enabled in (False,True):
        p,_=physics();p.update(command_delay_s=.08,tracking_feedback_enabled=enabled)
        p['foot_cushion']=json.loads((ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
        xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
        m.vis.global_.offwidth=640;m.vis.global_.offheight=360
        r=RobotController(Simulation(p,m));r.select_profile('arcturn')
        robots.append(r);renderers.append(mujoco.Renderer(m,height=360,width=640))
    font=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',22)
    small=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',18)
    cam=mujoco.MjvCamera();cam.distance=1.15
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    path=OUT/'tracking-before-after.mp4'
    proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x900','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    try:
        for frame in range(701):
            now=frame*.02
            for r in robots:
                if frame==100:r.command('drive 0 0 1',now)
                if 100<=frame<500 and frame%10==0:r.command(f'@D {frame} 0 -1000',now)
                if frame==500:r.command('@S 501',now)
                r.tick(now);r.drain()
            if frame%2:continue
            canvas=Image.new('RGB',(1280,900),'#172332');draw=ImageDraw.Draw(canvas)
            for i,(r,renderer) in enumerate(zip(robots,renderers)):
                x=640*i;d=r.plant.data;m=r.plant.model;row=r.plant.row()
                draw.text((x+16,8),('기존 보행 · 보정 OFF','실험 보행 · 보정 ON')[i],font=font,fill='white')
                draw.text((x+16,40),f'좌회전 · 명령 지연 80ms · {now:.2f}초'+(' · Stop' if now>=10 else ''),font=small,fill='#b8d5e9')
                cam.lookat[:]=d.xipos[m.body('cad_base').id];cam.lookat[2]-=.06
                for top,(az,el) in enumerate(((120,-18),(90,-89))):
                    cam.azimuth=az;cam.elevation=el
                    renderer.update_scene(d,camera=cam,scene_option=opt)
                    canvas.paste(Image.fromarray(renderer.render()),(x,75+top*360))
                draw.text((x+16,800),f'기울기 {row["roll_deg"]:+.2f}° / {row["pitch_deg"]:+.2f}°   진행률 {r.tracking.rate*100 if i else 100:.0f}%',font=small,fill='white')
                clearance=[1000*foot_clearance(m,d,m.geom(leg+'_foot').id) for leg in ('fl','fr','rl','rr')]
                draw.text((x+16,830),'발 여유 FL/FR/RL/RR: '+' / '.join(f'{v:.1f}' for v in clearance)+' mm',font=small,fill='#a8e8ff')
                draw.text((x+16,861),'추정 물리 · 실제 계산 영상 · 접촉/흔들림 개선 미달',font=small,fill='#ffcd86')
            proc.stdin.write(np.asarray(canvas).tobytes())
            if frame==350:canvas.save(OUT/'tracking-before-after-7s.png')
        print(path,flush=True)
    finally:
        proc.stdin.close();code=proc.wait()
        for renderer in renderers:renderer.close()
        if code:raise RuntimeError('encoding failed')
if __name__=='__main__':main()
