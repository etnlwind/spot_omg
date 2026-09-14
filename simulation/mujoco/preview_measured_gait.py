"""Render real MuJoCo A/B trajectories; no physical robot or phone connection."""
import argparse,json,subprocess,math
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from cad_physics import Simulation,foot_clearance
from virtual_robot import RobotController
from search_measured_gait import OUT

def add_ground_reference(renderer,origin):
    """Visual-only reference geometry; does not touch model contacts or dynamics."""
    scene=renderer.scene
    def line(a,b,color,width=1.):
        if scene.ngeom>=scene.maxgeom:raise RuntimeError('Reference grid exceeds scene capacity')
        geom=scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,width,np.array(a),np.array(b))
        scene.ngeom+=1
    x,y=origin[:2]
    for v in np.arange(-.5,.501,.05):
        line([x+v,y-.5,.002],[x+v,y+.5,.002],[.35,.42,.48,1])
        line([x-.5,y+v,.002],[x+.5,y+v,.002],[.35,.42,.48,1])
    angles=np.linspace(0,2*np.pi,49)
    for a,b in zip(angles[:-1],angles[1:]):
        line([x+.02*np.cos(a),y+.02*np.sin(a),.003],[x+.02*np.cos(b),y+.02*np.sin(b),.003],[1.,.72,.05,1.],2.)
    line([x-.03,y,.003],[x+.03,y,.003],[1.,.72,.05,1.],2.)
    line([x,y-.03,.003],[x,y+.03,.003],[1.,.72,.05,1.],2.)


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--profiles',type=Path,default=OUT/'profiles.json')
    parser.add_argument('--profile',default='measured_lift')
    parser.add_argument('--baseline-profiles',type=Path)
    parser.add_argument('--baseline-profile',default='cruise')
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--fixed-camera',action='store_true',help='Ground-fixed perspective and top view with a 50mm grid')
    parser.add_argument('--directions',nargs='+',choices=['forward','left','right'],default=['forward','left','right'])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    params=json.loads((OUT/'plant.json').read_text());params['timestep_s']=.0005
    font=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',22)
    small=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',18)
    path=args.output/'measured-gait-before-after.mp4'
    process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x920','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    try:
        cases={'forward':('전진',1000,0),'left':('좌회전',0,-1000),'right':('우회전',0,1000)}
        for direction,linear,yaw in [cases[d] for d in args.directions]:
            robots=[];renderers=[];origins=[]
            for new in (False,True):
                plant=Simulation(params);robot=RobotController(plant)
                if new:robot.load_experimental_profiles(args.profiles);robot.select_profile(args.profile)
                else:
                    if args.baseline_profiles:robot.load_experimental_profiles(args.baseline_profiles)
                    robot.select_profile(args.baseline_profile)
                plant.model.vis.global_.offwidth=640;plant.model.vis.global_.offheight=360
                origins.append(plant.data.xipos[plant.model.body('cad_base').id].copy())
                robots.append(robot);renderers.append(mujoco.Renderer(plant.model,height=360,width=640))
            camera=mujoco.MjvCamera();camera.distance=1.15
            option=mujoco.MjvOption();option.geomgroup[3]=0
            try:
                for frame in range(600):
                    now=frame*.02
                    for r in robots:
                        if frame==100:r.command('drive 0 0 1',now)
                        if 100<=frame<500 and frame%10==0:r.command(f'@D {frame} {linear} {yaw}',now)
                        if frame==500:r.command('@S 501',now)
                        r.tick(now);r.drain()
                    if frame%2:continue
                    canvas=Image.new('RGB',(1280,920),'#172332');draw=ImageDraw.Draw(canvas)
                    for i,(r,renderer) in enumerate(zip(robots,renderers)):
                        x=i*640;d=r.plant.data;m=r.plant.model;row=r.plant.row()
                        draw.text((x+16,8),('이전 정책 · '+args.baseline_profile,'개선 후보 · '+args.profile)[i],font=font,fill='white')
                        draw.text((x+16,40),f'{direction} · {now:.2f}초'+(' · Stop' if frame>=500 else ''),font=small,fill='#b8d5e9')
                        rotation=d.xmat[m.body('robot').id].reshape(3,3)
                        heading=np.degrees(math.atan2(rotation[1,0],rotation[0,0]))
                        camera.lookat[:]=origins[i] if args.fixed_camera else d.xipos[m.body('cad_base').id];camera.lookat[2]-=.06
                        for view,azimuth in enumerate((90,180)):
                            camera.azimuth=(45 if view==0 else 90) if args.fixed_camera else heading+azimuth
                            camera.elevation=(-25 if view==0 else -90) if args.fixed_camera else -15
                            renderer.update_scene(d,camera=camera,scene_option=option)
                            if args.fixed_camera:add_ground_reference(renderer,origins[i])
                            canvas.paste(Image.fromarray(renderer.render()),(x,75+360*view))
                        clearance=[1000*foot_clearance(m,d,m.geom(leg+'_foot').id) for leg in ('fl','fr','rl','rr')]
                        draw.text((x+16,800),f'Roll {row["roll_deg"]:+.2f}° / Pitch {row["pitch_deg"]:+.2f}° / {r.safety}',font=small,fill='white')
                        draw.text((x+16,830),'발 여유 FL/FR/RL/RR: '+' / '.join(f'{v:.1f}' for v in clearance)+' mm',font=small,fill='#a8e8ff')
                        draw.text((x+16,860),'실측 명령 비교 모델 · 질량/접촉/지연은 추정',font=small,fill='#ffcd86')
                        center=d.xipos[m.body('cad_base').id]
                        footer=(f'중심 이동 {np.linalg.norm(center[:2]-origins[i][:2])*1000:.1f}mm · 격자 50mm / 기준원 20mm' if args.fixed_camera else '검증 미달 포함 · 실물 재현 보장 아님')
                        draw.text((x+16,887),footer,font=small,fill='#ffcd86')
                    process.stdin.write(np.asarray(canvas).tobytes())
                    if frame==350:canvas.save(args.output/f'comparison-{linear}-{yaw}-7s.png')
            finally:
                for renderer in renderers:renderer.close()
            print('rendered',direction,flush=True)
    finally:
        process.stdin.close();code=process.wait()
        if code:raise RuntimeError('ffmpeg encoding failed')
    print(path,flush=True)
if __name__=='__main__':main()
