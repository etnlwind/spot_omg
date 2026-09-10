"""Stow design study. Offline physics only; never connects to a robot/bridge.

The extended-range case is a hypothetical mechanism, NOT approved joint limits.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import mujoco
import numpy as np
from cad_gait import CAD
from cad_physics import Simulation, build
from stow_policy import LANDING, FOLDED, FOLD_SECONDS, SETTLE_SECONDS

OUT = Path(__file__).parent / 'diagnostics' / 'stow'
# Logical motor angles, degrees, ordered FL/FR/RL/RR, each J1/J2/J3.
FORWARD = [0, -85, 0] * 4
STOW = [0, 95, 0] * 2 + [0, -85, 0] * 2
STAGES = [
    ('landing', LANDING, 2.),
    ('sit-back / rear-legs-forward', [0, 40, 130]*2 + [0, -85, 0]*2, 3.),
    ('front-legs-forward', FORWARD, 3.),
    ('front-legs-rotate-180', STOW, 5.),
    ('stow-hold', STOW, 2.),
]


def stages_for(name):
    if name == 'stow-cycle':
        return stages_for('simultaneous') + [
            ('all-legs-slow-unfold-to-landing', LANDING, FOLD_SECONDS),
            ('landing-hold', LANDING, 2.)]
    if name == 'simultaneous':
        folded = FOLDED.copy()
        return [('landing', LANDING, 2.),
                ('all-legs-slow-overhead-fold', folded, FOLD_SECONDS),
                ('stow-gravity-settle', folded, SETTLE_SECONDS)]
    if name != 'overhead':
        return STAGES
    # Same final geometric angle as +95, reached above the body via -180.
    folded = FOLDED.copy()
    return STAGES[:3] + [('front-legs-overhead-minus-180', folded, 5.),
                         ('stow-hold', folded, 2.)]


def make_plant(extended=False, overhead=False):
    p=json.loads((CAD/'physics_parameters.json').read_text())
    p['timestep_s']=.0005
    if overhead:
        # Unwrapped full-turn design study; the real single-turn encoder cannot
        # express this path. Keep physical motor/voltage/contact dynamics.
        p['embedded_servo_quantization']=False
        p['experimental_stow']=True
    xml,_=build(p,write_scene=False)
    model=mujoco.MjModel.from_xml_string(xml)
    if extended:
        for leg in ('fl','fr','rl','rr'):
            model.jnt_range[model.joint(leg+'_j2').id]=np.radians([-95,105])
    if overhead:
        for leg in ('fl','fr'):
            model.jnt_range[model.joint(leg+'_j2').id]=np.radians([-275,105])
    return Simulation(p,model=model)


def envelope(plant):
    """Full visual mesh AABB in robot coordinates, independent of body tipping."""
    m,d=plant.model,plant.data
    r=d.xmat[m.body('robot').id].reshape(3,3)
    origin=d.xpos[m.body('robot').id]
    points=[]
    for g in range(m.ngeom):
        if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        mesh=m.geom_dataid[g];start=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
        v=m.mesh_vert[start:start+n]
        world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        points.append((world-origin)@r)
    points=np.concatenate(points)
    return (points.max(0)-points.min(0)).tolist()


def run_case(name):
    if name not in ('simultaneous','stow-cycle'):
        raise ValueError('Historical unrestricted Stow paths are no longer executable')
    plant=make_plant(name!='nominal',overhead=name in ('overhead','simultaneous','stow-cycle'));m,d=plant.model,plant.data
    limits=np.degrees(m.jnt_range[[m.joint(f'{l.lower()}_j{j}').id for l in ('FL','FR','RL','RR') for j in (1,2,3)]])
    previous=np.degrees(plant.desired).copy()
    frames=[];labels=[];stages=[];collisions={};powered_collisions={};max_tilt=0.;max_error=0.
    for label,requested,duration in stages_for(name):
        if label=='all-legs-slow-unfold-to-landing':
            # Re-enable from the gravity-settled measured pose, as the bridge does.
            previous=np.degrees(d.qpos[plant.q]).copy()
            plant.desired=d.qpos[plant.q].copy();plant.filtered=plant.desired.copy()
            plant.target_velocity[:]=0
            from collections import deque
            plant.delay=deque([plant.desired.copy() for _ in range(len(plant.delay))])
        requested=np.array(requested,float)
        target=np.clip(requested,limits[:,0],limits[:,1])
        clamped=np.flatnonzero(abs(target-requested)>1e-6).tolist()
        for n in range(round(duration/.02)):
            t=plant.policy.smootherstep(min(1,(n+1)*.02/duration))
            command=previous+(target-previous)*t
            plant.step(targets_deg=command,balance=False,
                       torque_enabled=label!='stow-gravity-settle')
            r=d.xmat[m.body('robot').id].reshape(3,3)
            tilt=np.degrees(np.arccos(np.clip(r[2,2],-1,1)))
            max_tilt=max(max_tilt,float(tilt))
            max_error=max(max_error,float(np.max(abs(np.degrees(d.qpos[plant.q])-command))))
            for c in d.contact:
                if c.dist>=-.001:continue
                a,b=int(c.geom1),int(c.geom2)
                if m.geom_bodyid[a]==0 or m.geom_bodyid[b]==0:continue
                key=' / '.join(sorted([m.geom(a).name,m.geom(b).name]))
                collisions[key]=max(collisions.get(key,0),float(-c.dist))
                if label!='stow-gravity-settle':
                    powered_collisions[key]=max(powered_collisions.get(key,0),float(-c.dist))
            if n%2==0:
                frames.append(d.qpos.copy());labels.append(label)
        previous=target
        dimensions=envelope(plant)
        stages.append(dict(stage=label,requested_deg=requested.tolist(),commanded_deg=target.tolist(),
                           motor_torque_enabled=label!='stow-gravity-settle',
                           clamped_joint_indices=clamped,actual_deg=np.degrees(d.qpos[plant.q]).tolist(),
                           body_envelope_m=dimensions,volume_m3=float(np.prod(dimensions)),tilt_deg=float(tilt)))
    result=dict(case=name,experimental=True,physical_robot_approved=False,
                stages=stages,peak_tilt_deg=max_tilt,peak_tracking_error_deg=max_error,
                non_floor_penetrations_over_1mm=collisions,
                powered_non_floor_penetrations_over_1mm=powered_collisions,
                limitations=['Estimated masses and collision proxies; wiring/motor-case clearance not validated.',
                             'Extended J2 limits are hypothetical. Nominal case explicitly clips unavailable angles.',
                             'Overhead case uses unwrapped front J2 angles and disables single-turn servo encoding only in this isolated study.',
                             'Joint targets use real torque dynamics, gravity, contacts and existing servo encoding; no pose teleport in experiment.'])
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/f'{name}.json').write_text(json.dumps(result,indent=2))
    np.savez_compressed(OUT/f'{name}.npz',qpos=frames,labels=labels)
    return result


def replay(name,video=False,viewer=False):
    plant=make_plant(name!='nominal',overhead=name in ('overhead','simultaneous','stow-cycle'));m,d=plant.model,plant.data
    recording=np.load(OUT/f'{name}.npz');frames=recording['qpos'];labels=recording['labels']
    camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
    camera.distance=1.;camera.azimuth=90;camera.elevation=-12
    option=mujoco.MjvOption();option.geomgroup[3]=0
    if video:
        m.vis.global_.offwidth=960;m.vis.global_.offheight=540
        renderer=mujoco.Renderer(m,height=540,width=960)
        encoder=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x540','-framerate','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(OUT/f'{name}.mp4')],stdin=subprocess.PIPE)
        try:
            for q in frames:
                d.qpos[:]=q;mujoco.mj_forward(m,d)
                camera.lookat[:]=d.xipos[m.body('cad_base').id]
                renderer.update_scene(d,camera=camera,scene_option=option)
                encoder.stdin.write(renderer.render().tobytes())
        finally:
            encoder.stdin.close();encoder.wait();renderer.close()
        if encoder.returncode:raise RuntimeError('Video encoding failed')
    if viewer:
        import mujoco.viewer as mv
        with mv.launch_passive(m,d) as window:
            window.cam.distance=1.;window.cam.azimuth=90;window.cam.elevation=-12
            window.opt.geomgroup[3]=0
            i=0
            while window.is_running():
                start=time.monotonic()
                d.qpos[:]=frames[i];mujoco.mj_forward(m,d)
                window.cam.lookat[:]=d.xipos[m.body('cad_base').id]
                window.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                  'STOW DESIGN STUDY / recorded physics\nCase\nStage\nStatus',
                                  f'NO HARDWARE COMMANDS\n{name}\n{labels[i]}\nEXPERIMENTAL / NOT VALIDATED'))
                window.sync();i=(i+1)%len(frames)
                time.sleep(max(0,.04-(time.monotonic()-start)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--case',choices=['simultaneous','stow-cycle'],default='stow-cycle')
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--video',action='store_true')
    parser.add_argument('--viewer',action='store_true')
    args=parser.parse_args()
    if args.run: print(json.dumps(run_case(args.case),indent=2))
    if args.video or args.viewer:replay(args.case,args.video,args.viewer)
