"""Render the existing MuJoCo standing model for the app; no robot IO."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools/servo_tool')]
import mujoco
import numpy as np
from PIL import Image
from simulation.mujoco.runtime.cad_physics import Simulation

plant = Simulation()
m, d = plant.model, plant.data
m.vis.global_.offwidth = 900
m.vis.global_.offheight = 1100
camera = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(camera)
camera.azimuth = 0
camera.elevation = -90
camera.orthographic = 1
camera.distance = 0.85
anchors = [m.joint(leg + '_j2').id for leg in ('fl', 'fr', 'rl', 'rr')]
camera.lookat[:] = np.mean(d.xanchor[anchors], axis=0)
option = mujoco.MjvOption()
option.geomgroup[3] = 0
# Recolor only this in-memory illustration; simulator materials and geometry stay unchanged.
for i in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
    if name.startswith("body_") or name.endswith("_j2"):
        m.geom_rgba[i] = [0.95, 0.32, 0.035, 1]
    elif name.endswith(("_j1", "_j3", "_foot")):
        m.geom_rgba[i] = [0.10, 0.12, 0.15, 1]
m.vis.headlight.ambient[:] = 0.25
m.vis.headlight.diffuse[:] = 0.55
m.vis.headlight.specular[:] = 0.15
# Hide only the floor and collision guides.
for i in range(m.ngeom):
    if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
        m.geom_group[i] = 5
option.geomgroup[5] = 0
with mujoco.Renderer(m, height=1100, width=900) as renderer:
    renderer.update_scene(d, camera=camera, scene_option=option)
    rgb = renderer.render().copy()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(d, camera=camera, scene_option=option)
    segmentation = renderer.render()
    alpha = (segmentation[:, :, 0] >= 0).astype(np.uint8) * 255
    rgba = np.dstack((rgb, alpha))
image = Image.fromarray(rgba)
bounds = image.getbbox()
image = image.crop((max(0,bounds[0]-24),max(0,bounds[1]-24),min(900,bounds[2]+24),min(1100,bounds[3]+24)))
output = ROOT / 'apps/ios/SpotOMGController/SpotOMGController/Resources/Assets.xcassets/RobotTopView.imageset'
output.mkdir(parents=True, exist_ok=True)
image.save(output / 'robot-top.png')
(output/'Contents.json').write_text('{"images":[{"filename":"robot-top.png","idiom":"universal"}],"info":{"author":"xcode","version":1}}\n')
print(output/'robot-top.png')
