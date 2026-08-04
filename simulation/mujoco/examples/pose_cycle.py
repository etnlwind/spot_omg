import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = ROOT / "hardware" / "urdf" / "spot_omg.urdf"

model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
data = mujoco.MjData(model)

# 화면을 밝게 설정
model.vis.headlight.ambient[:] = [0.8, 0.8, 0.8]
model.vis.headlight.diffuse[:] = [0.9, 0.9, 0.9]
model.vis.headlight.specular[:] = [0.2, 0.2, 0.2]

for geom_id in range(model.ngeom):
    rgba = model.geom_rgba[geom_id]

    if rgba[0] < 0.15 and rgba[1] < 0.15 and rgba[2] < 0.15:
        model.geom_rgba[geom_id] = [0.35, 0.4, 0.48, 1.0]


def deg(value):
    return math.radians(value)


def make_pose(j1, front_j2, rear_j2, j3):
    return np.array([
        # FL
        j1, front_j2, j3,

        # FR
        j1, front_j2, j3,

        # RL
        j1, rear_j2, j3,

        # RR
        j1, rear_j2, j3,
    ], dtype=np.float64)

# def make_pose(j1, j2, j3):
#     """
#     네 다리 모두 동일한 J1/J2/J3 값 적용

#     순서:
#     FL J1 J2 J3
#     FR J1 J2 J3
#     RL J1 J2 J3
#     RR J1 J2 J3
#     """
#     return np.array([
#         j1, j2, j3,
#         j1, j2, j3,
#         j1, j2, j3,
#         j1, j2, j3,
#     ], dtype=np.float64)


# 초기 테스트용 자세
# J1: 좌우 벌림
# J2: 어깨 앞뒤 회전
# J3: 무릎 굽힘

SIT_POSE = make_pose(
    deg(0),
    deg(55),     # 앞다리 J2
    deg(55),    # 뒷다리 J2
    deg(125),    # J3
)

STAND_POSE = make_pose(
    deg(0),
    deg(20),     # 앞다리 J2
    deg(20),    # 뒷다리 J2
    deg(65),     # J3
)


def smoothstep(value):
    """급격한 움직임 방지"""
    return value * value * (3.0 - 2.0 * value)


def interpolate_pose(start_pose, end_pose, progress):
    progress = np.clip(progress, 0.0, 1.0)
    progress = smoothstep(progress)

    return start_pose + (end_pose - start_pose) * progress


print("Bodies:", model.nbody)
print("Joints:", model.njnt)
print("DOF:", model.nv)
print()
print("Sit and stand pose test")
print("Close the viewer to exit.")

# 시작은 앉기 자세
data.qpos[:] = SIT_POSE
mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    viewer.cam.distance = 1.3
    viewer.cam.lookat[:] = [0.0, 0.0, -0.1]

    phase_start = time.time()
    phase_duration = 3.0
    standing = True

    while viewer.is_running():
        elapsed = time.time() - phase_start
        progress = elapsed / phase_duration

        if standing:
            # 앉기 → 서기
            data.qpos[:] = interpolate_pose(
                SIT_POSE,
                STAND_POSE,
                progress,
            )
        else:
            # 서기 → 앉기
            data.qpos[:] = interpolate_pose(
                STAND_POSE,
                SIT_POSE,
                progress,
            )

        mujoco.mj_forward(model, data)
        viewer.sync()

        if progress >= 1.0:
            standing = not standing
            phase_start = time.time()

            if standing:
                print("Moving to stand")
            else:
                print("Moving to sit")

        time.sleep(0.01)
