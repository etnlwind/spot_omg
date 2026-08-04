import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = ROOT / "hardware" / "urdf" / "spot_omg.urdf"

model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
data = mujoco.MjData(model)

print("Model loaded")
print("Bodies:", model.nbody)
print("Joints:", model.njnt)
print("DOF:", model.nv)
print()

# 모델을 밝게 표시
model.vis.headlight.ambient[:] = [0.8, 0.8, 0.8]
model.vis.headlight.diffuse[:] = [0.9, 0.9, 0.9]
model.vis.headlight.specular[:] = [0.2, 0.2, 0.2]

# URDF 색상이 너무 어두운 경우 임시로 밝게 변경
for geom_id in range(model.ngeom):
    rgba = model.geom_rgba[geom_id]

    # 거의 검은색인 부품만 회색으로 변경
    if rgba[0] < 0.15 and rgba[1] < 0.15 and rgba[2] < 0.15:
        model.geom_rgba[geom_id] = [0.35, 0.4, 0.48, 1.0]

print("===== Joint list =====")

joint_names = []

for joint_id in range(model.njnt):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_id,
    )

    qpos_address = model.jnt_qposadr[joint_id]
    joint_range = model.jnt_range[joint_id]

    joint_names.append(name)

    print(
        f"{joint_id:2d}: "
        f"{name:20s} "
        f"qpos={qpos_address:2d} "
        f"range=[{joint_range[0]: .3f}, {joint_range[1]: .3f}]"
    )

print()
print("각 관절을 순서대로 움직입니다.")
print("Viewer를 닫으면 종료됩니다.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    viewer.cam.distance = 1.3
    viewer.cam.lookat[:] = [0.0, 0.0, -0.1]

    joint_index = 0
    joint_start_time = time.time()

    while viewer.is_running():
        now = time.time()
        elapsed = now - joint_start_time

        joint_id = joint_index
        qpos_address = model.jnt_qposadr[joint_id]

        # 모든 관절을 원점으로 초기화
        data.qpos[:] = 0.0

        # 현재 관절만 약 ±20도 움직임
        angle = math.radians(20.0) * math.sin(elapsed * 2.0)
        data.qpos[qpos_address] = angle

        mujoco.mj_forward(model, data)
        viewer.sync()

        # 관절 하나당 4초씩 확인
        if elapsed >= 4.0:
            joint_index = (joint_index + 1) % model.njnt
            joint_start_time = now

            print(
                f"Moving joint {joint_index}: "
                f"{joint_names[joint_index]}"
            )

        time.sleep(0.01)
