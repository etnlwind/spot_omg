import time

import mujoco
import mujoco.viewer


XML = """
<mujoco model="falling_ball">
    <option timestep="0.002" gravity="0 0 -9.81"/>

    <worldbody>
        <light pos="0 0 3" dir="0 0 -1"/>

        <geom
            name="floor"
            type="plane"
            size="3 3 0.1"
            rgba="0.8 0.8 0.8 1"
        />

        <body name="ball" pos="0 0 1">
            <freejoint/>
            <geom
                type="sphere"
                size="0.1"
                mass="1"
                rgba="0.2 0.4 0.9 1"
            />
        </body>
    </worldbody>
</mujoco>
"""


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    print("MuJoCo version:", mujoco.__version__)
    print("Viewer를 닫으면 프로그램이 종료됩니다.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - start_time
            if elapsed > 5.0:
                mujoco.mj_resetData(model, data)
                start_time = time.time()

            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
