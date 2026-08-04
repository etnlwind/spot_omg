# MuJoCo Examples

Spot OMG 시뮬레이션을 구성하면서 사용한 작은 실험 예제입니다. 실제 자세와 보행은
상위 디렉터리의 `preview_pose.py`와 `walk.py`를 사용합니다.

macOS에서는 실시간 Viewer를 위해 저장소 루트에서 `mjpython`으로 실행합니다.

```bash
mjpython simulation/mujoco/examples/falling_ball.py
mjpython simulation/mujoco/examples/joint_sweep.py
mjpython simulation/mujoco/examples/pose_cycle.py
```

- `falling_ball.py`: 중력과 평면 충돌 확인
- `joint_sweep.py`: URDF의 12개 관절을 하나씩 움직여 축 방향 확인
- `pose_cycle.py`: 초기 sit/stand 실험 자세를 보간하여 반복
