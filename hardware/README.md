# Spot OMG Hardware

- [`urdf/`](./urdf/README.md): Isaac Sim/Isaac Lab용 12-DOF 로봇 모델
- [`../firmware/stm32-learning/`](../firmware/stm32-learning/README.md):
  STM32F446RE, URT-2, STS3215와 BNO086 연결 및 시험 절차
- [`../tools/servo_tool/`](../tools/servo_tool/README.md): URT-2 USB 직접 연결 시
  서보 ID·보정값·포즈 관리

CAD와 출력용 mesh가 준비되면 시각 형상은 mesh로 교체하되, 물리 collision
(충돌 형상)은 안정적인 primitive geometry (기본 형상)를 우선 유지합니다.

서보 ID, 중심점과 방향의 단일 하드웨어 원본은
`tools/servo_tool/config/joints.json`입니다. 자동 생성된 Markdown은 사람이 직접
수정하지 않으며, STM32에서 사용하는 값은 `robot_config.c`에 명시적으로 이식하고
회귀 시험으로 일치 여부를 확인합니다.
