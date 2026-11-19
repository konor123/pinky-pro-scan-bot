# 시뮬레이터 확인 기록 (2026-11-16)

- 대상: `pinklab-art/pinky_pro`의 `pinky_mujoco` 물리 모델(ROS 없이 `Physics` 클래스만 사용).
- 방법: 2.4 × 1.2 × 0.3 m 상자를 패널로 두고(`panel.yaml`), URDF의 IR 위치 3곳에서 아래 방향 ray로 지지면 유무를 판정했다. Heading은 시뮬레이터의 실제 yaw를 썼다.
- 실행: `python -I probe.py <pinky_pro 경로>`, `python -I sweep.py <pinky_pro 경로> <속도> <지연> <후진> <줄 간격>`. `mujoco==3.6.0`, `trimesh`, `pycollada`, `scipy`, `pyyaml`, `xacro`가 필요하다. `shim/`은 ROS 없이 패키지 경로를 찾기 위한 대체 모듈이다.
- 결과: `results.txt`.
