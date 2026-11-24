# ros — 로봇 쪽 ROS 2 패키지

로봇(Pinky Pro)과 Gazebo에서 도는 ament_python 패키지다. 스캔 PC 쪽 코드는 `scan3d/`에 있다.

| 패키지 | 스캔에서 쓰는 것 | 내용 |
|---|---|---|
| `scan3d_robot` | `launch/robot_scan.launch.xml` | 로봇 에이전트. `agent.py`는 스캔 PC와 통신하는 TCP 서버(제어 7010, 스트림 7011)이고, `camera.py`는 요청이 올 때만 D455를 켜 원자료를 찍고, `node.py`는 ROS 쪽(odom·LiDAR 받기, `command_driver`에 문장 명령 보내기)을 맡는다. 런치는 `pinky_auto`의 robot 런치, `command_driver`(`stop_distance:=0.0`), 에이전트를 함께 띄운다. |
| `pinky_auto` | `launch/robot.launch.xml` | 로봇 구동. 로봇 모델, RPLiDAR C1, 모터 드라이버(속도 명령이 0.5 s 끊기면 정지), 배터리·센서 노드를 띄운다. 이전 과제에서 쓰던 지도 작성(`mapping`), Nav2 주행(`navigation`), 순찰(`patrol`) 런치도 들어 있으나 스캔에는 쓰지 않는다. |
| `scan3d_sim` | `launch/scan_sim.launch.py` | Gazebo 스캔 시험. D455를 옆으로 단 로봇(`urdf/`), 표식 판을 깐 방(`layouts/`), 실물 에이전트와 같은 TCP 접점을 내는 시뮬레이션용 에이전트(`sim_agent.py`, `sim_camera.py`) |

`command_driver`는 [pinky_sim_examples](https://github.com/fhekwn549/pinky_sim_examples)의 것을 그대로 쓴다. `scripts/deploy.sh`가 정해 둔 커밋을 받아 함께 빌드한다.

## 로봇에 배포하고 켜기

스캔 PC의 저장소 폴더에서 실행한다.

```bash
scripts/deploy.sh pinky1                       # ros/(scan3d_sim 제외)를 로봇 ~/pinky_auto_ws/src로 보내고 colcon 빌드. 성공: build ok
scripts/robot.sh -H pinky1 start robot_scan    # 역할(L·H)과 바퀴 값은 config/scan.yaml에서 읽는다
scripts/robot.sh -H pinky1 status
scripts/robot.sh -H pinky1 stop robot_scan
```

- 로봇 공식 워크스페이스 `~/pinky_pro`는 건드리지 않고, 그 위에 `~/pinky_auto_ws`를 오버레이로 빌드한다.
- `robot.sh`는 스캔 스택을 로봇별 `ROS_DOMAIN_ID`(L 11, H 12)와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 띄워 ROS 통신을 로봇 안에 둔다. 로봇 밖으로는 TCP만 오간다.
- LiDAR·모터는 한 프로세스만 쓸 수 있어, 다른 구동 스택이 떠 있으면 `start robot|robot_scan`은 시작하지 않고 그 프로세스를 보여 준다.

## 시뮬레이션

시뮬레이션 PC(Ubuntu 24.04, ROS 2 Jazzy, `~/pinky_pro`)에서 `scripts/sim_scan.sh L|H [--obstacle]`이 `scan3d_sim`과 의존 패키지를 `build/ros`에 오버레이로 빌드하고 한 바퀴를 돈다.

## 테스트

각 패키지의 `test/`는 저장소 루트의 `pytest`에 함께 들어간다(`pyproject.toml`의 `testpaths`). ROS 없이 도는 부분(모터 명령 끊김 정지, 순찰 순서, 에이전트, 시뮬레이션 카메라)을 검사한다.
