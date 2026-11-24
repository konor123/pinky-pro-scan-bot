# 3D Scan Bot — 개발 계획서

- 과제: Pinky Pro 2대에 RealSense D455를 서로 다른 높이로 달고, 두 대가 동시에 물체 주위를 돌며 촬영해 물체의 3D 메시를 만든다.
- 시연: 2026-11-28(토)
- 장비: Pinky Pro 2대(Raspberry Pi 5 2 GB, Ubuntu 24.04, ROS 2 Jazzy), D455 2대, 개발 PC, 스캔 PC(Ubuntu 노트북), 시뮬레이션 PC(Ubuntu 24.04 + ROS 2 Jazzy + Gazebo)
- 할 일 목록(온라인·현장): `docs/plan/TODOs.md`
- 작성: 2026-11-19

## 1. 목표

시연 장면: 표식 판 가운데에 물체를 놓고 스캔 PC에서 명령 하나를 실행하면, 두 로봇이 서로 반대편에서 함께 물체를 한 바퀴(24시점) 돌며 찍고, 스캔 PC 화면에 색이 있는 3D 메시가 뜬다.

완료 기준:

- 명령 하나로 두 대 동시 한 바퀴부터 메시 표시까지 15분 안에 끝난다.
- 기준 상자의 치수 오차가 ±1 cm 안이다.
- 메시에 겹친 벽(같은 면이 두 겹)이 눈에 띄지 않는다.
- 모든 시점에서 카메라와 물체 사이 거리가 0.40 m 이상이다.
- 원 경로에 작은 상자를 놓아도 멈춘 뒤 비켜 가서 한 바퀴를 마친다.

## 2. 개발 방식

- **로봇은 수집과 전달만, 처리는 전부 스캔 PC가 한다.** 로봇은 센서 값을 가공하지 않고 받은 그대로 보낸다. 표식 검출, 카메라 자세, 거리 확인, 장애물 판단, 이동 계산, 깊이 정렬, 중앙값, 재구성은 스캔 PC에서 한다. 로봇에 남는 계산은 모터 구동과, `command_driver`가 odom으로 확인하며 `turn`/`forward` 문장을 실행하는 저수준 제어뿐이다.
- **영상은 스트림 없이 스냅샷만 받는다.** 스캔 PC가 요청할 때만 로봇이 카메라를 켜서 몇 장 찍고 끈다. ROS 영상 토픽은 쓰지 않는다. odom과 LiDAR는 크기가 작아(로봇당 초당 수십 KB) 계속 보낸다.
- **온라인에서 다 만들고 현장에서는 한 번에 맞춘다.** 현장 작업은 장착, 확인 스크립트 실행, 녹화, 시연으로 한정한다.
- **SDD(명세 먼저)**: 코드를 쓰기 전에 모듈마다 1쪽짜리 명세(`docs/specs/`: 입력·출력·동작·완료 테스트)를 쓴다. 모듈 사이 접점(4장)은 이 문서에서 확정한다.
- **TDD(테스트 먼저)**: 명세의 완료 테스트를 먼저 쓰고 통과시킨다. 로봇 쪽은 가짜 에이전트로 바꿔 끼워 개발 PC에서 전체 흐름을 통과시킨 뒤 Gazebo, 실물 순서로 진행한다.
- **제조사·공개 예제를 출발점으로 쓴다**(아래 표). 새로 짜는 것은 로봇 에이전트, 표식 자세, 주행 판단, 두 대 엇갈림 진행, 데이터 저장, 재구성 묶음뿐이다.
- **성공 경로 우선**: 정상 흐름과 3.3의 주행 규칙만 구현한다. 그 밖의 오류가 나면 두 로봇을 멈추고 메시지를 출력한 뒤 처음부터 다시 실행한다. 현장에서 실제로 난 문제만 고친다.
- **현장에서 바꾸는 값은 `config/scan.yaml` 하나에 모은다.** 현장에서 코드를 고치지 않는다.
- **받은 데이터는 그대로 남긴다.** 스캔 PC는 로봇에서 받은 원본을 그대로 저장하므로, 현장 녹화를 개발 PC에서 처음부터 다시 처리해 재생 테스트로 쓸 수 있다.

| 출처 | 예제 | 쓰는 곳 |
|---|---|---|
| pinklab-art/pinky_pro | `pinky_bringup/bringup.py`(`cmd_vel` → 모터, `odom` 30 Hz) | 로봇 구동. `ros/pinky_auto`의 robot 런치가 이것에 명령 끊김 정지를 더한 것이다. |
| fhekwn549/pinky_sim_examples | `command_driver.py`(`turn <deg>`, `forward <m>`, `stop` 문장 명령을 odom으로 확인하며 실행, 실물에 그대로 사용) | 로봇의 문장 명령 실행. 장애물 판단은 스캔 PC가 하므로 `stop_distance:=0.0` |
| 〃 | `commands.py`(ROS 없는 함수: `sector_min`, `wrap_angle`, `yaw_from_quaternion`) | 스캔 PC 장애물 판단. `sector_min`이 LiDAR 장착 yaw 180°를 보정한다. |
| 〃 | `launch/twin_arena.launch.py`, `urdf/pinky_sim.urdf.xacro`(`d455:=true`), `config/d455_bridge.yaml`, `layouts/scan_room.yaml`, `world_builder`의 `texture`, `spawn_object` | Gazebo 스캔 시험: 표식 판 텍스처를 깐 방, 옆 방향·높이를 바꾼 D455, 실행 중 장애물 추가 |
| realsenseai/librealsense | `wrappers/python/examples/align-depth2color.py` | 로봇 스냅샷: 파이프라인 설정, 프레임 받기, 깊이 배율 |
| 〃 | `wrappers/python/examples/align-with-software-device.py` | 스캔 PC에서 받은 원본 프레임으로 깊이를 컬러에 정렬 |
| 〃 | `box_dimensioner_multicam/`, `depth_auto_calibration_example.py` | 여러 카메라를 판 하나로 맞추는 방식, 깊이 자체 보정 |
| 〃 | apt `ros-jazzy-librealsense2`의 `rs-enumerate-devices`, `rs-save-to-disk` | 카메라 확인, pyrealsense2가 로봇에서 안 될 때의 스냅샷 대체 경로 |
| isl-org/Open3D v0.20.0(사본 `docs/reference/open3d-v0.20.0/`) | `examples/python/t_reconstruction_system/integrate.py`, `reconstruction_system/sensors/realsense_recorder.py` | `integrate.py`의 TSDF 융합(`VoxelBlockGrid`, `compute_unique_block_coordinates` → `integrate`, 메시 추출)을 궤적 대신 표식 자세로 돌린다. 데이터 폴더는 `realsense_recorder.py` 형식(`color/`, `depth/`, `camera_intrinsic.json`). 바꾸는 점은 `docs/specs/recon.md`, `dataset.md` |
| OpenCV | `cv2.aruco`(`DICT_APRILTAG_36h11`, `Board`, `ArucoDetector`), `solvePnP` | 스캔 PC에서 표식 판 생성·검출·카메라 자세 |

## 3. 구성

### 3.1 배치(기본값, `config/scan.yaml`에서 바꾼다)

| 항목 | 값 |
|---|---|
| 물체 | 바닥 윤곽이 지름 0.28 m 원 안(정사각 상자는 대각선 28 cm 이하, 한 변 약 19.8 cm), 높이 0.20 m 이하. 무광, 단단함, 돌려 보면 모양이 다름 |
| 표식 판 | A1 무광 출력을 594 mm 정사각으로 잘라 평평한 바닥에 펴 놓고 가장자리만 테이프로 고정한다. AprilTag 36h11 한 변 85 mm, 115 mm 간격 5×5 격자의 바깥 둘레 16장(가운데 ±187.5 mm 비움). 원점에 0° +x·+y 화살표, 물체 자리 원(반지름 140 mm). 배치는 `docs/specs/board.md` |
| 카메라–물체 거리 `d` | 목표 0.45 m, 유지 범위 0.40 ~ 0.55 m |
| 카메라 원 반지름 `R` | 0.62 m(물체 중심에서 카메라까지) |
| 시점 | 한 바퀴 24시점(15° 간격). 두 로봇은 180° 떨어져 같은 방향(반시계)으로 돈다. |
| 낮은 로봇 L | 카메라 높이 약 0.15 m, 아래로 15° |
| 높은 로봇 H | 카메라 높이 약 0.45 m(본체 약 0.15 m + 알루미늄 프로파일 약 0.30 m), 아래로 37° |
| 카메라 장착 | 두 로봇 모두 차체 뒤쪽에 세운 기둥 하나(1/4인치 나사 끝) 위, 가로 방향. 진행 방향의 왼쪽 직각을 본다. 반시계로 돌면 물체 중심을 본다. |
| 스냅샷 | 컬러 1280×720 YUY2(센서 원래 형식) 1장 + 깊이 848×480 Z16 3장, 15 fps 프로파일 |
| 바닥 | 빈 곳 약 2.2 m × 2.2 m(우회 차선 포함) |

값의 근거:

- **거리**: 848×480의 최소 깊이 0.35 m가 물리적 한계이고, 데이터시트 측정 범위 0.4 m를 계획 하한으로 쓴다. 목표는 0.45 m다.
- **원 반지름**: 물체는 바닥 윤곽을 감싸는 원(반지름 `r_obj` 0.14 m)에 놓는 오차 0.03 m를 더한 `r_eff`(0.17 m)로 본다. `R = r_eff + 0.45`라서 상자 모서리가 카메라 쪽을 향해도 거리가 0.45 m다.
- **같은 사진에 표식과 물체**: 표식 자세를 스냅샷마다 구하므로 각 카메라의 사진에 바닥 표식과 맡은 물체 부분이 함께 들어와야 한다. 깊이 시야 87° × 58°(가로 장착) 기준이다.
- **카메라 높이**: L(0.15 m, 15°)은 가까운 면의 높이 0 ~ 0.27 m와 가까운 표식을 함께 본다. H(0.45 m, 37°)는 표식, 옆면 전체, 윗면을 함께 보고, 윗면 가운데를 약 22°로 내려다본다(`atan((h_H − H)/R)`). H의 피치 37°는 판 가운데 바닥을 겨누는 각(`atan(h_H/R)` ≈ 36°)에 맞춘 값이고, 가까운 표식(아래로 46 ~ 52°)과 윗면 먼 모서리(아래로 18°)가 세로 시야 58° 안에 함께 든다. 30°로 보려면 `h_H ≥ H + R·tan 30°`(물체 0.20 m면 0.56 m)다. 물체가 바뀌면 이 식들로 `R`과 H 높이를 다시 정한다.
- **표식 위치**: 표식은 판 중심에서 x 또는 y가 ±0.1875 ~ 0.2725 m인 정사각 띠(모서리 표식의 바깥 끝은 중심에서 약 0.385 m)에 있어 물체(반경 0.17 m) 밖이고, 두 카메라 모두 가까운 쪽 표식을 사진 아래쪽에 담는다. 판 모서리(반지름 0.42 m)는 로봇 안쪽 바퀴 경로(약 0.585 m) 안에 들어 로봇이 판을 밟지 않는다.
- **검증**(합성 영상, 24시점, 흐림 σ 1.5 px, 반경 0.17 m × 높이 0.20 m 원기둥이 가림, 표식 자세의 카메라 위치 오차): L(0.15 m, 15°) 6 ~ 8장·최대 0.3 mm, H(0.45 m, 37°) 7 ~ 10장·최대 2.2 mm(중앙값 0.1 mm). H 0.45 m에서 33 ~ 48.6°는 모두 최대 2.2 mm 이하라 피치는 표식 자세가 아니라 화면 구성으로 정한다.

### 3.2 소프트웨어

| 위치 | 구성 | 출처 |
|---|---|---|
| 로봇 | `pinky_auto` robot 런치: 모터, odom, LiDAR | 있음 |
| 로봇 | `command_driver`(`stop_distance:=0.0`): 문장 명령 실행 | pinky_sim_examples |
| 로봇 | `robot_agent`: TCP 서버. 제어 연결(`hello`, `move`, `snapshot`)과 스트림 연결(odom·LiDAR를 그대로 보냄, `stop` 받음) | 신규 |
| 스캔 PC | `station`(`scan3d/station.py`, `scan3d/robot/`, `scan3d/pair/coordinator.py`): 표식 자세, 주행 판단(3.3), 두 대 엇갈림 진행, 원본 저장, 정렬·중앙값, 재구성 실행, 메시 표시 | 신규 |
| 스캔 PC | `recon`(`scan3d/pair/recon.py`): 가장자리 깊이 지우기 → 자르기 → TSDF → 메시(→ 색 다시 입히기) → 치수 보고 | 신규(Open3D 예제 기반) |
| 개발 PC | 표식 판 생성, 합성 데이터, 2D 주행 시뮬레이터, 가짜 에이전트, 테스트 | 신규 |
| 시뮬레이션 PC | Gazebo 스캔 방(우리 world + D455 옆 장착 xacro), 시뮬레이션용 에이전트 | pinky_sim_examples 기반 |

- **네트워크 양**: 스냅샷 1회에 원본 약 4.3 MB(컬러 1.84 MB + 깊이 0.81 MB × 3)다. 로봇이 무손실 압축(Python 표준 `zlib` 수준 1, config `capture.compress`)해 보내 실제 영상에서 약 1.6 MB(컬러 52 ~ 60%, 깊이 19 ~ 25%)가 된다. 한 바퀴에 두 로봇 48회로 약 80 MB이고, 시점마다 나눠서 오간다. 영상을 15 fps로 계속 받으면 초당 약 69 MB이므로 그 대신이다. odom·LiDAR 스트림은 로봇당 초당 수십 KB다.
- **ROS 범위**: 로봇의 기존 설정은 그대로 두고, `scripts/robot.sh`가 스캔 스택을 띄울 때 환경에 로봇별 `ROS_DOMAIN_ID`(L 11, H 12)와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 넣어 ROS 통신을 로봇 안에 둔다. 로봇 밖으로는 TCP 메시지만 오간다.
- 저장소 구조:

```
scan3d/              스캔 PC 코드
                       protocol, clock, config, dataset, station(주 프로그램)
  robot/               로봇 1대: link(연결), unit(스캔 단계), drive, tagpose, align
  pair/                두 대 합침: coordinator(엇갈림 진행), recon(융합, 다시 합치기 명령)
  sim/                 개발용 가짜: synth(합성 영상), sim2d(2D 주행·LiDAR), fake_agent
  vendor/              pinky_sim_examples commands.py 사본과 라이선스
ros/                 로봇 쪽 ROS 2 패키지(ament_python). 설명은 ros/README.md
  pinky_auto/          로봇 구동: robot 런치(모터 드라이버, LiDAR, 센서). 지도·주행·순찰 런치는 스캔에 쓰지 않음
  scan3d_robot/        로봇 에이전트(agent.py·camera.py·node.py), launch/robot_scan.launch.xml
  scan3d_sim/          Gazebo 스캔 시험: 시뮬레이션용 에이전트, scan_sim.launch.py, urdf/(D455 옆 장착 xacro), layouts/(방)
config/              현장 값 scan.yaml, 시뮬레이션용 scan_sim.yaml, 표식 배치 board.json
docker/              스캔 PC 컨테이너(amd64·arm64)
scripts/             설치·배포·실행(setup_station.sh, setup_robot.sh + robot_root_setup.sh, deploy.sh, robot.sh, ssh_aliases.sh, sim_scan.sh)
tools/               표식 판·각도기·시뮬레이션 텍스처 생성, 현장 확인(snapshot_test, show_pose, drive_check, lidar_view, led_check, find_robots), scan.sh, upload.sh
tests/               pytest(합성 데이터, 2D 주행, 가짜 에이전트, 통합), replay/(현장 원자료 재생). ROS 패키지 테스트는 각 패키지의 test/
out/                 표식 판·각도기 PDF, 스캔 결과 사본
docs/                plan/(이 계획, TODOs), specs/(모듈 명세), report/(개발 보고서), archive/(이전 과제), reference/(Open3D 예제 사본)
```

### 3.3 주행

이동은 시점 사이의 짧은 구간(15°, 약 0.17 m)으로 나누고, 시점마다 스냅샷의 표식 자세로 위치를 다시 잡는다. 오차가 다음 구간으로 쌓이지 않는다. 로봇의 `command_driver`가 구간을 odom으로 실행하고, 스캔 PC가 거리 확인, 장애물 판단, 우회 경로를 정한다.

**거리 유지(깊이 최소 측정 거리)**

- 시점마다 스냅샷 직후 표식 자세로 `d = |카메라 위치의 판 좌표 xy| − r_eff`를 계산한다.
- `d`가 0.40 ~ 0.55 m를 벗어나면 반지름 방향으로 보정 이동(`turn; forward; turn`)하고 다시 찍는다(1회). 그래도 벗어나면 그대로 쓰고 보고서에 남긴다.
- 이동 계획이 반지름 `R` 원 위의 점을 목표로 하므로, 정상이면 `d`는 0.45 m 근처다.
- 스냅샷마다 물체 영역에서 깊이가 빈 픽셀 비율을 보고서에 남긴다. 너무 가깝거나 반사가 심하면 이 값이 커진다.
- 표식을 못 본 시점은 마지막 표식 자세에 odom 변화를 더해 `d`와 다음 이동을 계산한다.

**시점 사이 이동**

- 다음 목표: 원 위의 다음 각도(+15°), 진행 방향은 반시계 접선. 시점 이동은 `plan_move(…, final_turn=False)`로 `turn a`, `speed s; forward c`만 보내고 진행 방향 그대로 선다(a 약 15°, c 약 0.17 m). 표식 자세로 고친 yaw와 계획이 몇 도씩 달라 마지막 회전(5 ~ 8°)이 시점마다 한 번 더 들어갔으므로 뺐다. 진입, 거리 보정, 우회 복귀는 마지막 회전까지 한다. 움직일 것이 없으면 보내지 않는다.
- `command_driver`는 단계마다 약 1 cm·2° 덜 가서 멈춘다. 시점마다 표식 자세로 다시 잡으므로 쌓이지 않는다(시점 허용 오차 2.5 cm·5°).
- 직진 속도 0.10 m/s(config). 출발 전 장애물 확인을 거친다.

**장애물(LiDAR, 스캔 PC 판단)**

- 로봇은 LiDAR 스캔(10 Hz)과 odom(20 Hz)을 그대로 스캔 PC로 보낸다. 스캔 PC는 `commands.py`의 `sector_min`으로 진행 방향 ±25° 안의 가장 가까운 거리를 구한다.
- **출발 전**(직진마다, 회전을 마친 뒤): 그 거리가 0.30 m보다 가깝거나, 마지막 스캔이 0.5 s보다 오래되었으면 출발하지 않는다. 제자리 회전은 확인하지 않는다.
- **이동 중**(직진 중): 0.20 m보다 가까워지거나 스캔이 0.5 s 넘게 끊기면 `stop`을 보낸다.
- **막혔을 때**: 5 s를 기다려 비면 이어 간다. 그대로면 바깥 차선으로 비켜 간다.
  1. 지금 자세에서 바깥 차선(반지름 R + 0.25 m)의 같은 각 지점으로 간다.
  2. 바깥 원을 따라 막혔던 시점의 각도까지 간다(직진마다 출발 전 확인).
  3. 그 각에서 안쪽 원으로 돌아가는 직진의 출발 전 확인이 통과하면 돌아와 찍는다. 막혀 있으면 바깥 원에서 다음 시점 각도로 한 칸 더 간 뒤 다시 확인한다. 지나친 시점은 모두 건너뛰고 보고서에 남긴다. 마지막 시점을 넘으면 바깥 차선에서 끝낸다.
  4. 바깥 차선 자체가 막히면 멈추고 화면에 알린 뒤, 길이 비면 이어 간다. 진입·거리 보정·바깥 차선에서 막히거나 스캔이 끊긴 채 `block_timeout_s`(60 s)가 지나면 두 로봇을 멈추고 끝낸다.
  모든 이동 목표는 원 위의 절대 지점이고, 매번 지금 자세에서 다시 계획한다.
- **중간에 멈춘 뒤**: 구간 도중 `stop`으로 멈추면, 마지막 표식 자세에 그 뒤 odom 변화를 평면 강체 변환으로 합성한 위치에서 다음 이동을 다시 계산한다.
- **비켜 갈 수 있는 크기**: 원 경로 중심에서 바깥쪽으로 약 0.1 m 안에 놓인 물체(바깥 차선 0.25 m에서 진행 부채꼴 ±25°의 출발 여유 0.30 m 폭을 뺀 값). 더 크면 길이 빌 때까지 기다린다.
- **다른 로봇**: 반대편(약 1.3 m 건너편)에서 같은 시점 번호로 움직이므로 진행 방향 부채꼴에 들어오지 않는다. 한 로봇이 기다리거나 비켜 가는 동안 다른 로봇은 다음 시점에서 기다린다.
- **LiDAR가 못 보는 것**: 주사면(바닥 위 약 0.125 m)보다 낮은 물체(케이블, 낮은 상자, 표식 판)와 그보다 위에 떠 있는 물체
- 장애물 판단은 config `drive.obstacle`로 끌 수 있다. 잘못된 정지가 두 로봇을 함께 세울 때 쓴다.

**출발과 끝**

- **출발**: 두 로봇을 찍어 표식 자세를 구한다 → 각자 원 위 가장 가까운 점으로 진입한다(장애물 확인 포함) → 다시 찍어 `d`를 확인한다.
- **각도 차**: 두 로봇의 각도 차가 180° ± 30°가 아니면 시작하지 않고, 어느 로봇을 얼마나 옮길지 화면에 알린다.
- **끝**: 24시점을 마치면 제자리에서 멈춘다.
- **통신이 끊기면**: 로봇은 실행 중인 구간 하나(최대 약 0.25 m)를 마치고 멈춘다. 그 뒤에는 명령이 없으므로 움직이지 않는다.

**장착과 LiDAR**

- 카메라 기둥은 차체 뒤쪽에 하나만 세운다. 미니 삼각대처럼 다리가 여러 개인 받침은 LiDAR를 여러 방향에서 가린다.
- 가려지는 각도는 `tools/lidar_view.py`로 확인해 config `lidar.mask`에 적는다. 진행 방향 앞쪽 ±25°는 비어 있어야 한다.

## 4. 접점 명세(확정)

### 4.1 좌표와 자세

- **판 좌표계 B**: 원점은 판 중심의 인쇄면(z = 0, 물체가 놓이는 면), x축은 판의 0° 화살표, z축은 위, 단위 m. 용지를 바닥에 바로 펴 놓으므로 바닥도 z ≈ 0이다.
- **카메라 좌표계 C**: 컬러 카메라의 OpenCV 광학 좌표(x 오른쪽, y 아래, z 앞). 깊이 카메라 자세는 공장 보정값(깊이 → 컬러)으로 구한다.
- **저장하는 자세** `T_cam_board`(4×4): 판 좌표의 점을 컬러 카메라 좌표로 옮기는 변환. `solvePnP` 결과(rvec, tvec)와 같고, 컬러에 정렬한 깊이로 Open3D TSDF `integrate`를 할 때 extrinsic으로 그대로 쓴다.
- **시점 각** `phi`: 카메라 위치의 판 좌표 극각 `atan2(y, x)`, 도 단위
- **장착 변환** `T_base_cam`: config의 대략값(카메라 앞뒤·좌우 위치, 높이, 피치, 왼쪽 보기 yaw +90°). 이동 계산에만 쓰고 재구성에는 쓰지 않는다. 표식 자세에서 로봇 자세를 낼 때는 바닥에 내린 광축 방향과 앞뒤·좌우 위치만 쓴다(높이·피치 오차가 로봇 위치로 번지지 않게).
- **로봇 좌표**: `base_footprint`, x 앞, y 왼쪽, z 위. odom은 로봇 시작점 기준이다.
- **LiDAR 각도**: 스캔 각 0은 로봇 뒤쪽이다(장착 yaw 180°). 스캔 PC는 `sector_min`에 장착 yaw π를 넣어 로봇 기준 각도로 바꾼다.

### 4.2 로봇 스냅샷(`robot_agent`)

- `snapshot(n_depth, n_skip)`: 파이프라인 시작(컬러 1280×720 YUY2 15 fps, 깊이 848×480 Z16 15 fps) → 자동 노출이 맞을 때까지 `n_skip`세트 버림 → 컬러 1장, 깊이 `n_depth`장 → 파이프라인 정지 → 바이트 그대로 반환. 파이프라인이 멈춰 있는 동안에는 그 카메라의 적외선 투사기도 꺼진다고 보고 스냅샷을 한 로봇씩 차례로 찍는다. 이 가정은 현장 1에서 확인한다.
- `calibration()`(한 번): 컬러·깊이 내부 파라미터(폭, 높이, fx, fy, ppx, ppy, 왜곡 모델, 계수), 깊이 → 컬러 외부 파라미터(회전, 이동), 깊이 배율, 카메라 일련번호
- 로봇은 프레임 값을 바꾸지 않는다(형식 변환·정렬·압축 없음. `capture.compress`를 켰을 때만 `zlib` 무손실 압축).
- 머리말의 `fmt`가 형식을 알려 주고 스캔 PC가 형식별로 처리한다: 컬러 `yuyv`·`rgb8`, 깊이 `z16`(값 × `depth_scale` = m)·`f32`(m).
- 시뮬레이션용 에이전트: 같은 메시지 형식으로 Gazebo 토픽(`/d455/image`, `/d455/depth_image`, `/d455/camera_info`)에서 한 장씩 받아 형식 그대로(`rgb8`, `f32`) 보낸다. 외부 파라미터는 단위 행렬이다.
- 가짜 에이전트(개발 PC 테스트): 합성 장면을 렌더링해 실물과 같은 `yuyv`·`z16`으로 보낸다.
- 적외선 확인용 `snapshot(ir=true)`: 이 카메라의 투사기를 끄고 왼쪽 적외선 영상(Y8) 1장만 찍는다(현장 1에서 상대 로봇 투사기 확인).

### 4.3 주행 접점

- **스캔 PC**: `plan_move(pose, target, speed, final_turn=True) -> ["turn a", "speed s; forward c", "turn b"]`(순수 함수, 판 좌표 2D 자세, 0인 부분과 움직임 없는 문장은 뺌), 거리 확인·장애물 판단·우회(3.3). 자세한 것은 `docs/specs/drive.md`
- **로봇** `move(text)`: `/pinky_command`에 문장을 보내고 `/command_status`에서 `(모든 명령 완료)`가 오면 odom을 돌려준다. 구간 사이에서 `blocked`나 `error`가 오면 실패로 돌려준다.
- **로봇** `stop`: `/pinky_command`에 `stop`을 보낸다(즉시 정지, 남은 명령 취소). 이때 진행 중이던 `move`는 `{"ok":true,"stopped":true,"odom":[…]}`로 끝난다.
- **로봇 스트림**: `/odom`, `/scan`을 받은 그대로 보낸다.

### 4.4 스캔 PC ↔ 로봇 메시지

모든 메시지는 한 줄에 JSON 하나다. 실패 응답은 `{"ok": false, "error": "..."}`이고, 받으면 스캔 PC가 두 로봇에 `abort`를 보내고 끝낸다. 영상이 있는 응답의 바이트 규칙, 압축 해제와 머리말 정리는 `docs/specs/protocol.md`를 따른다.

**제어 연결(TCP 7010, 요청 하나에 응답 하나)**

| 요청 | 응답 | 로봇 동작 |
|---|---|---|
| `{"cmd":"hello","color":[1280,720,15],"depth":[848,480,15]}` | `{"ok":true,"role":"L","serial":"...","calib":{...}}` | 4.2 `calibration()`, 스냅샷 프로파일 고정, 중단 상태 해제 |
| `{"cmd":"move","text":"speed 0.10; forward 0.17","timeout_s":60}` | `{"ok":true,"stopped":false,"odom":[x,y,yaw]}` | 4.3 `move`(문장 앞 `speed`는 config `drive.speed`) |
| `{"cmd":"snapshot","n_depth":5,"n_skip":5,"compress":false,"color":[1280,720,15],"depth":[848,480,15]}` | `{"ok":true,"t":…,"odom":[…],"color":{"w":1280,"h":720,"fmt":"yuyv","bytes":1843200},"depth":{"w":848,"h":480,"fmt":"z16","n":5,"bytes":4070400}}` + 컬러 바이트 + 깊이 바이트 | 4.2 `snapshot` |
| `{"cmd":"snapshot","ir":true,"n_skip":5,"depth":[848,480,15]}` | `{"ok":true,"t":…,"odom":[…],"ir":{"w":848,"h":480,"fmt":"y8","bytes":407040}}` + 바이트 | 4.2 적외선 확인 |

**스트림 연결(TCP 7011, 양방향)**

| 방향 | 메시지 | 주기 |
|---|---|---|
| 로봇 → PC | `{"type":"odom","t":…,"x":…,"y":…,"yaw":…}` | 20 Hz |
| 로봇 → PC | `{"type":"scan","t":…,"angle_min":…,"angle_inc":…,"range_min":…,"range_max":…,"ranges_mm":[…]}`. 무효값(유한하지 않음, 범위 밖)은 0 | 10 Hz(LiDAR 주기) |
| 로봇 → PC | `{"type":"status","t":…,"text":"…"}`(`/command_status` 그대로) | 올 때 |
| PC → 로봇 | `{"type":"stop"}`(장애물 정지, 다음 `move`는 받음) | 필요할 때 |
| PC → 로봇 | `{"type":"abort"}`(스캔 중단, 다음 `hello`까지 모든 `move` 거절) | 실패할 때 |

**스캔 PC 진행 순서**

1. 두 로봇 `hello`, 스트림 연결
2. L·H `snapshot`(차례로) → 표식 자세 → 3.3 출발(진입, 각도 차 확인)
3. 시점 k = 0…23마다:
   1. 로봇마다 따로 진행한다. 시점이 k보다 작은 로봇은 다른 로봇이 시점 k − 1 이상에 도착한 뒤 시점 k로 이동한다(한 시점 넘게 앞서지 않음). 직진마다 출발 전 확인과 이동 중 감시(3.3), 제자리 회전은 확인 없음. k = 0은 진입 직후라 움직이지 않는다.
   2. `settle_s` 동안 기다린다.
   3. 촬영 차례를 얻어 `snapshot` → 표식 자세 → 거리 확인(벗어나면 차례를 놓고 보정 이동 후 한 번 다시 찍음). 스냅샷은 한 번에 한 로봇만 찍고, 그동안 다른 로봇은 다음 시점으로 이동한다(엇갈림). 두 대가 차례로 찍고 함께 이동하면 로봇마다 시점당 촬영 한 번(약 3.3 s)을 더 서 있게 된다.
4. 중앙값·정렬·왜곡 보정 → 재구성 → 메시 창

### 4.5 데이터셋 형식(스캔 PC)

```
scans/<scan_id>/<role>/raw/           받은 그대로
  calib.json                          hello 응답
  000000.json …                       snapshot 머리말(t, odom, 크기) + station 기록(k, use, phi_target)
  000000_color.yuyv …                 컬러 바이트
  000000_depth.z16 …                  깊이 바이트(n장 이어 붙임)
  stream.jsonl                        odom·LiDAR·status 스트림, 보낸 move·stop, 줄마다 스캔 PC 시각
scans/<scan_id>/<role>/               raw에서 만든 Open3D 예제 형식
  color/000000.jpg …                  BGR로 바꾸고 왜곡 보정한 컬러
  depth/000000.png …                  픽셀별 중앙값을 컬러에 정렬하고 왜곡 보정, uint16 mm
  camera_intrinsic.json               왜곡 보정한 컬러 카메라 PinholeCameraIntrinsic(width, height, intrinsic_matrix 열 우선)
  views.json                          [{k, snap, phi, d, T_cam_board(4×4), n_tags, reproj_px, depth_invalid}]
  outcomes.json                       시점마다 찍음·자세·d와 출처·빠진 이유
scans/<scan_id>/report.json           건너뛴 시점, 정지·거리 보정 횟수, 중단 원인, 걸린 시간
scans/<scan_id>/events.jsonl          구간 결과, 정지, 우회, 거리 보정 기록
scans/<scan_id>/config.yaml, board.json  시작할 때의 설정과 배치 파일 복사
scans/<scan_id>/out/                  mesh.ply, cloud.ply, report.txt(치수, 시점별 d, 건너뛴 시점)
```

`scans/<scan_id>/`의 `config.yaml`, `board.json`, `events.jsonl`, `raw/`만 있으면 나머지를 모두 다시 만들 수 있고, 주행 판단도 `stream.jsonl`로 다시 돌려 볼 수 있다.

### 4.6 설정 파일 `config/scan.yaml`

| 묶음 | 항목 |
|---|---|
| `board` | 배치 파일, 표식 한 변(mm) |
| `scene` | `r_obj`, 놓는 오차, 물체 높이, `R`, 시점 수, 자르기 여유, 판 높이 자르기(`floor_cut`) |
| `capture` | 컬러·깊이 프로파일, `n_depth`, `n_skip`, `settle_s`, `compress` |
| `drive` | 직진 속도, `d` 목표·하한·상한, `obstacle` 켜기, 진행 부채꼴 반폭(25°), 출발 여유(0.30 m), 이동 중 정지 거리(0.20 m), 기다림(5 s), 우회 차선 폭(0.25 m), 스캔 끊김 한도(0.5 s), 두 로봇 각도 차 허용(30°), 막힘 한도(60 s) |
| `lidar` | 장착 yaw(`mount_yaw_deg` 180), 가린 각도 `mask`(로봇 기준 `[[from_deg, to_deg], …]`) |
| `robots.L`, `robots.H` | 호스트, 포트, 장착 변환(앞뒤·좌우·높이·피치), `wheel_radius`·`wheel_separation` |
| `recon` | 복셀 크기(2 mm), 절단 거리(6 mm), 가장자리 깊이 지우기 기준(`edge_mm` 10), 색 다시 입히기 반복 수(`color_opt_iter`, 0이면 끔) |
| `station` | 데이터 폴더, 메시 창 띄우기 |

## 5. 모듈 명세와 먼저 쓸 테스트

| 모듈(위치) | 명세 핵심 | 먼저 쓸 테스트 |
|---|---|---|
| `board`(`tools/make_board.py`) | 배치 json, 실제 크기 인쇄 PDF(화살표·축 포함) | 생성 이미지에서 16장 모두 검출, 모서리 좌표가 배치 파일과 일치 |
| `synth`(개발 PC) | 메시 + 판 → 컬러·깊이(Open3D `RaycastingScene`, 판 영상은 호모그래피), 깊이 카메라와 컬러 카메라를 따로 렌더링 | 다른 테스트의 입력 |
| `sim2d`(개발 PC) | 문장 명령대로 움직이는 로봇(odom 잡음 옵션), 물체 원·다른 로봇·장애물 상자에 대한 2D LiDAR(장착 yaw 180°) | 다른 테스트의 입력 |
| `protocol`(공통) | 4.4 제어·스트림 메시지 | 보내고 받은 바이트가 같음 |
| `align`(PC) | YUY2 → BGR, 깊이 중앙값 → 컬러에 정렬(software device + `rs.align`) → 컬러 왜곡 계수로 함께 왜곡 보정 | 합성 깊이(깊이 카메라 시점)를 정렬한 결과가 컬러 시점 깊이와 비교해 깊이 경계 밖 픽셀의 95%가 1.2 mm 안, 최대 2.5 mm |
| `tagpose`(PC) | 왜곡 보정한 영상에서 검출, 보이는 표식 전부로 PnP 한 번 | 알려진 자세로 투영한 합성 영상에서 위치 3 mm, 각도 1° 안 |
| `drive`(PC) | 3.3 전부 | `sim2d`로 24시점 한 바퀴, 시점마다 2.5 cm·5° 안, `d` 0.40 ~ 0.55 m. 로봇 정면에 둔 상자가 `sector_min`(yaw π)에서 정면 거리로 나옴. 원 경로의 상자(구간 가운데, 다음 시점 바로 앞 두 위치) → 멈춤 → 우회 → 시점 1 ~ 2개 건너뜀 → 반지름 `R`로 복귀. 구간 도중 `stop` 뒤 다시 계산한 이동이 원으로 돌아옴. 너무 가까운 출발 → 보정 이동 뒤 `d` ≥ 0.40 m. 스캔이 끊기면 출발 안 함 |
| `dataset`(PC) | 4.5 `raw/` → Open3D 형식 | 같은 `raw/`에서 두 번 만든 결과가 같음, Open3D가 `camera_intrinsic.json`을 읽음 |
| `recon`(PC) | 자르기 → TSDF → 메시 → 치수 | 합성 상자 48시점에서 치수 5 mm 안, 메시 1개, 색 순서 맞음 |
| `station`(PC) | 4.4 진행 순서, 실패 시 `abort` | 가짜 에이전트 2개로 메시지 순서, 스냅샷이 한 로봇씩 차례로, 한 로봇이 우회하는 동안 다른 로봇이 기다림, 각도 차 180° 유지 |
| `robot_agent`(로봇) | 4.2·4.3·4.4 | 가짜 파이프라인·가짜 `/command_status`·가짜 `/odom`·`/scan`으로 메시지 처리, 바이트 크기, `stop`·`abort` 전달, 중단 잠금 |
| 재생 | 현장 `scans/<scan_id>/`(`config.yaml`, `board.json`, `events.jsonl`, `raw/`) | 현장 1 이후 추가: 실제 영상의 표식 자세, 정렬, 재구성, `stream.jsonl`로 주행 판단 |

통합 시험:

- **I1(개발 PC)**: 가짜 에이전트 2대(`sim2d` + `synth`) → `scan_station` 전체 흐름 → 메시. 원 경로에 상자를 하나 둔 경우도 돈다.
- **I2(시뮬레이션 PC Gazebo)**: 우리 world에서 `command_driver`, LiDAR, 시뮬레이션용 에이전트로 L 설정 한 바퀴, H 설정 한 바퀴 → 재구성. 한 바퀴 도중 `spawn_object`로 원 경로에 상자를 놓는다. 반나절 안에 안 되면 건너뛰고 I1과 현장 1로 대신한다.

## 6. 단계 순서(세부는 `TODOs.md`)

| 단계 | 장소 | 앞 단계 | 끝나는 조건 |
|---|---|---|---|
| 온라인 1: 명세, 테스트, 스캔 PC 모듈, 로봇 에이전트, 표식 판 파일, 현장 도구 | 개발 PC | — (표식 판 파일을 가장 먼저) | 테스트 전부 통과, I1 통과, `board_A1.pdf` |
| 온라인 2: Gazebo 시험 | 시뮬레이션 PC | 온라인 1 | I2 통과 또는 건너뜀 |
| 온라인 3: 두 로봇 원격 설치·확인 | 로봇(켜진 상태) | 온라인 1, 준비 0-1 | 두 로봇 스냅샷 24회 연속 성공, odom·LiDAR 스트림 수신, 이동 확인 |
| 현장 1: 장착, 확인, 한 대씩 한 바퀴, 녹화 | 현장 | 온라인 3, 표식 판 출력 | 실물 `raw/`, 한 대 메시, 장애물·근접 출발 확인 |
| 온라인 4: 현장 데이터로 수정 | 개발 PC | 현장 1 | 재생 테스트 통과 |
| 현장 2: 두 대 동시, 기준 상자 | 현장 | 온라인 4 | 1장 완료 기준 |
| 현장 3: 시연 물체, 리허설 | 현장 | 현장 2 | 3회 연속 성공 |
| 시연 | 현장 | 현장 3 | |

## 7. 구현에 필요한 사실

- **로봇 접속**: 로봇은 자체 Wi-Fi AP(`pinky_XXXX`, 로봇 주소 192.168.4.1)를 띄우고, Pinky Studio로 공유기 Wi-Fi에도 연결할 수 있다. 접속은 `ssh pinky@<로봇 주소>`다.
- **배포·실행**: `scripts/deploy.sh`(로봇 `~/pinky_auto_ws` 오버레이, 공식 `~/pinky_pro`는 건드리지 않음), `scripts/robot.sh start|stop|status`. 백그라운드 `ros2 launch`는 SIGINT를 무시하므로 프로세스 그룹에 SIGTERM을 보낸다.
- **직렬 포트**: 모터 `/dev/ttyAMA4`, LiDAR `/dev/ttyAMA0`는 한 프로세스만 쓸 수 있다. 다른 bringup이 떠 있는지 먼저 본다.
- **로봇 자원**: Pi 5 RAM 2 GB
- **USB 전원**: Pi 5는 5 V 5 A 전원을 감지하면 USB 기기에 1.6 A, 아니면 600 mA를 준다. 로봇의 Pi는 3 A 전원으로 감지하고(`/proc/device-tree/chosen/power/max_current` 3000), D455는 USB에 720 mA를 선언한다(최대 약 0.69 A). 그래서 `config.txt`에 `usb_max_current_enable=1`을 넣어 1.6 A로 올린다(재부팅 필요, `scripts/robot_root_setup.sh`). 설정 전에도 레이저 최대로 20초 스트리밍은 저전압 없이 됐다. 모터를 돌리며 스냅샷이 끊기면 보조배터리 USB 허브를 쓴다.
- **LiDAR**: RPLiDAR C1이 차체 맨 위에 있고 주사면은 바닥 위 약 0.125 m다(공식 URDF). 로봇에 180° 돌아가 붙어 있어 스캔 각 0이 로봇 뒤쪽이다. 주사면보다 낮은 물체는 보이지 않는다(pinky_sim_examples README). `pinky_auto` robot 런치가 `/scan`을 낸다.
- **로봇 카메라 소프트웨어**: PyPI `pyrealsense2` 2.59.2(aarch64·Python 3.12 휠, librealsense 포함)를 venv에 설치한다(sudo 필요 없음). 이 휠은 V4L2로 카메라에 접근하고 로봇의 영상 장치 파일은 누구나 열 수 있어, udev 규칙 없이도 카메라가 열린다. sudo로는 udev 규칙(librealsense 저장소 `config/99-realsense-libusb.rules`)과 위 USB 전류 한도만 설정한다. 로봇에는 OpenCV가 필요 없다. venv는 `--system-site-packages`로 만들어 ROS의 `rclpy`와 `pyrealsense2`를 한 프로세스에서 쓰고, `robot_scan.launch.xml`은 `robot_agent`를 그 venv의 python으로 실행한다(`ExecuteProcess`). 휠이 안 되면 apt `ros-jazzy-librealsense2` 2.59.1의 `rs-save-to-disk`로 찍는다(이 apt 패키지에는 Python 바인딩과 udev 규칙이 없다).
- **D455 컬러 형식**: 데이터시트의 동시 스트림 표(Table 4-5)에서 D455 컬러는 YUY2(16 bit)다. RGB8을 요청하면 librealsense가 로봇에서 변환하므로 YUY2로 받는다. 쓸 프로파일(1280×720 YUY2, 848×480 Z16, 15 fps)이 있는지는 `scripts/setup_robot.sh`가 카메라에서 확인해 출력한다.
- **스캔 PC 저장소**: 스캔 PC는 이 저장소를 clone해서 쓰고, `scripts/setup_station.sh`가 Python 패키지를 설치한다. 같은 환경을 Docker 컨테이너로도 꾸릴 수 있다(`docker/`, amd64·arm64).
- **스캔 PC Python**: `numpy` 1.26, `pyrealsense2==2.59.2.11152`, `open3d==0.20.0`, `opencv-contrib-python-headless==4.11.0.86`(5.x는 numpy 2 필요). 이 조합이 Python 3.12(x86_64)에서 함께 import되고 `rs.software_device()`가 만들어지는 것을 x86_64 Linux에서 확인했다.
- **`command_driver`**: 제어 주기 20 Hz로 이동 중 `cmd_vel`을 계속 보내므로 모터 드라이버의 0.5 s 명령 끊김 정지와 맞는다. 거리·각도 허용 오차는 1 cm·2°다. 구독이 연결되기 전에 보낸 명령은 사라지므로 `robot_agent`는 `/pinky_command` 구독자가 생긴 뒤에 보낸다. 자체 LiDAR 정지(`stop_distance`, 기본 0.20 m)가 있지만 장애물 판단은 스캔 PC가 하므로 0.0으로 끈다. `package.xml`에 Gazebo·Nav2 패키지가 실행 의존으로 적혀 있으므로 로봇에서는 rosdep 없이 `colcon build --packages-select pinky_sim_examples`로만 빌드한다.
- **Gazebo D455**(pinky_sim_examples): 정면 방향, 640×360, 15 Hz, near 0.6 m, 질량 0.39 kg으로 정의되어 있다. 우리 xacro에서 yaw +90°, 높이, 피치, near 0.4 m, 질량 0.116 kg으로 바꾼다. 깊이는 32FC1(m). 그 launch는 Gazebo 서버를 `-r -s`로만 띄우므로 화면 없이(SSH 등) 돌릴 때는 `--headless-rendering`을 더해야 카메라가 나온다.
- **여러 로봇 네트워크**: 여러 Pinky를 한 공유기에 물리면 DDS 트래픽으로 통신이 끊길 수 있다(공식 README). 3.2의 로컬 탐색 설정으로 피한다.
- **D455**: 측정 범위 0.4 m 이상, 848×480 최소 깊이 0.35 m, 컬러는 글로벌 셔터·최대 1280×800·시야각 90°×65°, 6축 IMU, 1/4-20 삼각대 나사, 116 g
- **Pinky Pro**: 110 × 120 × 142 mm(W×D×H), 맨 위 RPLiDAR C1, 공개된 장착 구멍 없음, `wheel_separation` 0.0961 m

## 8. 정해지지 않은 것(기본값으로 진행)

1. **대상 물체**: 3.1의 조건. 기준 물체는 다이소 크라프트 상자(`TODOs.md` 0단계)
2. **정밀도**: 치수 오차 ±1 cm
3. **윗면**: 포함(H 높이 0.45 m, 약 22°로 내려다봄)
