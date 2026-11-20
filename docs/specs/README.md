# 모듈 명세

`docs/plan/PLAN.md` 4장(접점)과 5장(모듈 표)을 모듈마다 1쪽 명세로 옮기고, 명세의 완료 테스트를 먼저 쓴 뒤 구현한다.

## 진행

1. 명세를 쓴다(아래 양식).
2. 완료 테스트를 쓰고 실패하는 것을 본다.
3. 구현해서 `pytest -q`를 통과시킨다.
4. `docs/plan/TODOs.md`의 해당 항목을 체크한다.

명세가 PLAN 4장 접점과 달라지면 PLAN을 먼저 고친다.

## 명세 목록

| 순서 | 명세 | 코드 | PLAN | 테스트 | 가져다 쓰는 것 |
|---|---|---|---|---|---|
| 1 | `board.md` | `tools/make_board.py` | 3.1, 4.1, 5 | `tests/test_board.py` | — |
| 2 | `synth.md` | `scan3d/synth.py` | 5 | `tests/test_synth.py` | board |
| 3 | `sim2d.md` | `scan3d/sim2d.py` | 3.3, 5 | `tests/test_sim2d.py` | vendor `parse` |
| 4 | `protocol.md` | `scan3d/protocol.py` | 4.4 | `tests/test_protocol.py` | — |
| 5 | `align.md` | `scan3d/align.py` | 4.2, 4.5, 5 | `tests/test_align.py` | synth |
| 6 | `tagpose.md` | `scan3d/tagpose.py` | 4.1, 5 | `tests/test_tagpose.py` | board, synth, align |
| 7 | `drive.md` | `scan3d/drive.py` | 3.3, 4.3 | `tests/test_drive.py` | sim2d, vendor `parse`·`sector_min` |
| 8 | `dataset.md` | `scan3d/dataset.py` | 4.5 | `tests/test_dataset.py` | align, tagpose |
| 9 | `recon.md` | `scan3d/recon.py` | 5 | `tests/test_recon.py` | synth, dataset |
| 10 | `fake_agent.md` | `scan3d/fake_agent.py` | 4.2, 4.4 | `tests/test_fake_agent.py` | synth, sim2d, protocol |
| 11 | `station.md` | `scan3d/station.py` | 4.4, 5 | `tests/test_station.py`, I1 `tests/test_integration.py` | 전부 |
| 12 | `robot_agent.md` | `ros/scan3d_robot/` | 4.2~4.4, 7 | `tests/test_robot_agent.py` | protocol |

`board.md`가 먼저다. 표식 판 PDF를 출력소에 맡겨야 하기 때문이다.

## 명세 양식

````markdown
# <모듈>

- 코드: `scan3d/<모듈>.py`
- 테스트: `tests/test_<모듈>.py`
- PLAN: <절>
- 출처 예제: <저장소> <버전> `<경로>`

## 하는 일

한두 문장

## 인터페이스

공개 함수·클래스의 시그니처와 반환값. 단위와 좌표계를 적는다.

## 동작

번호 목록. 정상 흐름만 쓰고, 벗어나면 올리는 예외를 적는다.

## 설정

`config/scan.yaml`에서 읽는 키

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
````

## 공통 규칙

- **단위**: 길이 m, 시간 s. 각도는 코드 안에서 rad이고, `config/scan.yaml`(`_deg` 키), 이동 문장, `views.json`의 `phi`, 보고서에서는 도다.
- **변환 이름**: `T_a_b`는 b 좌표의 점을 a 좌표로 옮기는 4×4 행렬이다(`p_a = T_a_b @ p_b`). PLAN 4.1의 `T_cam_board`, `T_base_cam`, `T_board_base`가 이 규칙을 따른다.
- **모듈 사이 영상**: 컬러는 BGR `uint8` H×W×3, 깊이는 `uint16` mm H×W. 원본 바이트 형식(`yuyv`, `z16`, `rgb8`, `f32`)은 protocol, align의 `decode_*`, dataset `raw/`에서만 다룬다.
- **설정**: 모듈 함수는 필요한 config 묶음(dict)을 인자로 받는다. `config/scan.yaml` 파일을 읽는 곳은 station과 `tools/`뿐이다. `dataset.build`는 스캔할 때 복사해 둔 `scans/<scan_id>/config.yaml`을 읽는다.
- **오류**: 정상 흐름을 벗어나면 예외를 올린다. station이 받아 두 로봇에 `abort`를 보내고 끝낸다.
- **외부 코드**: 이동 문장 해석과 `sector_min`은 `scan3d/vendor/pinky_commands.py`를 그대로 쓴다.

## 테스트 규칙

- 저장소 폴더에서 `.venv/bin/python -m pytest -q` 하나로 전부 돈다. 로봇, 카메라, ROS, Gazebo 없이 개발 PC에서 돌고(소켓은 localhost만), 그 자리는 `synth`, `sim2d`, 가짜 에이전트가 맡는다. board 테스트는 pdftoppm(poppler-utils)이 필요하다.
- 파일: `tests/test_<모듈>.py`. 현장 원본으로 도는 재생 테스트는 `tests/replay/`, 그 원본은 `tests/data/`.
- fixture(`tests/conftest.py`): `cfg`(테스트마다 새로 읽은 `config/scan.yaml`, 바꿔 써도 됨), `rng`(seed 0), `repo_root`
- 통과 기준 수치는 PLAN 5장 값을 그대로 쓴다.
- 오래 걸리는 테스트(합성 48시점 재구성, I1)에는 `@pytest.mark.slow`를 붙인다. 빠르게 돌릴 때는 `pytest -q -m "not slow"`.
- 환경 확인: `scripts/setup_station.sh`가 `.venv`를 만들고 `tests/test_env.py`를 돌린 뒤 `station ok`를 출력한다.

## 외부 코드에서 확인한 사실

### pinky_sim_examples `1d14183`(`command_driver.py`, `commands.py`)

- **이동 문장**: `forward <m>`·`back <m>`(0 < m ≤ 5), `turn <deg>`(0이 아니고 |deg| ≤ 720, +가 반시계), `speed <m/s>`(0.02~0.25), `stop`을 `;`로 잇는다. 한 부분이라도 틀리면 문장 전체를 거부한다. 그래서 `plan_move`는 0인 `turn`·`forward`를 빼고 만들고, 테스트는 만든 문장을 `parse`에 넣어 본다.
- **`/command_status`**: `accepted: <문장> (대기 n단계)`, 단계마다 `done: move +0.17 m`·`done: turn +8 deg`이고 마지막 `done`에만 ` (모든 명령 완료)`가 붙는다. 그 밖에 `speed: 0.06 m/s`, `blocked: …`, `error: …`(해석 실패), `stopped: 남은 명령 취소`(`stop`).
- **정지**: `stop`과 `blocked`는 남은 명령을 지우고 바로 멈춘다.
- **속도**: 직진은 끝에 가까우면 남은 거리 × 1.5로 줄인다(최저 0.03 m/s). 회전은 최고 0.8 rad/s, 최저 0.15 rad/s. 허용 오차 1 cm, 2°, 제어 20 Hz. 직진 속도 파라미터 `linear_speed`의 기본값은 0.10 m/s다.
- **odom**: 한 번도 받지 않았으면 명령을 실행하지 않는다.
- **`sector_min(ranges, angle_min, angle_increment, center, half_width, mount_yaw=0, range_min=0)`**: 거리 m, 각도 rad. 유한하지 않거나 `range_min`보다 작은 값만 건너뛴다. 가린 각도 인자는 없고, 유효값이 없으면 `inf`다.

### 예제 버전

| 출처 | 버전 | 쓰는 파일 |
|---|---|---|
| realsenseai/librealsense | `v2.59.2`(pyrealsense2 2.59.2.11152) | `wrappers/python/examples/align-depth2color.py`, `align-with-software-device.py`, `box_dimensioner_multicam/`, `depth_auto_calibration_example.py`, `config/99-realsense-libusb.rules` |
| isl-org/Open3D | `v0.20.0`(사본 `docs/reference/open3d-v0.20.0/`) | `examples/python/t_reconstruction_system/integrate.py`, `examples/python/reconstruction_system/sensors/realsense_recorder.py` |
| opencv/opencv_contrib | `4.11.0` | `cv2.aruco`(`DICT_APRILTAG_36h11` 587장, `ArucoDetector`, `Board`) |

## 명세에서 정한 것

- **LiDAR 무효값**(protocol): `scan`에 `range_min`, `range_max`를 싣고, 무효값은 `ranges_mm`에서 0이다. `decode_scan`이 0을 `inf`로 되돌린다.
- **이동 문장**(drive): 한 구간을 `turn a`, `speed s; forward c`, `turn b`로 나눠 하나씩 보낸다. LiDAR 확인은 직진에만 한다. 움직임이 없으면 보내지 않는다(`speed`만 있는 문장은 `command_driver`가 완료를 내지 않는다). `command_driver` 파라미터는 기본값 그대로다.
- **시점 위치 오차**(drive, sim2d): `command_driver`는 단계마다 약 1 cm·2° 덜 간다. 시점마다 표식 자세로 다시 잡으므로 쌓이지 않고, 시점 허용 오차는 2.5 cm·5°다.
- **로봇 자세**(drive): 표식 자세에서 로봇 자세를 낼 때 카메라 높이·피치를 쓰지 않고 바닥에 내린 광축 방향과 장착 xy만 쓴다. odom은 평면 강체 변환으로 합성한다.
- **로봇 주소**(station): `robots.<role>.host`는 `ssh -G <host>`의 `hostname`으로 바꿔 연결한다.
- **가린 각도**(drive): `lidar.mask`는 로봇 기준 각도 범위 `[[from_deg, to_deg], …]`(0이 앞, from에서 반시계로 to까지)
- **로봇의 protocol**(robot_agent): `protocol.py`는 표준 라이브러리만 쓰고, `deploy.sh`가 로봇 `~/scan3d_lib/scan3d/`에 복사한다. 본체 `agent.py`는 rclpy·pyrealsense2를 import하지 않는다.
- **정렬**(align): software device + `rs.align` 하나만 쓴다(numpy 대체 경로 없음). 쌍을 다시 넣을 때는 시도마다 `timestamp`·`frame_number`를 늘린다. 정렬한 컬러·깊이를 컬러 왜곡 계수로 함께 펴서 그 뒤는 모두 핀홀 K로 다룬다. 깊이 n장의 중앙값을 먼저 내고 정렬은 한 번 한다.
- **픽셀 중심**(synth, tagpose): 정수 좌표가 픽셀 중심이다. OpenCV `CORNER_REFINE_APRILTAG` 모서리에서 0.5 px를 뺀다.
- **재구성**(recon): Open3D `compute_unique_block_coordinates` → `integrate`(절단 거리는 `trunc / voxel` 배수, 6 mm). 자르고 남은 것이 없는 시점은 건너뛴다. 겹침 보정(ICP)은 현장 데이터에서 겹친 벽이 보일 때 온라인 4에서 다룬다.
- **시간**(station, fake_agent): 모든 기다림과 한도는 하나의 `Clock`을 쓰고, `--fake`면 가짜 에이전트와 같은 시계를 쓴다.
- **중단**(station, protocol, robot_agent): 실패하면 공유 취소를 켜고 모든 로봇에 `abort`를 보낸다. 로봇은 다음 `hello`까지 중단 상태로 모든 `move`를 거절하고, 문장 발행과 중단 처리를 같은 잠금 안에서 해서 늦게 도착한 `move`가 로봇을 다시 움직이지 않는다. 장애물 정지 `stop`은 이어 갈 수 있는 정지다.
- **프로파일**(protocol): `hello`에서 스냅샷 프로파일을 정하고 그 보정값을 받는다. 다른 프로파일의 스냅샷 요청은 거절한다.
- **정렬 깊이 값**(align): `rs.align`의 깊이 카메라 z를 컬러 카메라 z로 바꿔 쓴다(`z_c = (z_d + r3·t) / (r3·ray)`).
- **기록**(dataset): `scans/<scan_id>/`에 config·배치 파일 복사, `events.jsonl`, `report.json`을 남겨 `raw/`와 함께 다시 처리할 수 있게 한다.
