# drive

- 코드: `scan3d/robot/drive.py`
- 테스트: `tests/test_drive.py`
- PLAN: 3.3, 4.1, 4.3
- 출처 예제: `scan3d/vendor/pinky_commands.py`(`parse`, `sector_min`, `wrap_angle`), pinky_sim_examples `1d14183` `command_driver.py`(정지 허용 오차 1 cm·2°)

## 하는 일

로봇 한 대의 원 주행을 맡는다. 표식 자세와 odom으로 위치를 알고, 시점 사이 이동을 회전·직진 단위로 나눠 보내고, LiDAR로 직진의 출발·정지를 판단하고, 막히면 기다린 뒤 바깥 차선으로 비켜 간다. 두 대 맞춤 진행은 station이 한다.

## 인터페이스

- 자세 `Pose2 = (x, y, yaw)`: 로봇 기준점(`base_footprint`, x 앞, y 왼쪽, z 위), yaw rad
- `base_from_cam(T_cam_board, mount) -> Pose2`(판 좌표): 카메라 광축(+z)을 바닥에 내린 방향의 각에서 `mount.yaw_deg`를 뺀 것이 로봇 yaw, 카메라 xy에서 `Rot(yaw)·(mount.x, mount.y)`를 뺀 것이 로봇 xy. 카메라 높이와 피치는 쓰지 않는다.
- `cam_xy(pose, mount) -> (x, y)`: 위의 역. 표식이 안 보인 시점의 `d` 계산에 쓴다.
- `view_pose(phi_deg, radius, mount, yaw=None) -> Pose2`: 카메라가 반지름 `radius` 원의 각 `phi`에 오는 로봇 자세. `yaw`가 없으면 phi + 90°(반시계 접선), 있으면 그 yaw로 섰을 때 카메라가 그 자리에 오는 로봇 위치
- `camera_pose(pose, mount) -> T_cam_board`: 로봇 자세와 장착(높이 `z`, 피치 포함)으로 구한 컬러 카메라 자세. `base_from_cam`의 역이고, 테스트와 가짜 에이전트가 쓴다. `synth.pose_on_circle`과 같은 규칙이다.
- `Driver.phi_deg(k)`: 시점 k의 목표 각(도, `enter()` 뒤)
- `arc_yaw(phi_deg, radius, mount) -> yaw`: 카메라가 반지름 `radius` 원의 각 `phi`에 있고 판 중심 둘레의 원호로 달릴 수 있는 로봇 방향. 바퀴 축의 연장선이 판 중심을 지나는 방향이고 `phi + acos(mount.x / radius)`다(카메라가 0.08 m 뒤, 반지름 0.62 m면 카메라 원의 접선보다 7.4° 안쪽)
- `plan_orbit(pose, target, speed) -> list[str]`: 판 중심 둘레의 원 위 자세 `target`(`arc_yaw` 방향)까지 원호 하나로 가는 문장 `speed s; arc r θ`. 기준 경로는 판 중심 둘레, 목표의 기준점 반지름 r의 원. 지금 자세가 원에서 δ만큼 바깥(안쪽은 음수), 방향이 접선에서 e만큼 왼쪽(오른쪽은 음수)이면 휨 = `1/r + k1·δ − k2·e`, `k1 = (1 − p)² / s²`, `k2 = (3 + p)(1 − p) / (2s)`, s는 목표까지의 원호 길이, p = `ORBIT_POLE`(0.5). 문장은 반지름 `1/|휨|`(0.001 m로 반올림), 각도 `s × 휨`(0.01°, 왼쪽으로 돌면 양수). 회전 문장을 쓰지 않는다. 휨이 1/20 m보다 작으면 `speed s; forward s`. 목표가 2 cm 안이거나 뒤쪽이거나, 원에서 5 cm·20° 넘게 벗어났거나, 반지름이 0.10 m보다 작아지면 `plan_move`의 문장. 원호 문장은 `protocol.parse_arc`를 통과한다(vendor `parse`는 원호를 모른다).
- `plan_move(pose, target, speed, final_turn=True) -> list[str]`: 순서대로 보낼 문장들. `turn a`(목표 위치 방향으로), `speed s; forward c`(0.25 m보다 길면 나눔), `turn b`(목표 yaw로, `final_turn`이 거짓이면 빼서 진행 방향 그대로 선다). 각도 0.1°, 거리 0.001 m로 반올림. |a|·|b| < 1°인 회전, c < 0.005 m인 직진은 뺀다. c < 0.02 m이면 직진 없이 목표 yaw로 `turn` 하나. 할 것이 없으면 빈 목록. 각 문장은 `parse`를 통과한다.
- `in_mask(angle_deg, mask) -> bool`: 로봇 기준 각이 `lidar.mask`의 어느 범위 안인지
- `front_min(scan, age_s, cfg) -> float`: 로봇 정면 ±`sector_half_deg` 안의 최소 거리. `lidar.mask`의 각 범위(`[from_deg, to_deg]`, from에서 반시계로 to까지, 360으로 나눈 나머지)의 값은 버린다. `scan`이 없거나 `age_s` > `scan_timeout_s`이면 0
- `RobotIO`(station의 `RobotLink`와 `sim2d.SimIO`가 구현): `move(text, watch=None) -> MoveResult(ok, stopped, odom)`(이동 중 0.1 s마다 `watch()`, true면 `stop`), `scan() -> (scan, age_s)`, `odom() -> Pose2`(odom 좌표), `now()`, `sleep(s)`, `say(text)`, `cancelled() -> bool`
- `Driver.arc`: 시점 이동을 원호로 할지(`drive.arc`, 기본 참). 참이면 시점·바깥 차선의 목표 yaw가 `arc_yaw`다. unit이 원호를 모르는 에이전트에서 끈다.
- `Driver(io, cfg, mount, on_event=None)`: 정지, 기다림, 우회, 건너뜀, 막힘을 일어날 때 `on_event(dict)`로 알린다. `set_fix(pose, odom_at_capture)`, `pose()`, `enter() -> Leg`, `go_to_view(k) -> Leg`, `correct(k) -> Leg`. `Leg(view: int, skipped: list[int], stops: int, waited_s: float)`. `enter()`는 `view = 0`

## 동작

1. **위치**: `pose() = fix ⊕ (odom_fix⁻¹ ⊕ odom_now)`(평면 강체 변환 합성). `odom_fix`는 그 스냅샷 머리말의 odom이다. station이 표식 자세가 있는 스냅샷마다 `set_fix`를 부른다.
2. **진입** `enter()`: 지금 카메라 각 `phi0`에서 `view_pose(phi0, R)`로 간다. 시점 k의 각은 `phi0 + k·360/n_views`다.
3. **시점 이동** `go_to_view(k)`, 원호(`Driver.arc` 참): `plan_orbit(pose(), 시점 k의 자세)`의 문장을 보낸다. 원호는 직진과 같이 보호한다(4번). 원호가 멈추면 `pose()`에서 다시 계획하고, 막힌 채 `wait_s`가 지나면 5번의 우회로 간다. 진입·거리 보정·우회는 `turn`·`forward`로 가되 `arc_yaw` 방향으로 선다.
   **시점 이동**, 회전 + 직진(`Driver.arc` 거짓): 진행 방향 h = `pose()`에서 `view_pose(phi_k, R)` 쪽 각. `plan_move(pose(), view_pose(phi_k, R, yaw=h), final_turn=False)`의 문장을 하나씩 보낸다. 마지막 회전을 하지 않아 시점마다 회전 한 번이 줄고, 도착 yaw는 접선과 시점 간격의 절반(7.5°)쯤 다르며 카메라는 원 위에 온다. 회전은 그대로 보내고, 직진은 아래 보호 직진으로 보낸다. 직진이 멈추면 `pose()`에서 다시 계획한다. 진입, 거리 보정, 우회 뒤 복귀는 마지막 회전으로 접선을 맞춘다.
4. **보호 직진**(`drive.obstacle`이 꺼져 있으면 확인 없이 보냄): 회전을 마친 뒤 `front_min ≥ start_clear`일 때 보낸다. 이동 중 `watch`는 `front_min < stop_dist`(스캔 끊김 포함)이면 true다. 막히면 0.2 s마다 다시 보며 `wait_s`까지 기다린다.
5. **우회**(시점 이동의 직진이 `wait_s`가 지나도 막혀 있을 때):
   1. 바깥 차선(R + `bypass_lane`)의 지금 각 지점으로 간다: `plan_move(pose(), view_pose(phi_now, R + lane))`
   2. 바깥 원을 따라 시점 j = k, k+1, …의 각으로 간다(보호 직진, 막히면 `say` 후 기다림).
   3. 각 j에서 `plan_move(pose(), view_pose(phi_j, R))`로 돌아간다. 돌아가는 직진의 출발 전 확인이 막혀 있거나 돌아가는 도중 멈추면, 기다리지 않고 바깥 차선 `view_pose(phi_j, R + lane)`로 되돌아가 j + 1로 한 칸 더 간다. 돌아오면 `Leg(view=j, skipped=[k…j−1])`
   4. j가 `n_views − 1`을 넘으면 바깥 차선에서 멈추고 `Leg(view=n_views, skipped=[k…n_views−1])`
6. **거리 보정** `correct(k)`: `view_pose(phi_k, R)`로 한 번 간다(우회 없음).
7. **원 위의 각으로** `go_to_phi(phi)`(보강 촬영, `docs/specs/webui.md`): 카메라가 원 위의 각 `phi`에 오는 자세로 한 번 간다(우회 없음, 막히면 `block_timeout_s` 뒤 `Blocked`). `on_orbit()`(원호 모드이고 원에서 5 cm, 접선에서 20° 안)이고 `phi`가 반시계 쪽이면 원호 하나라 180° 안쪽까지 되고, 그 밖은 회전 + 직진이라 15° 안쪽만 쓴다(현이 원 안을 지나지 않게). `angle_deg()`는 지금 카메라 각.
8. **기다림 한도**: 진입·거리 보정·바깥 차선에서 막힌 상태나 스캔 끊김이 `block_timeout_s`를 넘으면 `Blocked` 예외. `io.cancelled()`가 true면 다음 문장을 보내지 않고 `Cancelled` 예외
9. 시점 위치 오차: `command_driver`가 단계마다 1 cm·2° 남기고 멈추므로 한 구간 끝에서 1 ~ 2 cm·2 ~ 4°가 남을 수 있다. 시점마다 표식 자세로 다시 잡으므로 쌓이지 않는다.

## 설정

`drive` 전부, `scene.R`, `scene.r_obj`, `scene.place_err`, `scene.n_views`, `lidar.mount_yaw_deg`, `lidar.mask`, `robots.<role>.mount`

## 완료 테스트(`sim2d.SimIO`, 시점마다 참값 ±2 mm·0.2° 자세로 `set_fix`. 이름에 `arc`가 없는 테스트는 `drive.arc` 끔)

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_base_from_cam` | 참값 자세의 카메라를 피치 5° 틀리게 놓음, L·H, 서로 다른 yaw | 로봇 xy 2 mm, yaw 0.5° 안 |
| `test_pose_compose` | L(+x 쪽)·H(−x 쪽)에서 odom이 로봇 앞으로 0.1 m | `pose()`가 판 좌표에서 로봇 진행 방향으로 0.1 m |
| `test_plan_move` | 시점 0 → 1, 이미 목표에 있음, 0.01 m 옆, `final_turn=False` | 모두 `parse` 통과, 0인 부분 없음, 이미 목표면 빈 목록, sim2d 실행 결과가 목표와 2.5 cm·5° 안, `final_turn=False`면 마지막 문장이 직진 |
| `test_full_circle` | 24시점, odom 잡음(배율 ±2%, yaw 2°/m) | 시점마다 카메라가 목표와 2.5 cm 안, yaw는 진입(시점 0)이 접선과 5°, 그 뒤 시점은 12.5° 안, `d`가 `d_min` ~ `d_max`, 시점 이동마다 회전 문장 1개 이하 |
| `test_front_min` | 정면 ±25° 안에 0.15 m 반사(가림 범위 `[[-5, 5]]`)와 0.40 m 반사(안 가림) | 0.40 m가 나옴(가린 0.15 m는 버림) |
| `test_mask_wrap` | `in_mask(angle_deg, [[170, -170]])` | 175°·−175°는 가림, 165°·0°는 안 가림 |
| `test_bypass` | 지름 0.10 m 상자를 로봇 원 위 시점 2·3 사이 가운데에 고정 | 출발 안 함 → `wait_s` → 우회 → 반지름 R(± 2.5 cm)로 복귀 → 24시점 마침, `skipped` 1 ~ 2개 |
| `test_stop_resume` | 시점 2로 가는 직진을 시작하고 1 s 뒤 `Sim.at`으로 정면 0.15 m에 상자를 놓고 2 s 뒤 치움 | `stop` 1번, 이어 가서 카메라가 목표와 2.5 cm, yaw 12.5° 안, `skipped` 없음 |
| `test_bypass_tail` | 시점 22에 도착한 뒤 시점 22·23 기준점 목표 사이 가운데에 0.02 m 상자, 시점 23 안쪽 복귀 경로에 0.05 m 상자(0.10 m 상자는 출발 여유 0.30 m 때문에 한 구간 앞에서 막힌다) | 바깥 차선에서 멈추고 `view = n_views`, `skipped = [23]` |
| `test_correct` | 시점 k에서 `d` 0.30이 되게 `set_fix` 후 `correct(k)`만 | `d` ≥ `d_min` |
| `test_stale_scan` | `drop_scans` | 직진을 보내지 않음, 이동 중이면 `stop`, `block_timeout_s` 뒤 `Blocked` |
| `test_cancel` | 기다리는 중 `cancelled()` true | `Cancelled`, 더 보내지 않음 |
| `test_arc_pose` | L·H `arc_yaw` | 카메라가 원 위 그 각에 있고 바퀴 축의 연장선이 판 중심을 지남, 카메라 원의 접선과 `asin(0.08 / 0.62)` 차 |
| `test_plan_orbit` | 원 위 시점 0 → 1, 1 cm 바깥·안쪽, 4° 오른쪽·왼쪽으로 틀어진 출발, 원에서 10% 벗어남, 25° 틀어짐, 목표가 뒤쪽 | 원호 하나(반지름 `√(R² − 0.08²)`, 15°)를 sim2d로 실행하면 목표와 5 mm·0.5° 안. 바깥·오른쪽은 더 휘고 안쪽·왼쪽은 덜 휘며 원호 길이는 같음. 1 cm 바깥 출발은 6 ~ 9.5 mm가 남고 방향이 안쪽으로 듦. 크게 벗어났거나 뒤쪽이면 `plan_move`와 같음 |
| `test_orbit_fix_noise` | 원호, 시점마다 자세에 ±2 mm·0.2°와 ±10 mm·1°의 잡음 | 23번의 시점 이동이 모두 원호 문장 하나씩(회전 없음), 시점마다 2.5 cm·5° 안 |
| `test_full_circle_arc` | 24시점, odom 잡음, 원호 | 시점마다 2.5 cm·5° 안, `d` 범위 안, 시점 이동마다 원호 문장 1개이고 `forward` 없음, 7번째 시점부터 방향 2° 안, 회전 문장은 진입의 2개 이하 |
| `test_bypass_arc` | `test_bypass`와 같은 상자, 원호 | 기다림 → 우회 → 반지름 R로 복귀, `skipped` 1 ~ 2개 |
| `test_stop_resume_arc` | 원호를 시작하고 0.5 s 뒤 정면 0.17 m에 상자, 2 s 뒤 치움 | `stop` 1번, 이어 가서 목표와 2.5 cm·5° 안 |
