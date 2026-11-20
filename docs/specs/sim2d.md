# sim2d

- 코드: `scan3d/sim2d.py`
- 테스트: `tests/test_sim2d.py`
- PLAN: 3.3, 5
- 출처 예제: pinky_sim_examples `1d14183` `command_driver.py`(동작), `scan3d/vendor/pinky_commands.py`(`parse`)

## 하는 일

`command_driver`처럼 이동 문장을 실행하는 2D 로봇과, 그 로봇이 보는 2D LiDAR를 흉내 낸다. drive, 가짜 에이전트, station 테스트의 입력이다. 시간은 `step(dt)`로만 흐른다.

## 인터페이스

- `World(circles=[(x, y, r)], boxes=[(x, y, w, d, yaw_deg)])`: 판 좌표 2D 장애물. 물체는 원, 장애물 상자는 회전한 직사각형. `add_box`, `remove`로 실행 중에 바꾼다.
- `Robot(world, pose=(x, y, yaw), name, linear_speed=0.10, odom_noise=None, rng=None)`: 반지름 0.07 m 원으로 다른 로봇의 LiDAR에 보인다.
- `Robot.command(text)`: `/pinky_command`와 같다. `Robot.statuses()`: 그동안 나온 `/command_status` 문자열을 꺼낸다.
- `Robot.step(dt)`: dt만큼 움직인다. `Robot.busy()`: 실행 중이거나 남은 문장이 있으면 True
- `Robot.odom() -> (x, y, yaw)`: odom 좌표(출발 자세가 원점, 잡음 포함). `Robot.true_pose()`: 판 좌표 참값
- `Robot.scan() -> dict`: `{"angle_min": -π, "angle_inc": 2π/720, "range_min": 0.05, "range_max": 12.0, "ranges": [m…]}`. LiDAR 좌표(장착 yaw π, 로봇 중심에서 x −0.017 m, 각도 0이 로봇 뒤쪽), 맞는 것이 없으면 `inf`
- `Sim(world, robots, dt=0.05)`: 여러 로봇을 함께 진행. `run_until(pred, timeout)`, `now`, `at(t, fn)`(시뮬레이션 시각 t에 `fn()`을 부름. 장애물을 놓거나 치우는 테스트용)
- `SimIO(sim, robot, scan_hz=10)`: drive의 `RobotIO` 구현. `move`는 `(모든 명령 완료)`·`stopped:`·`error:`가 나올 때까지 `sim`을 진행하며 0.1 s(시뮬레이션 시간)마다 `watch()`를 부른다. `sleep`도 `sim`을 진행한다. `scan()`의 `age_s`는 마지막 스캔 뒤 흐른 시뮬레이션 시간이다. `drop_scans`(true면 스캔을 더 내지 않음), `cancel()`(이후 `cancelled()` true), `move_log`(보낸 문장과 시각). `move`는 `timeout_s`(기본 60, 시뮬레이션 시간)가 지나면 `stop`을 넣고 실패를 돌려준다(장애물에 걸려 문장이 안 끝나는 경우)

## 동작

1. `command`는 `parse`로 해석한다. 틀리면 `error: …`, `stop`이면 남은 명령을 지우고 `stopped: 남은 명령 취소`, 아니면 큐에 넣고 `accepted: <문장> (대기 n단계)`
2. `speed`는 직진 속도를 바꾸고 `speed: x.xx m/s`
3. `command_driver.control()`을 20 Hz로 그대로 따른다. 직진은 남은 거리 × 1.5로 줄이며(최저 0.03 m/s) 남은 거리 1 cm 이하면 끝, 회전은 `max(0.15, min(0.8, 2 × |남은 각|))` rad/s에 남은 각의 부호로 돌고 2° 이하면 끝. 그래서 단계마다 약 1 cm·2° 덜 간다. 단계가 끝날 때마다 `done: move +0.17 m`·`done: turn +8 deg`, 큐가 비면 끝에 ` (모든 명령 완료)`. `speed`만 있는 문장은 `speed:` 뒤에 아무것도 내지 않는다(실물과 같음).
4. 로봇은 장애물을 뚫고 지나가지 않는다. 다음 위치가 장애물과 겹치면 그 자리에 선다(문장은 끝나지 않는다).
5. odom 잡음: `odom_noise={"scale": 0.02, "yaw_drift_deg_per_m": 2.0}`이면 이동 거리 배율과 직진 중 yaw 누적 오차를 더한다.

## 설정

없음(인자로 받는다).

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_command_status` | `"speed 0.06; turn 7.5; forward 0.17; turn 7.5"` | 상태 문자열 순서와 형식이 `command_driver`와 같음, 마지막에만 `(모든 명령 완료)` |
| `test_motion` | `"turn 7.5"`, `"turn -30"`, `"speed 0.06; forward 0.17"`, 잡음 없음 | 단계마다 남은 양이 0~1 cm·0~2°에서 멈춤(덜 감), 음수 회전은 시계 방향 |
| `test_speed_only` | `"speed 0.06"` | `accepted: …`, `speed: 0.06 m/s` 두 줄만 나오고 `done`은 없음 |
| `test_stop` | 직진 중 `stop` | 0.05 s 안에 멈춤, 남은 명령 없음 |
| `test_bad_text` | `"turn 0"` | `error:`로 시작, 움직이지 않음 |
| `test_scan_front` | 앞면이 로봇 중심에서 정면 0.5 m인 상자 | `sector_min(ranges, angle_min, angle_inc, 0, 25°, mount_yaw=π)`가 0.517 ± 0.01 m(LiDAR가 중심 뒤 0.017 m) |
| `test_other_robot` | 두 로봇이 마주 봄 | 서로의 스캔에 보임 |
