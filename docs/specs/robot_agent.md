# robot_agent

- 코드: `ros/scan3d_robot/`(ament_python): `scan3d_robot/agent.py`(ROS·카메라를 import하지 않는 본체), `scan3d_robot/camera.py`(pyrealsense2), `scan3d_robot/node.py`(rclpy 연결과 실행), `launch/robot_scan.launch.xml`
- 테스트: `ros/scan3d_robot/test/test_agent.py`(개발 PC, ROS·카메라 없이). 저장소 pytest 설정의 `testpaths`·`pythonpath`에 이 패키지가 들어 있어 `pytest -q` 하나로 함께 돈다.
- PLAN: 4.2, 4.3, 4.4, 7
- 출처 예제: librealsense v2.59.2 `wrappers/python/examples/align-depth2color.py`(파이프라인·프레임·깊이 배율), pinky_sim_examples `1d14183` `command_driver.py`(토픽과 상태 문자열)

## 하는 일

로봇에서 도는 TCP 서버. 스캔 PC의 요청대로 `command_driver`에 이동 문장을 보내고, 카메라 스냅샷을 원본 그대로 보내고, odom·LiDAR·상태 문자열을 그대로 흘려보낸다. 값을 가공하지 않는다.

## 인터페이스

- `Agent(role, camera, bus, port=7010, stream_port=7011)`: `serve_forever()`, `shutdown()`
- `camera`(`camera.py`가 pyrealsense2로 구현): `serial() -> str`, `calibration(color_profile, depth_profile) -> dict`(protocol `calib`, `model`은 `intrinsics.model.name`), `snapshot(color_profile, depth_profile, n_depth, n_skip) -> (t, color_hdr, color_bytes, depth_hdr, depth_bytes)`, `snapshot_ir(depth_profile, n_skip) -> (t, ir_hdr, ir_bytes)`. 프로파일은 `[폭, 높이, fps]`
- `bus`(node.py가 rclpy로 구현): `publish_command(text)`, `publish_twist(linear, angular)`(`/cmd_vel`), `command_subscribers() -> int`, `next_status(timeout) -> str | None`(move 대기용 큐, 스트림 전송과 따로), `odom() -> (t, x, y, yaw)`, 콜백 `on_odom(fn)`, `on_scan(fn)`(LaserScan의 `angle_min`, `angle_increment`, `range_min`, `range_max`, `ranges`), `on_status(fn)`. 콜백의 `t`는 로봇 시계로 받은 시각
- 로봇 실행: `robot_scan.launch.xml role:=L` = `pinky_auto` robot 런치 + `command_driver`(`stop_distance:=0.0`) + `ExecuteProcess`(`~/scan3d_venv/bin/python -m scan3d_robot.node --role L`, `PYTHONPATH`에 `~/scan3d_lib`)
- `scripts/deploy.sh`가 `scan3d/__init__.py`, `scan3d/protocol.py`를 로봇 `~/scan3d_lib/scan3d/`에 복사한다. `scripts/setup_robot.sh`가 `~/scan3d_venv`(`--system-site-packages`)에 `pyrealsense2`를 넣는다.

## 동작

1. 제어 연결은 한 번에 하나만 받고 요청을 차례로 처리한다.
2. `hello`: 요청의 프로파일을 이 연결의 스냅샷 프로파일로 정하고, `serial`과 `camera.calibration(프로파일)`(파이프라인을 열어 읽고 닫는다), `"arc": true`(원호 문장을 안다)를 돌려준다. 중단 상태를 푼다.
3. `move`: 상태 대기열을 비우고, `/pinky_command` 구독자가 생길 때까지(최대 5 s) 기다린 뒤 문장을 보낸다. 문장 발행과 `abort` 처리는 같은 잠금 안에서 하고, 발행 직전에 중단 상태면 보내지 않고 `aborted`로 실패한다. `done: … (모든 명령 완료)`면 `ok`, `stopped:`면 `ok`·`stopped:true`, `blocked:`·`error:`면 실패, `timeout_s`가 지나면 `stop`을 보내고 실패. 응답의 `odom`은 그때 마지막 odom
   `move`의 원호 문장(`protocol.parse_arc`가 받는 `[speed s;] arc <반지름> <각도>`)은 `/pinky_command`로 보내지 않고 직접 실행한다. `cmd_vel`에 `linear.x = v`, `angular.z = ±v / 반지름`(양수 각도는 왼쪽)을 20 Hz로 보내며, `v`는 `min(속도, 1.5 × 남은 거리)`이고 최저 0.03 m/s다. odom 위치 변화를 더한 이동 거리가 `반지름 × |각도|`에서 3 mm 안으로 남으면 0을 보내고 `ok`. 스트림의 `stop`이면 0을 보내고 `stopped:true`, `abort`면 `aborted`, `timeout_s`가 지나면 0을 보내고 실패. `speed`가 없으면 0.10 m/s. 값이 틀린 원호 문장은 움직이지 않고 `error: …`로 실패한다.
4. `snapshot`: 요청의 프로파일로 파이프라인 시작(컬러 YUY2, 깊이 Z16) → `n_skip`세트 버림 → 다음 세트의 컬러 1장과 깊이, 이어지는 세트의 깊이로 `n_depth`장 → 정지. 프레임 바이트는 그대로 보낸다. `compress`면 `zlib`. `t`는 남긴 첫 세트를 받은 순간의 로봇 시계, `odom`은 9번
5. `snapshot(ir=true)`: 이 카메라의 투사기를 끄고 왼쪽 적외선(Y8) 1장을 찍은 뒤 투사기를 원래대로 켠다.
6. `abort`(스트림): 잠금 안에서 중단 상태를 켜고 `/pinky_command`에 `stop`을 보낸다. 진행 중인 `move`는 `aborted`로 끝난다. 다음 `hello`에서 풀린다.
7. 스트림 연결: odom은 최대 20 Hz로 솎아서, scan은 올 때마다 `encode_scan`(`angle_inc` = `angle_increment`), `/command_status`는 `status`로 보낸다. `{"type":"stop"}`을 받으면 `/pinky_command`에 `stop`
8. 제어 연결이 끊겨도 `stop`을 보내지 않는다(실행 중인 문장 하나를 마치고 선다).
9. 스냅샷 머리말의 `odom`은 bus가 들고 있는 최근 2 s odom 기록에서 첫 세트 시각 `t`에 가장 가까운 값이다.

## 설정

없음(launch 인자 `role`, 포트). 스캔 PC의 `capture` 값은 요청에 실려 온다.

## 완료 테스트(가짜 camera·bus, localhost TCP, protocol 클라이언트)

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_hello` | `hello` | `role`, `serial`, `calib`의 키가 protocol과 같음 |
| `test_camera_pipeline` | 가짜 `rs.pipeline`(camera.py에 주입) | 요청 프로파일로 시작, `n_skip`세트 버린 뒤 첫 세트의 컬러 + 깊이 `n_depth`장, 끝나면 정지, IR이면 투사기 끔 → 원래대로, `model`이 `.name` |
| `test_move_paths` | 상태 대본: 완료 / `stopped:` / `blocked:` / `error:` / 무응답 | 각각 ok / stopped / 실패 / 실패 / `timeout_s` 뒤 실패와 `stop` 발행 |
| `test_waits_subscriber` | 구독자 0 → 1 s 뒤 1 | 구독자가 생긴 뒤에 발행 |
| `test_snapshot_bytes` | 가짜 카메라 | 머리말과 바이트 1,843,200 + 4,070,400, `compress`면 풀어서 같음 |
| `test_stream` | odom 100 Hz, scan(`inf` 포함), 상태 문자열, `stop` 요청 | odom 20 Hz 이하, `ranges_mm`의 `inf`는 0, 상태 그대로, `stop` 발행 |
| `test_disconnect` | `move` 중 제어 연결 끊음 | `stop`을 발행하지 않음, 다음 연결을 받음 |
| `test_abort_order` | `abort`가 (가) 구독자를 기다리는 `move`보다 먼저 (나) 이동 중 (다) 완료 뒤에 옴 | (가) 문장을 발행하지 않음 (나) `stop` 발행, `aborted` 실패 (다) 다음 `move` 거절. 새 `hello` 뒤에는 받음 |
| `test_profile_mismatch` | `hello`와 다른 프로파일의 스냅샷 요청 | `error: profile` |
| `test_snapshot_odom` | 스냅샷 중 odom이 바뀜 | 머리말 `odom`이 첫 세트 시각의 값 |
| `test_arc` | `cmd_vel`대로 odom을 적분하는 가짜 bus, `speed 0.10; arc 0.62 15`와 `−15` | `hello`에 `arc`, `/pinky_command` 발행 없음, 회전/직진 비 ±1/0.62, 최고 0.10 m/s, 이동 거리 0.16 m에서 0을 보내고 `ok`, 반지름 0.62 m 원 위 |
| `test_arc_stop_abort_timeout_bad` | 원호 중 `stop`, `abort`, 짧은 `timeout_s`, `turn 5; arc 0.62 15` | 각각 `stopped:true`, `aborted`, `timeout`, `error:`이고 모두 마지막 `cmd_vel`이 0 |
