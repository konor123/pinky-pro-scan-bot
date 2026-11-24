# fake_agent

- 코드: `scan3d/sim/fake_agent.py`
- 테스트: `tests/test_fake_agent.py`
- PLAN: 4.2, 4.4, 5(I1)

## 하는 일

개발 PC에서 robot_agent 대신 뜨는 TCP 서버(localhost). 같은 메시지(protocol)로 응답하되, 움직임은 sim2d, 스냅샷은 synth가 만든다. 두 로봇이 한 sim2d 세계를 같이 쓴다.

## 인터페이스

- `FakeFleet(cfg, clock, mesh=None, board=None, roles=("L", "H"), starts=None, obstacle=None, fail=None, rng_seed=0, snapshot_s=0.0)`
  - `mesh` 기본은 `synth.box_mesh`(config `scene` 크기 안), `board` 기본은 `config/board.json`
  - `starts`: `{"L": phi_deg 또는 (phi_deg, yaw_offset_deg), …}`. 기본 `{"L": 0, "H": 180}`. 카메라가 반지름 R + 0.08 m, 각 phi에 오고 로봇 yaw가 반시계 접선 + `yaw_offset`(기본 0) ± 5°(`rng_seed`). yaw_offset 180이면 카메라가 바깥을 본다.
  - `obstacle`: `(k, size_m=0.10)`. L의 시점 k와 k + 1 로봇 기준점 목표(`drive.view_pose`) 사이 가운데에 놓는 정사각 상자(변이 진행 방향과 나란함)
  - `fail`: `{"role": "H", "cmd": "move", "nth": 3, "after": "L_moving"}`이면 그 요청에 `ok:false`. `after`가 있으면 그 조건(`L_moving`: L의 sim2d 로봇이 움직이는 중, `L_waiting`: L이 기다리는 중)이 될 때까지 응답을 미룬다
  - `after` 조건은 최대 30 s(`clock`) 기다리고, 그때 조건이 맞았는지를 `fail_log`(역할, 시각, 다른 로봇이 움직이던 중인지)에 남긴다.
  - `snapshot_s`: 스냅샷 하나에 걸리는 `clock` 초(실물은 약 3.3 s). 렌더링 뒤 이만큼 기다리고 응답한다. 기본 0
  - `serials`(역할별 일련번호, 같은 값을 주면 일련번호 확인 시험), `ports`(역할별 (제어, 스트림) 포트, 기본 0)
  - 기록: `hello_log`, `move_log`, `stop_log`(stop·abort), `snapshot_log`(역할, 실제 시작·끝 시각(`snapshot_s` 기다림 포함), `clock` 시각), `arrive_log`(시점 목표에 도착한 시각), `fail_log`. 조회: `true_pose(role)`, `view_target(role, k)`, `world_boxes()`
- `start() -> {"L": (host, ctrl_port, stream_port), …}`: 포트 0으로 열어 실제 포트를 돌려준다. `stop()`
- `python -m scan3d.fake_agent [--obstacle 2] [--time-scale 5]`: config의 포트로 띄운다.

## 동작

1. sim2d는 별도 스레드에서 `clock`에 맞춰 진행한다(`clock`의 1 s = sim2d 1 s). 스냅샷을 렌더링하는 동안은 `clock.hold()`로 시계를 멈춘다(렌더링 시간이 주행 시간을 먹지 않게, 실물에서 스냅샷 동안 로봇이 서 있는 것과 같다).
2. `hello`: `role`, `serial` `"FAKE-<role>"`, `calib` = `synth.default_calib()`(요청 프로파일 크기). 중단 상태를 푼다.
3. `move`: 문장을 sim2d 로봇에 넣고 `(모든 명령 완료)`면 `stopped:false`, `stopped:`면 `stopped:true`, `error:`면 실패, `timeout_s`가 지나면 sim2d에 `stop`을 넣고 실패
4. `snapshot`: 요청의 해상도로, 로봇 참값 자세와 `mount`(높이, 피치 포함)로 구한 컬러 카메라 자세를 synth로 렌더링한다. 깊이는 `n_depth`장을 서로 다른 잡음(σ 2 mm at 1 m)으로 만든다. 형식은 `yuyv`·`z16`, `compress`면 zlib. `t`·`odom`은 렌더링을 시작할 때의 값. `ir` 요청은 `ok:false`(실물에서만 쓰는 기능)
5. 스트림: odom 20 Hz, scan 10 Hz(`clock` 기준), 상태 문자열, `stop`을 받으면 sim2d에 `stop`, `abort`를 받으면 robot_agent와 같이 sim2d에 `stop`을 넣고 진행 중인 `move`를 `aborted`로 실패시키며, 다음 `hello`까지 모든 `move`를 거절
6. 제어 연결이 끊겨도 sim2d 로봇은 남은 문장을 마친다.

## 설정

`robots.<role>.port`·`stream_port`·`mount`, `scene`, `capture`

## 완료 테스트(`clock` `time_scale` 5)

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_hello_snapshot` | protocol 클라이언트로 `hello`, `snapshot` | 머리말과 바이트 크기가 실물과 같음(컬러 1,843,200 + 깊이 814,080 × `n_depth`) |
| `test_move_stop` | `move("speed 0.06; forward 0.3")` 중 스트림으로 `stop` | 응답 `stopped:true`, `stop_log`에 기록 |
| `test_streams` | 스트림 연결 `clock` 2 s | odom 32 ~ 48개, scan 16 ~ 24개 |
| `test_snapshot_pose` | 스냅샷 → align → tagpose | 참값 자세와 3 mm·1° 안 |
| `test_fail` | `fail` 설정 | 그 요청만 `ok:false` |
| `test_abort` | 이동 중 스트림 `abort` | 진행 중 `move`와 다음 `move`가 `aborted`, 새 `hello` 뒤에는 받음 |
| `test_obstacle_position` | `obstacle=(2, 0.10)` | 상자가 L 시점 2·3 로봇 목표 가운데, 변이 진행 방향과 나란함 |
