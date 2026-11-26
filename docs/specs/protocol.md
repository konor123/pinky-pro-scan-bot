# protocol

- 코드: `scan3d/protocol.py`(표준 라이브러리만 쓴다. 로봇의 robot_agent도 이 파일을 쓴다)
- 테스트: `tests/test_protocol.py`
- PLAN: 4.4

## 하는 일

스캔 PC와 로봇 사이 TCP 메시지의 형식과 주고받기. 메시지는 UTF-8 JSON 한 줄(`\n`)이고, 영상이 있으면 그 줄 바로 뒤에 원본 바이트가 이어진다.

## 메시지

**제어 연결(TCP 7010, 요청 하나에 응답 하나)**

| 요청 | 응답 |
|---|---|
| `{"cmd":"hello","color":[1280,720,15],"depth":[848,480,15]}` | `{"ok":true,"role":"L","serial":"…","calib":{…}}`. 이 연결의 스냅샷 프로파일이 여기서 정해지고, `calib`은 이 프로파일의 값이다. 로봇의 중단 상태를 푼다 |
| `{"cmd":"move","text":"speed 0.10; forward 0.17","timeout_s":60}` | `{"ok":true,"stopped":false,"odom":[x,y,yaw]}`(odom 좌표). `stop`으로 끝나면 `"stopped":true` |
| `{"cmd":"snapshot","n_depth":5,"n_skip":5,"compress":false,"color":[1280,720,15],"depth":[848,480,15]}` | `{"ok":true,"t":…,"odom":[…],"color":{"w":1280,"h":720,"fmt":"yuyv","bytes":1843200},"depth":{"w":848,"h":480,"fmt":"z16","n":5,"bytes":4070400}}` + 컬러 바이트 + 깊이 바이트 |
| `{"cmd":"snapshot","ir":true,"n_skip":5,"depth":[848,480,15]}` | `{"ok":true,"t":…,"odom":[…],"ir":{"w":848,"h":480,"fmt":"y8","bytes":407040}}` + 바이트. `t`·`odom`은 그 적외선 프레임을 받은 순간 |

- 실패 응답: `{"ok":false,"error":"…"}`. `move`에서 `blocked: …`·`error: …`가 오거나 `timeout_s`가 지나도 실패다.
- `move`의 `text`는 drive `plan_move`·`plan_orbit`이 만든 문장 하나(회전 하나, `speed s; forward c` 하나, 또는 `speed s; arc r θ` 하나)다. 움직임이 없는 문장(`speed`만)은 보내지 않는다.
- `parse_arc(text) -> (속도 또는 None, 반지름 m, 각도 °) | None`: 원호 문장 `[speed s;] arc <반지름> <각도>`를 읽는다(양수 각도는 왼쪽으로 도는 반시계). 원호 문장이 아니면 None. 다른 문장과 섞였거나 속도 0.02 ~ 0.25 m/s·반지름 0.05 m 이상·각도 0이 아닌 360° 이하·길이 5 m 이하를 벗어나면 ValueError. 로봇 에이전트, sim2d, 스캔 PC가 같이 쓴다.
- `hello` 응답의 `arc`(true): 원호 문장을 아는 에이전트. 없으면 스캔 PC가 원호 문장을 보내지 않는다.
- 스냅샷의 `t`(로봇 시계, 초)와 `odom`은 남긴 첫 컬러 프레임을 받은 순간의 값이다. `hello`와 스냅샷 요청의 `color`·`depth`는 `[폭, 높이, fps]`이고 config `capture`에서 온다. 스냅샷 요청의 프로파일이 `hello`와 다르면 `{"ok":false,"error":"profile"}`. 형식은 컬러 YUY2, 깊이 Z16, 적외선 Y8로 고정이다.
- `calib`: `{"color":{"w","h","fx","fy","ppx","ppy","model","coeffs":[5]},"depth":{…같은 키…},"depth_to_color":{"rotation":[9],"translation":[3]},"depth_scale":0.001}`. `model`은 librealsense `distortion.name`(`none`, `brown_conrady`, `inverse_brown_conrady`, `modified_brown_conrady`), `rotation`은 librealsense 순서(열 우선) 그대로, `translation`은 m. 픽셀 중심은 정수 좌표다(librealsense·OpenCV와 같음).
- 바이트 길이: 머리말의 `color`, `depth`, `ir` 중 있는 것의 `bytes`를 이 순서로 더한 만큼 뒤따른다.
- 압축: `compress`가 true면 로봇이 스트림마다 따로 `zlib`으로 압축하고, 해당 항목에 `"zlib":true`, `"raw_bytes":원래 길이`를 넣고 `bytes`는 압축한 길이다. 받는 쪽 `recv`가 풀고, 돌려주는 머리말에서는 `zlib`·`raw_bytes`를 지우고 `bytes`를 푼 길이로 바꾼다.
- 바이트 배열: 행 우선, 빈틈 없음, 리틀 엔디언. Z16 값 × `depth_scale` = m
- 형식 이름: 컬러 `yuyv`·`rgb8`, 깊이 `z16`(값 × `depth_scale` = m)·`f32`(m), 적외선 `y8`

**스트림 연결(TCP 7011, 양방향)**

| 방향 | 메시지 | 주기 |
|---|---|---|
| 로봇 → PC | `{"type":"odom","t":…,"x":…,"y":…,"yaw":…}` | 20 Hz |
| 로봇 → PC | `{"type":"scan","t":…,"angle_min":…,"angle_inc":…,"range_min":…,"range_max":…,"ranges_mm":[…]}` | LiDAR 주기(10 Hz) |
| 로봇 → PC | `{"type":"status","t":…,"text":"done: … (모든 명령 완료)"}` | `/command_status`가 올 때 |
| PC → 로봇 | `{"type":"stop"}`: 장애물 정지. 진행 중인 `move`가 `stopped:true`로 끝나고 다음 `move`는 받는다 | 필요할 때 |
| PC → 로봇 | `{"type":"abort"}`: 스캔 중단. 로봇이 `stop`을 내고 중단 상태가 되어, 다음 `hello`가 올 때까지 모든 `move`를 `{"ok":false,"error":"aborted"}`로 거절한다 | 실패할 때 |

- `t`는 로봇 시계(초)다. 스캔이 오래되었는지는 스캔 PC가 받은 시각으로 판단한다.
- `ranges_mm`: 정수 mm. 유한하지 않거나 `range_min`보다 작거나 `range_max`보다 큰 값은 0이다. 0은 측정값이 아니다.

## 인터페이스

- `Channel(sock)`: `send(obj, payloads=())`(여러 스레드에서 불러도 줄이 섞이지 않음), `recv(timeout) -> (obj, payloads)`(바이트 길이 규칙대로 읽고 `zlib`을 풂. 연결이 끊기면 `ConnectionError`, 시간이 지나면 `TimeoutError`)
- `connect(host, port, timeout=5.0) -> Channel`: 연결 시간 한도만 정한다. 요청별 한도는 호출하는 쪽이 `recv(timeout)`로 준다(hello 30 s, move `timeout_s` + 5 s, snapshot 30 s).
- `encode_scan(t, angle_min, angle_inc, range_min, range_max, ranges_m) -> dict`, `decode_scan(msg) -> dict`(`ranges`는 m, 0은 `inf`)
- `pack_snapshot(header, streams: dict[str, bytes], compress) -> (header, payloads)`

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_roundtrip` | 4.4의 모든 메시지, `socket.socketpair`, 한 바이트씩 나눠 보내기 | 받은 객체와 바이트가 보낸 것과 같음 |
| `test_snapshot_sizes` | 1280×720 YUY2 + 848×480 Z16 × 5 | 바이트 1,843,200 + 4,070,400 |
| `test_zlib` | `compress=true` | 풀린 바이트가 원본과 같음, 돌려준 머리말의 `bytes`가 푼 길이이고 `zlib`이 없음 |
| `test_timeout` | 응답 없는 서버, `recv(0.2)` | `TimeoutError` |
| `test_scan_codec` | `[0.5, inf, nan, 0.01, 20.0]`, `range_min` 0.05, `range_max` 12 | `ranges_mm` `[500, 0, 0, 0, 0]`, 되돌리면 `[0.5, inf, inf, inf, inf]` |
| `test_korean_text` | `status` 문장 `(모든 명령 완료)` | 그대로 받음 |
| `test_disconnect` | 줄 중간에 연결 끊김 | `ConnectionError` |
