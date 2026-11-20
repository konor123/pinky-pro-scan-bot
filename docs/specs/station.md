# station

- 코드: `scan3d/station.py`(`python -m scan3d.station`), 현장에서는 `tools/scan.sh`가 부른다.
- 테스트: `tests/test_station.py`, I1 `tests/test_integration.py`(localhost 소켓만 쓴다)
- PLAN: 3.3(출발과 끝), 4.4(진행 순서), 5

## 하는 일

스캔 PC의 주 프로그램. 두 로봇(또는 한 대)과 연결해 출발, 24시점 맞춤 진행, 스냅샷, 원본 저장, 데이터셋, 재구성, 메시 표시까지 한 번에 한다.

## 인터페이스

- `python -m scan3d.station [--config config/scan.yaml] [--only L|H] [--fake [--obstacle <k>] [--time-scale N]] [--no-window]`
- `Clock(time_scale=1.0)`: `now()`, `sleep(s)`. 모든 시간(기다림, 0.1 s 감시 주기, 스캔 나이, 요청 한도)은 이 시계의 초다. `--fake`면 `FakeFleet`과 같은 시계를 쓴다.
- `RobotLink(role, host, port, stream_port, raw_writer, clock, cancel)`: drive의 `RobotIO` 구현
  - 스트림을 읽는 스레드가 마지막 odom, 마지막 scan(`decode_scan`, 받은 시각)을 들고 있고, 받은 줄과 보낸 `move`·`stop`을 모두 `stream.jsonl`에 쓴다.
  - `move`는 요청 한도(`timeout_s` + 5 s) 안에서 응답을 기다리며 0.1 s마다 `watch()`를 부르고, true면 스트림으로 `stop`을 한 번 보낸다.
  - `snapshot(k, use, phi_target)`: 요청에 `capture` 값을 싣고, 받은 것을 `raw_writer`에 쓴 뒤 `(header, payloads, NNNNNN)`을 돌려준다.
  - `abort()`: 스트림으로 `abort`를 보내고(로봇이 중단 상태가 되어 늦게 도착한 `move`도 거절) 중단 표시를 켠다. 이후 `move`는 보내지 않고 `Cancelled`. 기다리던 `recv`는 제어 소켓을 닫아 끊는다.
  - `cancelled()`: 공유 `cancel` 이벤트
- `run(cfg, links, writer, board) -> scan_dir`

## 동작

1. **준비**: `ScanWriter`로 `scans/<scan_id>/`를 만든다. `robots.<role>.host`를 `ssh -G <host>`의 `hostname`으로 바꾼다(스캔 PC `~/.ssh/config`의 `pinky1`·`pinky2` 별칭). `--fake`면 `FakeFleet`을 같은 프로세스에 띄워 그 포트를 쓴다. 각 로봇 `hello`(config `capture` 프로파일, → `calib.json`), 스트림 연결. 응답의 `role`이 config의 역할과 같고 두 로봇의 `serial`이 다른지 확인하고, 아니면 종료 코드 1(움직이지 않음)
2. **출발 위치**: L, H 차례로 `snapshot(use=start)` → `align.prepare` → `tagpose.estimate` → `drive.base_from_cam` → `set_fix`. 표식이 안 보이면 어느 로봇인지 출력하고 종료 코드 1(움직이지 않음)
3. **각도 차**(두 대일 때): `|wrap(phi_H − phi_L)|`가 180 ± `pair_gap_tol_deg` 밖이면 어느 로봇을 몇 도 옮길지 출력하고 종료 코드 1(움직이지 않음). 받아들인 각도 차는 한 바퀴 내내 유지된다.
4. **진입**: 두 로봇 `enter()`를 동시에(로봇마다 스레드) 실행한다. 끝나면 두 로봇 모두 시점 0이다.
5. **시점 k = 0…n_views−1**
   1. 지금 시점이 k보다 작은 로봇만 `go_to_view(k)`를 동시에 실행한다. 우회로 시점을 건너뛴 로봇은 앞선 시점에서 기다린다.
   2. `settle_s` 기다린다.
   3. 지금 시점이 k인 로봇을 L, H 순서로 하나씩: `snapshot(k, view, phi_k)` → 자세 → `set_fix`(odom은 스냅샷 머리말 값) → `d`. 자세가 없으면 `drive.pose()`로 `d`를 낸다. `d`가 `d_min`~`d_max` 밖이면 그 로봇만 `correct(k)` → `settle_s` → `snapshot(k, retake, phi_k)` 한 번 → 자세가 있으면 `set_fix` → `d`. 그 로봇이 끝난 뒤 다음 로봇을 찍는다.
   4. 구간 결과, 정지, 우회, 건너뜀, 거리 보정(전후 `d`와 그 출처: 표식 또는 odom)을 일어날 때마다 `events.jsonl`에 쓴다. drive의 정지·기다림·우회 결정은 `Driver(on_event=…)`로 받아 그때 쓴다.
6. **끝**: 두 로봇은 마지막 시점에서 멈춘다. 역할마다 `dataset.build` → `recon.reconstruct`(어느 역할이든 융합한 시점이 없으면 실패) → `out/` → `report.json`(시작부터 `out/`을 쓴 때까지 걸린 시간, `report.txt`와 같은 값) → 치수와 걸린 시간 출력 → `station.show_mesh`이고 `--no-window`가 아니면 Open3D 창
7. **실패**: 어느 스레드에서든 예외(`ok:false`, 연결 끊김, 요청 한도 초과, `Blocked`)가 나거나 Ctrl-C면 바로 공유 `cancel`을 켜고 연결된 모든 로봇에 `abort`를 보낸다. 스레드의 예외는 끝나는 대로 바로 확인한다(다른 스레드가 끝나기를 기다리지 않는다). 다른 스레드는 다음 `move` 전이나 기다리는 중에 `Cancelled`로 끝난다(최대 2 s 기다림). 원인을 `report.json`의 `aborted`와 화면에 남기고 종료 코드 1. 받은 `raw/`는 남는다.

## 설정

`config/scan.yaml` 전부(파일을 읽는 곳)

## 완료 테스트(`FakeFleet`, `time_scale` 5)

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_sequence` (slow) | 두 대, 시작 각 0°·180° | 로봇마다 `hello`가 먼저, 스냅샷 시간 구간이 서로 겹치지 않음, 시점마다 L·H 각도 차 180 ± 10°, 로봇마다 시점 스냅샷 24개 |
| `test_gap_kept` (slow) | 시작 각 0°·160° | 시점마다 각도 차가 시작 각도 차 ± 10° |
| `test_bypass_waits` (slow) | `--obstacle 2`(L의 시점 2·3 기준점 목표 사이 가운데에 한 변 0.10 m 정사각 상자) | H가 시점 k + 1 문장을 받은 시각이 L이 시점 k 이상에 도착한 시각보다 늦음(모든 k), L의 `events.jsonl`에 기다림과 우회가 있음, 두 로봇 모두 끝까지 감, 건너뛴 시점이 로봇마다 1~2개(L은 시점 3 근처, H는 12시점 뒤 같은 상자 근처)이고 나머지 시점은 모두 찍힘, `report.json`과 `events.jsonl`이 같음 |
| `test_pair_gap` | 시작 각 0°·120° | 종료 코드 1, 어느 로봇도 `move`를 받지 않음 |
| `test_no_tags_start` | L을 바깥쪽을 보게 시작(`starts`의 yaw 180°) | 종료 코드 1, `move` 없음 |
| `test_role_serial` | 두 연결의 `role`이 바뀜, `serial`이 같음 | 각각 종료 코드 1, `move` 없음 |
| `test_failure_stops` | `fail`로 H의 시점 3 이동 문장에 `ok:false`(L이 움직이는 중에 응답) | L이 이동 중이든 기다리는 중이든 2 s 안에 끝남, 두 로봇이 `abort`를 받음, 그 뒤 움직임 없음, 종료 코드 1, `raw/`와 `report.json`(`aborted`) 남음 |
| `test_only` | `--only L` | 한 대로 24시점 |
| `test_retake_next_leg` | 시점 5에서 `d` 0.30이 되게 시작 자세를 틀어 놓음. (가) retake에 표식 있음 (나) retake에 표식 없음 | (가) retake 자세로 `set_fix`, (나) odom으로 이어 감. 두 경우 모두 다음 구간 뒤 시점 6 카메라가 목표와 2.5 cm·5° 안 |
| I1 `test_integration` (slow) | `--fake`, `--fake --obstacle 2` | `out/mesh.ply` 생성, 상자 치수 10 mm 안, 면마다 색이 다른 상자의 빨강 면 영역 꼭짓점 색이 빨강(RGB 순서), 역할마다 융합한 k가 시점별 결과(`outcomes.json`)와 같음, `report.txt`에 걸린 시간 |
