# dataset

- 코드: `scan3d/dataset.py`
- 테스트: `tests/test_dataset.py`
- PLAN: 4.5
- 출처 예제: Open3D v0.20.0 `examples/python/reconstruction_system/sensors/realsense_recorder.py`(폴더 형식, 사본 `docs/reference/open3d-v0.20.0/reconstruction_system/sensors/`)

## 하는 일

로봇에서 받은 원본과 스캔 PC의 판단 기록을 `scans/<scan_id>/`에 쓰고, 그것만으로 Open3D 예제 형식의 데이터셋(컬러, 깊이, 내부 파라미터, 시점별 자세)을 다시 만든다.

## 폴더

```
scans/<scan_id>/config.yaml                  시작할 때의 config/scan.yaml 복사
scans/<scan_id>/board.json                   시작할 때의 config/board.json 복사
scans/<scan_id>/events.jsonl                 station·drive 판단 기록(구간 결과, 정지, 우회, 건너뜀, 거리 보정, 중단 원인)
scans/<scan_id>/report.json                  {"skipped": {"L": [k…], "H": [k…]}, "stops": {"L": n, "H": n}, "corrections": {"L": n, "H": n}, "aborted": null | "원인", "elapsed_s": …}
scans/<scan_id>/<role>/raw/calib.json         hello 응답 그대로
scans/<scan_id>/<role>/raw/NNNNNN.json        protocol이 돌려준 snapshot 머리말 + "station": {"k", "use", "phi_target"}
scans/<scan_id>/<role>/raw/NNNNNN_color.<fmt> 컬러 바이트(머리말 `bytes`만큼)
scans/<scan_id>/<role>/raw/NNNNNN_depth.<fmt> 깊이 바이트 n장 이어 붙임
scans/<scan_id>/<role>/raw/stream.jsonl       받은 odom·scan·status와 보낸 move·stop, 줄마다 "pc_t"(스캔 PC 시각)와 "dir"(in/out)
scans/<scan_id>/<role>/color/KKKKKK.jpg       왜곡 보정한 컬러, 품질 95
scans/<scan_id>/<role>/depth/KKKKKK.png       왜곡 보정, 컬러 정렬, uint16 mm
scans/<scan_id>/<role>/camera_intrinsic.json  {"width", "height", "intrinsic_matrix": K 열 우선 9개}
scans/<scan_id>/<role>/views.json             [{"k", "snap", "phi", "d", "T_cam_board", "n_tags", "reproj_px", "depth_invalid"}]
scans/<scan_id>/<role>/outcomes.json          시점별 결과(아래)
```

- `NNNNNN`은 받은 순서의 스냅샷 번호, `KKKKKK`는 시점 번호 k다.
- `use`: `start`(출발 전, `k`와 `phi_target`은 null), `view`(시점), `retake`(거리 보정 뒤 다시 찍음). `phi_target`은 도
- `<scan_id>`는 시작 시각 `YYYYMMDD-HHMMSS`
- `<role>/` 아래 `color/`, `depth/`, `camera_intrinsic.json`은 `realsense_recorder.py`가 만드는 형식(`%06d.jpg`, `%06d.png`, 같은 JSON 키)과 같다. 다른 점은 번호가 받은 순서가 아니라 시점 번호 k라는 것, 깊이가 중앙값·정렬·왜곡 보정을 거친 것, `views.json`에 시점별 자세가 따로 있다는 것이다. 그래서 Open3D 예제 `integrate.py`도 이 폴더를 읽을 수 있다.

## 인터페이스

- `ScanWriter(root, cfg_path, board_path) -> scan_dir`: 폴더를 만들고 config와 배치 파일을 복사한다. `role(role) -> RawWriter`, `event(dict)`, `report(dict)`
- `RawWriter`: `calib(hello)`, `snapshot(header, payloads, k, use, phi_target) -> NNNNNN`, `stream(msg, direction, pc_t)`. 줄마다 flush
- `build(scan_dir, role) -> list[view]`: `scan_dir`의 `config.yaml`, `board.json`, `events.jsonl`, `raw/`만 쓴다. 시작할 때 `color/`, `depth/`, `views.json`, `outcomes.json`을 지우고 새로 만든다. 시점 k마다 `use`가 `view`·`retake`인 스냅샷을 늦은 것부터 `align.prepare` → `tagpose.estimate`해서 처음으로 자세가 나온 것을 쓴다. 자세가 나온 것이 없으면 그 k는 views에 넣지 않는다.
- `depth_invalid`: 그 시점 자세로 품질 원기둥(반지름 r_eff, 높이 `obj_h`, recon 자르기 원기둥보다 작음)을 영상에 투영한 영역 안에서 깊이가 0인 픽셀 비율
- `outcomes.json`: 시점 k마다 `{"k", "captured", "pose": true|false, "d", "d_source": "tag"|"odom"|null, "reason": null|"bypass"|"no_tags", "crop": null|"ok"|"empty"|"no_block", "fused": null|true|false}`. `bypass`와 odom `d`는 `events.jsonl`(station이 씀)에서, `no_tags`와 표식 `d`는 build에서 정한다. `crop`과 `fused`는 recon이 채운다. 표식 자세가 없는 시점의 `n_tags`·`reproj_px`·`depth_invalid`는 null
- `load(role_dir) -> (K, width, height, views)`: recon이 읽는다.

## 설정

`scan_dir/config.yaml`의 `board.tag_mm`, `scene`

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_raw_roundtrip` | 테스트 안에서 만든 작은 protocol 스냅샷 3개(하나는 retake, 하나는 `compress`), stream 몇 줄 | `raw/` 파일 이름·크기·내용이 받은 것과 같음 |
| `test_build` | synth로 만든 `raw/`(k = 0, 1, 3, k = 2는 표식 없음) | views의 k가 {0, 1, 3}, 영상 크기가 컬러 해상도, 자세가 참값 3 mm·1° 안 |
| `test_build_deterministic` | 같은 `scan_dir`로 두 번 `build` | 모든 출력 파일 바이트가 같음 |
| `test_build_stale` | `build` 뒤 k = 1 스냅샷의 표식을 지우고 다시 `build` | `color/`·`depth/`의 파일 이름이 정확히 k = 0, 3만 남음, `outcomes.json`의 k = 1이 `no_tags` |
| `test_retake` | 같은 k에 view와 retake. (가) 둘 다 자세 있음 (나) retake만 표식 없음 (다) 둘 다 없음 | (가) retake (나) view (다) k 없음 |
| `test_intrinsic_open3d` | `camera_intrinsic.json` | `o3d.io.read_pinhole_camera_intrinsic`가 같은 K를 읽음 |
