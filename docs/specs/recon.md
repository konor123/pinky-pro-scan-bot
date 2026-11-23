# recon

- 코드: `scan3d/pair/recon.py`
- 테스트: `tests/test_recon.py`
- PLAN: 5
- 출처 예제: Open3D v0.20.0 `examples/python/t_reconstruction_system/integrate.py`(사본 `docs/reference/open3d-v0.20.0/t_reconstruction_system/`), 색 다시 입히기는 `examples/python/pipelines/color_map_optimization.py`

## 하는 일

dataset의 시점별 컬러·깊이와 표식 자세 `T_cam_board`로 물체만 잘라 TSDF로 융합하고, 색 있는 메시와 치수를 낸다.

## 인터페이스

- `reconstruct(role_dirs, cfg) -> Result(mesh, cloud, dims_mm, used)`: `role_dirs`는 `scans/<id>/L`, `scans/<id>/H` 중 있는 것. 역할마다 자기 `camera_intrinsic.json`과 `views.json`을 쓴다(시점 번호는 빠질 수 있다). `used`는 역할별로 융합에 쓴 k 목록
- `drop_depth_edges(depth, edge_mm) -> depth`: 3×3 이웃의 최대 − 최소가 `edge_mm`보다 크거나 이웃에 0(값 없음)이 있는 픽셀을 0으로 만든 사본. `edge_mm` ≤ 0이면 그대로
- `recolor(mesh, role_dirs, used, iters) -> mesh`: 융합에 쓴 시점(`used`)의 컬러·깊이와 표식 자세로 Open3D `pipelines.color_map.run_rigid_optimizer`를 돌려 꼭짓점 색을 다시 구한다. 모양(꼭짓점 위치·삼각형)은 바꾸지 않는다
- `python -m scan3d.pair.recon <역할 폴더>... [--config config/scan.yaml] [--out DIR] [--color-opt N]`: 저장된 역할 폴더를 지금 설정으로 다시 융합해 `DIR`에 `mesh.ply`, `cloud.ply`, `report.txt`(치수, 역할별 `<스캔>/<역할>`과 융합한 시점, 쓴 설정. 절대 경로는 적지 않는다)를 쓴다. 원래 폴더는 바꾸지 않는다. `--color-opt N`은 `recon.color_opt_iter`를 N으로 바꿔 돌린다(색 다시 입히기 켜고 끈 결과 비교용). `DIR` 기본값은 역할 폴더들이 한 스캔 폴더 안이면 `<스캔>/out_recon`, 아니면 `--out`이 필요하다. 다른 스캔의 L·H를 합칠 때는 두 촬영 사이에 판과 물체를 움직이지 않았어야 한다.
- `measure(mesh) -> (w, d, h)` mm: 물체 전체 크기. h는 꼭짓점 최대 z, w·d는 `floor_cut` 위 모든 꼭짓점 xy를 감싸는 최소 면적 직사각형(`cv2.minAreaRect`)의 두 변(큰 것이 w)
- `write_outputs(scan_dir, result, report)`: `out/mesh.ply`, `out/cloud.ply`, `out/report.txt`, 역할별 `outcomes.json`의 `crop`·`fused`

## 동작

1. **읽기**: 컬러는 `o3d.t.io.read_image`로 읽어 RGB로 쓴다. 깊이는 uint16 mm
   - **가장자리 지우기**: 자르기 전에 `drop_depth_edges(depth, recon.edge_mm)`. 물체 윤곽에서 앞뒤 깊이가 섞인 픽셀(flying pixel)과 정렬 구멍 둘레를 지워, 판·바닥 색이 물체 가장자리에 묻지 않게 한다. 자른 뒤에 하면 자르기 경계가 깎이므로 먼저 한다.
2. **자르기**: 깊이 픽셀마다 3D 점을 판 좌표로 옮겨, `hypot(x, y) ≤ r_obj + place_err + crop_margin`이고 `floor_cut < z ≤ obj_h + crop_margin`인 것만 남기고 나머지는 0으로 만든다. 판과 바닥이 빠진다. 남은 픽셀이 없거나, 남았어도 `compute_unique_block_coordinates`가 `No block is touched`로 실패한 시점은 건너뛴다.
3. **융합**: `o3d.t.geometry.VoxelBlockGrid`(`tsdf`, `weight`, `color`), 복셀 `recon.voxel`, 블록 해상도 16, CPU. 시점마다
   - `coords = vbg.compute_unique_block_coordinates(depth, K, T_cam_board, depth_scale=1000.0, depth_max=1.5, trunc_voxel_multiplier=m)`
   - `vbg.integrate(coords, depth, color, K, K, T_cam_board, 1000.0, 1.5, m)`
   - `m = recon.trunc / recon.voxel`
4. **메시**: `extract_triangle_mesh()` → 레거시 메시로 바꿔 연결 덩어리를 나눈 뒤 가장 큰 덩어리만 남긴다. 점구름은 `extract_point_cloud()`. 융합한 시점이 없거나 메시가 비면 빈 `Result`(치수 없음)
   - **색 다시 입히기**(`recon.color_opt_iter` > 0일 때): `recolor(mesh, role_dirs, used, color_opt_iter)`. 시점 자세는 표식 자세에서 시작해 사진끼리 색이 맞도록 조금 다듬고, 꼭짓점마다 보이는 시점들의 색으로 다시 칠한다. 보이는지 판단: 깊이 차 10 mm, 깊이 기울기 0.05, `maximum_allowable_depth` = 1.5 m
5. **보고**: `report.txt`에 치수(mm), 역할별 융합한 시점 수와 k, 시점별 `d`·`n_tags`·`reproj_px`·`depth_invalid`, 건너뛴 시점(우회, 표식 없음, 자르고 남은 것 없음), 거리 보정 횟수, 걸린 시간

## 예제와의 대응

`integrate.py`의 `integrate()`를 그대로 따르고, 아래만 바꾼다.

| 항목 | 예제 | recon |
|---|---|---|
| 입력 폴더 | `color/`, `depth/`, `camera_intrinsic.json` | 같음(dataset이 이 형식으로 만든다) |
| 외부 파라미터 | 궤적 파일(`.log`, 포즈 그래프)의 세계 → 카메라 행렬 | 시점마다 표식 자세 `T_cam_board`(판 → 카메라). 같은 방향의 행렬이라 그대로 넣는다 |
| 내부 파라미터 | `load_intrinsic` → Float64 3×3, 깊이·컬러 따로 | 왜곡 보정한 컬러 K 하나를 깊이·컬러에 같이 쓴다(깊이가 컬러에 정렬됨) |
| `VoxelBlockGrid` | `('tsdf', 'weight', 'color')`, 블록 해상도 16, `voxel_size` 3/512 m(약 5.9 mm), `block_count` 50000 | 같은 속성·블록 해상도, `voxel_size` = `recon.voxel`(2 mm), `block_count` 2000(넘으면 Open3D 해시맵이 스스로 늘어난다. 20000이면 격자 하나가 약 1.6 GB) |
| 장치 | `CUDA:0` | `CPU:0` |
| 절단 거리 | 넘기지 않아 기본값 8복셀 | `trunc_voxel_multiplier = recon.trunc / recon.voxel`(3복셀, 6 mm)를 두 호출에 모두 넘긴다 |
| `depth_max` | 3.0 m | 1.5 m |
| 깊이 전처리 | 없음 | 가장자리 지우기(동작 1), 자르기(동작 2), 남은 것이 없거나 블록이 안 생기는 시점은 건너뜀 |
| 결과 | `extract_point_cloud()`, `extract_triangle_mesh()`를 화면에 띄움 | 같은 두 함수, 메시는 가장 큰 덩어리만 남겨 `out/`에 저장하고 치수를 잰다 |

## 설정

`recon.voxel`, `recon.trunc`, `recon.edge_mm`, `recon.color_opt_iter`, `scene.r_obj`, `scene.place_err`, `scene.obj_h`, `scene.crop_margin`, `scene.floor_cut`

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_box_dims` (slow) | 면마다 색이 다른 synth 상자(config 크기 안), L·H 각 24시점, 깊이 잡음 | 치수가 각각 5 mm 안, 덩어리를 고르기 전 가장 큰 덩어리가 삼각형의 95% 이상, 빨강 면 영역 꼭짓점 색이 R > G·B(RGB 순서) |
| `test_sparse_views` | 같은 장면에서 L k = 3, 4를 뺌 | `used`가 정확히 L {0…23} − {3, 4}, H {0…23} |
| `test_crop_empty` | 판만 있는 장면 | 예외 없이 빈 `Result` |
| `test_crop_sparse` | 자른 뒤 0.5 m 깊이 픽셀 하나만 남는 시점 두 개: (501, 300)(블록 없음), (500, 300)(블록 하나). 이 배치는 복셀 3 mm 기준이라 `voxel` 3 mm·`trunc` 6 mm·`edge_mm` 0으로 돌린다 | 앞 시점은 `no_block`으로 건너뛰고 뒤 시점은 융합, 예외 없음 |
| `test_edge_filter` | 0.5 m 평면 위 0.45 m 정사각, 픽셀당 4 mm 기울어진 평면, 구멍 한 칸 | 정사각 경계 양쪽 한 픽셀씩과 구멍 둘레만 0, 정사각 안쪽과 기울어진 평면은 그대로, `edge_mm` 0이면 입력과 같음 |
| `test_recolor` (slow) | `test_box_dims`와 같은 장면, `color_opt_iter` 20 | 예외 없음, 꼭짓점·삼각형 수 그대로, 빨강 면 영역 꼭짓점 색이 R > G·B인 비율이 다시 칠하기 전보다 0.02 넘게 낮지 않음 |
| `test_cli` (slow) | `test_box_dims`와 같은 장면의 L·H 폴더로 `python -m scan3d.pair.recon`, 다시 `--color-opt 5 --out <임시>` | 종료 코드 0, `out_recon/`에 `mesh.ply`·`report.txt`(치수), 두 번째는 `report.txt`에 `color_opt_iter 5`, 역할 폴더의 파일은 바뀌지 않음 |
