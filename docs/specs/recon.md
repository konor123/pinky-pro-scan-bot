# recon

- 코드: `scan3d/pair/recon.py`
- 테스트: `tests/test_recon.py`
- PLAN: 5
- 출처 예제: Open3D v0.20.0 `examples/python/t_reconstruction_system/integrate.py`(사본 `docs/reference/open3d-v0.20.0/t_reconstruction_system/`)

## 하는 일

dataset의 시점별 컬러·깊이와 표식 자세 `T_cam_board`로 물체만 잘라 TSDF로 융합하고, 색 있는 메시와 치수를 낸다.

## 인터페이스

- `reconstruct(role_dirs, cfg) -> Result(mesh, cloud, dims_mm, used)`: `role_dirs`는 `scans/<id>/L`, `scans/<id>/H` 중 있는 것. 역할마다 자기 `camera_intrinsic.json`과 `views.json`을 쓴다(시점 번호는 빠질 수 있다). `used`는 역할별로 융합에 쓴 k 목록
- `measure(mesh) -> (w, d, h)` mm: 물체 전체 크기. h는 꼭짓점 최대 z, w·d는 `floor_cut` 위 모든 꼭짓점 xy를 감싸는 최소 면적 직사각형(`cv2.minAreaRect`)의 두 변(큰 것이 w)
- `write_outputs(scan_dir, result, report)`: `out/mesh.ply`, `out/cloud.ply`, `out/report.txt`, 역할별 `outcomes.json`의 `crop`·`fused`

## 동작

1. **읽기**: 컬러는 `o3d.t.io.read_image`로 읽어 RGB로 쓴다. 깊이는 uint16 mm
2. **자르기**: 깊이 픽셀마다 3D 점을 판 좌표로 옮겨, `hypot(x, y) ≤ r_obj + place_err + crop_margin`이고 `floor_cut < z ≤ obj_h + crop_margin`인 것만 남기고 나머지는 0으로 만든다. 판과 바닥이 빠진다. 남은 픽셀이 없거나, 남았어도 `compute_unique_block_coordinates`가 `No block is touched`로 실패한 시점은 건너뛴다.
3. **융합**: `o3d.t.geometry.VoxelBlockGrid`(`tsdf`, `weight`, `color`), 복셀 `recon.voxel`, 블록 해상도 16, CPU. 시점마다
   - `coords = vbg.compute_unique_block_coordinates(depth, K, T_cam_board, depth_scale=1000.0, depth_max=1.5, trunc_voxel_multiplier=m)`
   - `vbg.integrate(coords, depth, color, K, K, T_cam_board, 1000.0, 1.5, m)`
   - `m = recon.trunc / recon.voxel`
4. **메시**: `extract_triangle_mesh()` → 레거시 메시로 바꿔 연결 덩어리를 나눈 뒤 가장 큰 덩어리만 남긴다. 점구름은 `extract_point_cloud()`. 융합한 시점이 없거나 메시가 비면 빈 `Result`(치수 없음)
5. **보고**: `report.txt`에 치수(mm), 역할별 융합한 시점 수와 k, 시점별 `d`·`n_tags`·`reproj_px`·`depth_invalid`, 건너뛴 시점(우회, 표식 없음, 자르고 남은 것 없음), 거리 보정 횟수, 걸린 시간

## 예제와의 대응

`integrate.py`의 `integrate()`를 그대로 따르고, 아래만 바꾼다.

| 항목 | 예제 | recon |
|---|---|---|
| 입력 폴더 | `color/`, `depth/`, `camera_intrinsic.json` | 같음(dataset이 이 형식으로 만든다) |
| 외부 파라미터 | 궤적 파일(`.log`, 포즈 그래프)의 세계 → 카메라 행렬 | 시점마다 표식 자세 `T_cam_board`(판 → 카메라). 같은 방향의 행렬이라 그대로 넣는다 |
| 내부 파라미터 | `load_intrinsic` → Float64 3×3, 깊이·컬러 따로 | 왜곡 보정한 컬러 K 하나를 깊이·컬러에 같이 쓴다(깊이가 컬러에 정렬됨) |
| `VoxelBlockGrid` | `('tsdf', 'weight', 'color')`, 블록 해상도 16, `voxel_size` 3/512 m(약 5.9 mm), `block_count` 50000 | 같은 속성·블록 해상도, `voxel_size` = `recon.voxel`(3 mm), `block_count` 2000(넘으면 Open3D 해시맵이 스스로 늘어난다. 20000이면 격자 하나가 약 1.6 GB) |
| 장치 | `CUDA:0` | `CPU:0` |
| 절단 거리 | 넘기지 않아 기본값 8복셀 | `trunc_voxel_multiplier = recon.trunc / recon.voxel`(2복셀, 6 mm)를 두 호출에 모두 넘긴다 |
| `depth_max` | 3.0 m | 1.5 m |
| 깊이 전처리 | 없음 | 자르기(동작 2), 남은 것이 없거나 블록이 안 생기는 시점은 건너뜀 |
| 결과 | `extract_point_cloud()`, `extract_triangle_mesh()`를 화면에 띄움 | 같은 두 함수, 메시는 가장 큰 덩어리만 남겨 `out/`에 저장하고 치수를 잰다 |

## 설정

`recon.voxel`, `recon.trunc`, `scene.r_obj`, `scene.place_err`, `scene.obj_h`, `scene.crop_margin`, `scene.floor_cut`

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_box_dims` (slow) | 면마다 색이 다른 synth 상자(config 크기 안), L·H 각 24시점, 깊이 잡음 | 치수가 각각 5 mm 안, 덩어리를 고르기 전 가장 큰 덩어리가 삼각형의 95% 이상, 빨강 면 영역 꼭짓점 색이 R > G·B(RGB 순서) |
| `test_sparse_views` | 같은 장면에서 L k = 3, 4를 뺌 | `used`가 정확히 L {0…23} − {3, 4}, H {0…23} |
| `test_crop_empty` | 판만 있는 장면 | 예외 없이 빈 `Result` |
| `test_crop_sparse` | 자른 뒤 0.5 m 깊이 픽셀 하나만 남는 시점 두 개: (501, 300)(블록 없음), (500, 300)(블록 하나) | 앞 시점은 `no_block`으로 건너뛰고 뒤 시점은 융합, 예외 없음 |
