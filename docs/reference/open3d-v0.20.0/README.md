# Open3D v0.20.0 예제 사본

[isl-org/Open3D](https://github.com/isl-org/Open3D) 태그 `v0.20.0`(커밋 `b6c5e196384ad71e75b6e6f9c5da22d046221f1d`)의 `examples/python/` 아래 파일을 바꾸지 않고 복사했다. 라이선스는 MIT(`LICENSE`)다.

| 파일 | 원본 경로 | 이 프로젝트에서 쓰는 곳 |
|---|---|---|
| `t_reconstruction_system/integrate.py` | `examples/python/t_reconstruction_system/integrate.py` | `scan3d/recon.py`의 TSDF 융합과 메시·점구름 추출(`docs/specs/recon.md`) |
| `t_reconstruction_system/common.py` | 같은 폴더 | 데이터 폴더 읽기(`load_rgbd_file_names`), 내부 파라미터(`load_intrinsic`), 외부 파라미터(`load_extrinsics`) |
| `t_reconstruction_system/config.py` | 같은 폴더 | 예제의 설정 읽기 |
| `t_reconstruction_system/default_config.yml` | 같은 폴더 | 예제 기본값(`voxel_size`, `trunc_voxel_multiplier`, `depth_max` 등) |
| `t_reconstruction_system/default_intrinsics.json` | 같은 폴더 | 내부 파라미터 파일 예 |
| `reconstruction_system/sensors/realsense_recorder.py` | `examples/python/reconstruction_system/sensors/realsense_recorder.py` | `scan3d/dataset.py`의 데이터 폴더 형식(`docs/specs/dataset.md`) |

## 예제의 흐름(`integrate.py`)

1. 데이터 폴더의 `depth/`·`color/` 영상 목록, 내부 파라미터(Float64 3×3), 외부 파라미터(세계 → 카메라 4×4, 궤적 파일의 자세를 역행렬로 바꾼 것)를 읽는다.
2. `VoxelBlockGrid(attr_names=('tsdf', 'weight', 'color'), voxel_size=3.0/512, block_resolution=16, block_count=50000)`을 만든다.
3. 영상마다 `compute_unique_block_coordinates(depth, intrinsic, extrinsic, depth_scale, depth_max)` → `integrate(block_coords, depth, color, depth_intrinsic, color_intrinsic, extrinsic, depth_scale, depth_max)`
4. `extract_point_cloud()`, `extract_triangle_mesh()`

## 데이터 폴더 형식(`realsense_recorder.py`)

- `color/%06d.jpg`: 컬러(BGR로 저장한 JPEG)
- `depth/%06d.png`: 컬러에 정렬한 깊이(uint16, 깊이 배율 0.001이면 mm)
- `camera_intrinsic.json`: 컬러 카메라의 `{"width", "height", "intrinsic_matrix": [fx, 0, 0, 0, fy, 0, ppx, ppy, 1]}`(열 우선)

## 예제 실행

이 폴더의 파일은 참고용이다. 예제를 그대로 돌리려면 `tqdm`, `configargparse`가 더 필요하다.

```
.venv/bin/python -m pip install tqdm configargparse
cd docs/reference/open3d-v0.20.0/t_reconstruction_system
../../../../.venv/bin/python integrate.py --path_dataset <데이터 폴더> --path_trajectory <궤적 .log> --device CPU:0
```
