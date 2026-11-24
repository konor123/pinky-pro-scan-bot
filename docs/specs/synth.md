# synth

- 코드: `scan3d/sim/synth.py`
- 테스트: `tests/test_synth.py`
- PLAN: 5
- 출처 예제: Open3D v0.20.0 `o3d.t.geometry.RaycastingScene`

## 하는 일

알려진 물체 메시와 표식 판을 놓은 장면을, 알려진 자세의 컬러 카메라와 깊이 카메라로 따로 렌더링한다. tagpose, align, recon, 가짜 에이전트 테스트의 입력이다.

## 인터페이스

- `default_calib() -> dict`: `hello`의 `calib`과 같은 형식(protocol)의 D455 비슷한 값. 컬러 1280×720 fx = fy = 640, ppx 639.5, ppy 359.5, 깊이 848×480 fx = fy = 447(가로 시야 87°), ppx 423.5, ppy 239.5, 모델 `brown_conrady`, 계수 0, 깊이 → 컬러 이동 (−0.059, 0, 0) m, 회전은 y축으로 0.3°(열 우선), 깊이 배율 0.001
- 픽셀 중심 규칙: 픽셀 (u, v)의 광선 방향은 `((u − ppx)/fx, (v − ppy)/fy, 1)`이다(정수 픽셀 중심, OpenCV·librealsense와 같음). Open3D `create_rays_pinhole`은 u + 0.5를 쓰므로 그대로 쓰지 않는다.
- `board_texture(board, px_per_mm=4) -> np.ndarray`: `board.json`과 `cv2.aruco.generateImageMarker`로 그린 판 영상(회색, 잉크 30, 종이 225). `tools/make_board.py`의 PDF와 같은 배치
- `box_mesh(w, d, h, face_colors=None)`(면마다 색을 줄 수 있음), `asym_mesh()`: 판 위(z = 0)에 놓인 시험 물체(Open3D 메시, 꼭짓점 색 있음). `asym_mesh`는 config `scene` 크기(반지름 `r_obj`, 높이 `obj_h`) 안의 상자 + 원기둥 조합
- `Scene(board, mesh, floor_gray=90)`: 판(z = 0, `board_mm` 정사각), 판 밖 바닥(z = 0, 회색. 용지를 바닥에 바로 펴 놓으므로 판과 같은 높이), 물체. `board`는 `board.json` dict
- `Scene.render_color(T_cam_board, intr) -> BGR uint8`: 물체는 꼭짓점 색 × 람베르트 음영, 판은 텍스처, 3배 과표본 후 `INTER_AREA` 축소
- `Scene.render_depth(T_depth_board, intr, rng=None, noise_mm=0) -> uint16 mm`: 광축 방향 거리(z). 맞는 면이 없거나 0.35 m보다 가까우면 0. `rng`가 있으면 거리 비례 잡음(σ = `noise_mm` × (z/1 m)²)
- `pose_on_circle(phi_deg, R, h, pitch_down_deg) -> T_cam_board`: 카메라가 반지름 R 원의 각 phi, 판 좌표 z = h에 있고, 수평 방향은 판 가운데를 향하며 주어진 피치로 내려다보는 컬러 카메라 자세
- `depth_pose(T_cam_board, calib) -> T_depth_board`: 깊이 → 컬러 외부 파라미터로 구한 깊이 카메라 자세
- `to_yuyv(bgr) -> bytes`, `to_z16(depth) -> bytes`: 실물과 같은 원본 바이트

## 동작

1. 광선은 `RaycastingScene.cast_rays`로 쏜다. 판과 바닥은 평면 교차로 계산하고 물체에 먼저 맞으면 물체 값을 쓴다.
2. 같은 입력과 같은 `rng` seed면 같은 영상이 나온다.

## 설정

없음(인자로 받는다).

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_depth_plane` | 판만 있는 장면, 바로 위 1 m에서 아래를 보는 깊이 카메라 | 가운데 픽셀, 판 안의 비스듬한 광선, 판 밖 바닥 모두 1000 ± 1 mm |
| `test_depth_box_top` | 0.2 × 0.2 × 0.18 m 상자, 같은 카메라 | 상자 윗면 픽셀 820 ± 1 mm |
| `test_color_board_corners` | 판, `pose_on_circle(0, R, H 높이, H 피치)` | 표식 모서리를 `T_cam_board`로 투영한 위치와 `CORNER_REFINE_SUBPIX`로 검출한 위치의 부호 있는 평균 차가 x·y 모두 0.15 px 안 |
| `test_raw_bytes` | 1280×720 컬러(채도 낮은 영상), 848×480 깊이 | `to_yuyv` 1,843,200 B, `to_z16` 814,080 B, `cv2.COLOR_YUV2BGR_YUYV`로 되돌린 영상이 원래와 평균 3 이내 |
