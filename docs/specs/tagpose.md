# tagpose

- 코드: `scan3d/tagpose.py`
- 테스트: `tests/test_tagpose.py`
- PLAN: 4.1, 5
- 출처 예제: opencv_contrib 4.11.0 `cv2.aruco.ArucoDetector`, `Board.matchImagePoints`, `cv2.solvePnP`

## 하는 일

align이 만든 왜곡 없는 컬러 영상과 K에서 표식을 찾아, 보이는 표식 전부로 컬러 카메라 자세 `T_cam_board`를 한 번에 구한다. 이 자세에서 시점 각 `phi`와 거리 `d`를 계산한다.

## 인터페이스

- `load_board(board_json, tag_mm) -> Board`: `corners`에 `tag_mm / board_json["tag_mm"]`을 곱해 `cv2.aruco.Board`를 만든다.
- `estimate(bgr, K, board) -> Pose | None`: `Pose(T_cam_board 4×4, n_tags, reproj_px, ids)`. 서로 다른 id가 2개보다 적으면 `None`
- `cam_position(T_cam_board) -> (x, y, z)`: 판 좌표의 카메라 위치(= `−Rᵀt`)
- `phi_deg(T_cam_board) -> float`: `atan2(y, x)`, 도, (−180, 180]
- `distance_d(T_cam_board, r_eff) -> float`: `hypot(x, y) − r_eff`

## 동작

1. 회색으로 바꿔 `DICT_APRILTAG_36h11`, `cornerRefinementMethod = CORNER_REFINE_APRILTAG`으로 검출한다. 이 보정은 모서리를 (+0.5, +0.5) px 치우쳐 돌려주므로(OpenCV 4.11, 정수 픽셀 중심 기준) 모서리에서 0.5 px를 뺀다.
2. 배치 파일에 없는 id와, 모서리 하나라도 영상 가장자리에서 2 px 안에 있는 표식(잘린 표식)은 버린다. 서로 다른 id가 2개보다 적으면 `None`. `board.matchImagePoints`로 3D–2D 대응을 만들고 `solvePnP(SOLVEPNP_ITERATIVE)`를 왜곡 계수 없이 한 번 푼다(영상은 align이 이미 폈다).
3. `reproj_px`는 대응점 재투영 오차의 RMS다. 걸러 내지 않고 기록만 한다.

## 설정

`board.layout`, `board.tag_mm`, `scene.r_obj`, `scene.place_err`(r_eff = r_obj + place_err)

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_pose_views` | synth 판 + 원기둥(반지름 r_eff, 높이 `obj_h`), config의 L·H 장착 높이·피치, 원 반지름 R × 시점 0°·45°·90°·135°, 흐림 σ 1.5 px, 잡음 σ 5 | 위치 3 mm, 각도 1° 안, 표식 4장 이상 |
| `test_corner_bias` | 같은 영상들 | 영상마다 투영한 참 모서리와 검출 모서리의 부호 있는 평균 차가 x·y 모두 0.15 px 안, 모서리 하나하나는 1 px 안 |
| `test_phi_d` | `pose_on_circle(30, R, H 높이, H 피치)` | `phi` 30 ± 0.5°, `d` = R − r_eff ± 0.005 m |
| `test_scale` | 같은 영상, `tag_mm`을 배치 파일 값 × 1.01로 만든 판 | 카메라 위치가 판 중심 기준으로 1% 멀어짐 |
| `test_no_tags` | 회색 영상 | `None` |
