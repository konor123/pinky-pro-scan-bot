# align

- 코드: `scan3d/align.py`
- 테스트: `tests/test_align.py`
- PLAN: 4.2, 4.5, 5
- 출처 예제: librealsense v2.59.2 `wrappers/python/examples/align-with-software-device.py`

## 하는 일

스냅샷 원본(컬러 1장, 깊이 n장, 보정값)을 컬러 카메라 기준의 왜곡 없는 컬러·깊이 한 쌍과 핀홀 내부 행렬 K로 만든다. tagpose, dataset, recon은 이 결과만 쓴다.

## 인터페이스

- `decode_color(hdr, data) -> BGR uint8`: `yuyv`는 `cv2.COLOR_YUV2BGR_YUYV`, `rgb8`은 `cv2.COLOR_RGB2BGR`
- `decode_depth(hdr, data, depth_scale) -> list[uint16 mm]`: `z16`은 값 × `depth_scale` × 1000을 반올림, `f32`(m)는 × 1000을 반올림. 유한하지 않거나 0 이하이면 0
- `median_depth(frames) -> uint16 mm`: 픽셀마다 0이 아닌 값의 중앙값(개수가 짝수면 가운데 두 값의 평균을 반올림). 0이 아닌 값이 `ceil(n/2)`개보다 적으면 0
- `align_to_color(depth_mm, calib) -> uint16 mm`(컬러 해상도, 왜곡 있는 컬러 픽셀 격자)
- `undistort(color, depth, calib) -> (color, depth, K)`: 컬러 왜곡 계수 `[k1, k2, p1, p2, k3]`를 OpenCV 계수로 써서 컬러(`INTER_LINEAR`)와 정렬된 깊이(`INTER_NEAREST`)를 같은 지도로 편다. K는 컬러 내부 파라미터 그대로. 모델이 `none`이거나 계수가 모두 0이면 복사만 한다.
- `prepare(header, payloads, calib) -> Frame(color, depth, K)`: decode → 중앙값 → 정렬 → 왜곡 보정. `calib`은 `hello` 응답의 `calib`

## 동작

1. 중앙값을 먼저 내고 정렬은 한 번만 한다(스냅샷 동안 카메라와 장면이 멈춰 있다).
2. 정렬은 예제대로 software device로 한다.
   - 깊이 센서와 컬러 센서를 `calib` 내부 파라미터로 만들고, 깊이 → 컬러 외부 파라미터를 등록한다.
   - 깊이 센서에 `add_read_only_option(rs.option.depth_units, 0.001)`을 넣고 mm 값으로 프레임을 만든다. 컬러 스트림은 YUY2(bpp 2) 빈 프레임이면 된다.
   - 한 쌍의 두 프레임에는 같은 `timestamp`와 `frame_number`를 준다. syncer의 첫 세트에는 한 스트림만 있을 수 있으므로, 깊이와 컬러가 모두 든 세트가 나올 때까지 같은 픽셀로 쌍을 다시 넣되 시도마다 `timestamp`와 `frame_number`를 늘린다(최대 10번, 그래도 없으면 `RuntimeError`).
   - 그 세트에 `rs.align(rs.stream.color)`를 적용한다.
3. `rs.align`이 채운 값은 깊이 카메라 기준 z다. 이를 컬러 카메라 z로 바꾼다: 깊이 → 컬러 변환을 `P_c = R·P_d + t`라 하고 `r3`를 R의 셋째 열, `ray = K_c⁻¹·[u, v, 1]`(왜곡 보정 전 컬러 픽셀의 광선, 컬러 왜곡 계수로 편 것)이라 하면 `z_c = (z_d + r3·t) / (r3·ray)`(길이는 모두 m로 계산, 깊이 0인 픽셀은 0으로 둔다). 출력 픽셀에 표본 깊이를 옮겨 담는 데서 생기는 오차가 남는다(기운 면에서 95%가 1.1 mm, 최대 약 2.1 mm).
4. 왜곡 모델이 `brown_conrady`·`inverse_brown_conrady`·`modified_brown_conrady`면 계수를 OpenCV 계수로 그대로 쓴다(`test_undistort`가 D455 비슷한 계수에서 v2.59.2 `rs2_project_point_to_pixel`과 비교한다). `none`은 왜곡 없음이다. 그 밖의 모델이면 `ValueError`
5. 시뮬레이션 에이전트(`rgb8`, `f32`, 외부 파라미터 단위 행렬, 두 카메라 내부 파라미터 같음, 모델 `none`)도 같은 함수로 처리된다.

## 설정

없음(`calib`과 머리말로 동작).

## 완료 테스트(synth, `default_calib`의 깊이 → 컬러 회전 0.3°)

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_decode` | 채도가 낮은 컬러 영상의 `to_yuyv`, 깊이 `to_z16`, `depth_scale` 0.001과 0.0001 | 컬러 평균 차 3 이내, 깊이 mm 같음 |
| `test_median` | 정해 둔 값 5장(±2 mm 안의 고정 잡음), 한 영역은 3장에서 0, 한 영역은 0이 아닌 값이 4개(짝수) | 중앙값이 참값 ±2 mm, 0이 3장인 영역은 0, 짝수 개는 가운데 두 값 평균을 0.5 올림(예: 1000, 1001 → 1001) |
| `test_align` | 깊이 카메라 시점 깊이 | 컬러 시점 참 깊이와 비교: 그 3D 점이 깊이 카메라에도 보이는 픽셀의 95% 이상이 채워지고, 깊이 경계에서 3 px 밖인 픽셀의 95%가 1.2 mm 안, 최대 2.5 mm |
| `test_align_tilted_plane` | 0.5~0.94 m에 걸친 기운 평면, 회전 0.3° | 내부 픽셀의 95%가 컬러 시점 참 z와 1.2 mm 안, 최대 2.5 mm(z 변환 전에는 최대 6 mm) |
| `test_sync_retry` | 첫 세트가 한 스트림만 나오는 경우 | 두 번째 이후 시도에서 완성, 10번 실패면 `RuntimeError` |
| `test_align_identity` | 외부 파라미터 단위 행렬, 같은 내부 파라미터, `f32`, 모델 `none` | 입력과 1 mm 안 |
| `test_undistort` | 핀홀 영상의 점들을 `rs.rs2_project_point_to_pixel`(`inverse_brown_conrady`, D455 비슷한 계수)로 왜곡한 영상 | 편 영상에서 점 위치가 원래와 0.5 px 안 |
