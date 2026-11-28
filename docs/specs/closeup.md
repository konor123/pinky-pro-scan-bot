# closeup — 고른 지점에 로봇이 다가가서 찍기(웹 화면)

`scan3d/web/closeup.py`, `scan3d/web/session.py`의 `start_closeup`, config `closeup`. 계획은 `docs/plan/CLOSEUP-PLAN.md` 3장.
지금 구현은 그 계획에서 로봇 쪽을 바꾸지 않는 범위다: 방향은 바퀴 주행거리(odom), 전진 확인은 정면 LiDAR 거리. IMU, 초음파,
앞 카메라로 방향 확인하기는 들어 있지 않다.

스캔이 이 서버에서 끝났고 로봇과 물체를 옮기지 않았을 때만 한다. 메시는 다시 합치지 않는다(가까운 사진은 `use: "closeup"`으로
`<role>/raw/`에 남고 데이터셋에는 들어가지 않는다).

## 한 지점의 진행(`CloseupRun`, 보강 촬영 `FocusRun`을 이어받음)

1. 로봇과 보는 방향 `phi`: 보강 촬영과 같다(`webui.split_z` 이상이거나 위를 향한 면이면 H, 법선의 수평 방향에서 본다).
2. 원 위 이동: 보강 촬영과 같다(두 로봇이 같은 각만큼 함께, 긴 원호).
3. `E`: 원 위에서 한 장 찍어 표식으로 자세를 잡는다. 이 표식 자세가 근접 구간의 출발점이다. 표식이 안 보이면 실패.
4. 찍는 자리: 카메라가 지점에서 방향 `phi`로 `closeup.standoff`(기본 0.25 m, 0.20 m보다 작게 두지 않음) 떨어져 지점을 정면으로
   보는 로봇 자세(`shot_pose`, 주행 중 맞춘 장착값 사용). 로봇 기준점이 지점에서 `clear_obj`(0.18 m) 이상 떨어지고, 지점이 사진
   가장자리에서 40픽셀 넘게 안쪽에 들 때까지 `standoff`를 0.02 m씩 늘린다. 원 위보다 0.03 m도 못 다가가면 원 위에서 찍는다.
5. 다가가기(`_approach`): 그 자리 쪽으로 회전 → 멈춘 채 정면 LiDAR 거리(±`front_half_deg`, 5번의 중앙값) `r0` → `forward` 한 번
   → 다시 잰 `r1`. 줄어든 양 `r0 − r1`이 보낸 전진과 `front_tol`(0.08 m) 안에서 같으면 그 값을 믿고(`source: lidar`),
   `fix_tol`(0.02 m) 넘게 덜 갔거나 더 갔으면 `forward`·`back` 한 번으로 고친다. 아니면 odom만(`source: odom`).
   전진 중에는 스캔이 끊기거나 정면 거리가 `r0 − 전진 − 0.05 m`(LiDAR에 물체가 안 보이면 0.10 m)보다 줄면 선다.
   이 구간은 `drive.start_clear`·`stop_dist`를 쓰지 않는다. 마지막에 카메라가 지점을 보게 회전.
6. 촬영: 스냅샷(`closeup`). 카메라 자세는 `E`의 표식 자세를 로봇이 바닥 위에서 움직인 만큼 옮겨 구한다(`moved`). 이 사진에서
   표식으로 자세를 잡지 않는다. 지점을 투영해 `webui.crop_px` 창으로 자르고 고리를 그린다.
   `closeup/<k>_<role>.jpg`(자른 사진), `closeup/<k>_<role>_full.jpg`(전체).
   표식이 보이면 그 자세로 투영한 자리와의 차 `tag_px`만 기록한다.
7. 돌아가기: `E`의 자세로 회전 + 보호 직진 → 한 장 찍어 표식으로 다시 잡는다. 추정 자세와 표식 자세의 차를
   `return_err`(m, °)로 남긴다.

다가가는 것은 찍는 로봇 한 대뿐이고, 다른 로봇은 원 위에 서 있다. 끝나면 두 로봇은 원 위에 있고 각도 차는 그대로다.

## HTTP와 화면

- `POST /api/closeup/start` `{"markers":[{point, normal|null}…]}` → 400 형식, 409 작업 중·스캔 없음
- `state.job.kind: "closeup"`, `progress.phase`: plan → fix → transit → approach → capture → return,
  `progress.crops`·`result.crops`: `{target, role, k, phi, url, reason, distance_m, source, standoff_m, pixel, tag_px, return_err}`
- 사진은 `GET /api/focus/<n>.jpg`(보강 사진과 같은 경로)
- 화면: 보강 묶음의 `근접 촬영` 버튼(고른 점·영역의 대표점을 보낸다), 사진 칸 제목 `근접 사진`

## 테스트(`tests/test_webui.py`)

| 테스트 | 통과 조건 |
|---|---|
| `test_closeup_geometry` | 찍는 자리의 카메라가 지점에서 `standoff`·사진 가로 가운데, `moved`가 그 자리의 실제 카메라 자세와 같음, 원에 가까운 점은 다가가지 않음, 로봇 고르기 |
| `test_closeup_run`(slow) | 가짜 로봇 스캔 뒤 옆면 낮은 점 → L이 0.33 m 안까지 다가가 찍고 두 로봇 모두 원 위(4 cm 안), 각도 차 그대로, 자른 사진과 지점 픽셀, 돌아와서 잰 차 5 cm 안, 메시와 `views.json`은 그대로 |
