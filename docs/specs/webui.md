# webui — 웹 화면 하나로 연결 확인, 맞추기, 스캔, 보강 촬영

스캔 PC에서 서버 하나(`python -m scan3d.web`, `tools/webui.sh`)를 띄우고 브라우저로 모든 일을 한다.
터미널 명령(`tools/scan.sh`, `tools/show_pose.py`, `tools/view_web.py`)으로 하던 일을 화면 버튼으로 옮긴 것이고,
스캔 자체는 `pair.coordinator`·`station.run`을 그대로 쓴다.

## 구성

| 모듈 | 하는 일 |
|---|---|
| `scan3d/web/session.py` | 로봇 연결을 쥐고 미리보기·스캔·보강 촬영을 한 번에 하나만 진행하는 상태 기계 |
| `scan3d/web/preview.py` | 스냅샷 한 장 → 표식 자세, 카메라 각도, 깊이 거리, 표식을 그린 작은 사진 |
| `scan3d/web/focus.py` | 마커 → 찍을 로봇·각도 계획, 원 위 이동, 보강 촬영, 자른 사진, 다시 합치기 |
| `scan3d/web/holes.py` | 메시의 열린 경계(구멍)를 찾아 마커 후보로 |
| `scan3d/web/coverage.py` | 덮인 정도: 꼭짓점마다 보인 시점 수·정면도(깊이로 가림 판정) |
| `scan3d/web/results.py` | `scans/`·`results/`의 결과 목록과 파일(표준 라이브러리만, `tools/view_web.py`도 씀) |
| `scan3d/web/server.py`, `__main__.py` | HTTP 서버(표준 `http.server`), `/api/*`와 화면 파일 |
| `scan3d/web/static/` | 화면(`index.html`, `app.js`, `meshview.js`, `ply.js`, `style.css`). 외부 스크립트를 받지 않는다 |

`tools/view_web.py`는 같은 화면을 보기 전용(`mode: "view"`)으로 내준다(cv2·open3d 없이 호스트 python3로 뜬다).

## 상태

```
disconnected ──connect──▶ ready ──scan/start──▶ scanning ⇄ paused ──▶ reconstructing ──▶ ready(결과 있음)
                  ▲          │                     │ cancel                                    │
                  └──────────┴──── 로봇 끊김 ◀──────┘                      focus/start ──▶ focusing ⇄ paused ──▶ ready
```

- 한 번에 한 작업(`scan`, `focus`)만. 작업 중에는 미리보기를 멈춘다(카메라와 제어 연결은 한 곳만 쓴다).
- `ready`에서 미리보기를 켜 두면 로봇마다 차례로(L → H → L …) 스냅샷을 찍어 맞추기 값을 갱신한다.
  두 로봇이 동시에 카메라를 켜지 않는다(투사기 간섭).
- 취소하면 두 로봇에 `abort`를 보낸다(제어 연결이 닫힌다). 작업이 끝나면 연결을 새로 맺는다.
- 일시정지는 다음 단계 경계(시점 이동 앞, 촬영 앞)에서 선다. 이미 보낸 이동은 끝까지 간다(시점 이동은 수 초, 보강 촬영의 긴 원호는 10 s 안팎).
- 메시를 합치는 중(`job.recon`)에는 로봇이 서 있으므로 일시정지·취소를 받지 않는다(409).
- 사용자가 누른 취소만 `cancelled`다. 스캔이 실패하면(표식 안 보임, 경로 막힘 등) `failed`와 그 이유.
- 작업 스레드는 정리(링크 원래대로, 중단된 연결 닫기)를 마친 뒤에 끝 상태를 낸다. 서버를 끄면 진행 중인 작업에 abort를 보내고 정리를 기다린다.

## 맞추기 값(미리보기, 로봇마다)

스냅샷 → `align.prepare` → `tagpose.estimate`. 표식이 2장 미만이면 `pose: false`와 이유만.

| 값 | 뜻 |
|---|---|
| `phi_deg` | 판 좌표 카메라 각(`tagpose.phi_deg`) |
| `d_m` | 카메라–물체 수평 거리 `hypot(x, y) − r_eff` |
| `cam_z_m`, `pitch_deg` | 카메라 높이, 아래로 숙인 각(광축과 수평의 각). config `mount`와의 차 `pitch_err_deg`, `z_err_m` |
| `aim_err_deg` | 광축의 수평 방향과 카메라 → 판 중심 방향의 차(양수 = 판 중심이 화면 왼쪽) |
| `depth_center_m` | 정렬한 깊이의 가운데 1/5 창 중앙값(0 제외). 물체까지의 실제 깊이 |
| `n_tags`, `reproj_px` | 보인 표식 수, 재투영 오차 |
| `age_s` | 이 값을 잰 뒤 지난 시간 |

두 대 값: `gap_deg`(H − L 각도 차)와 `gap_ok`(180 ± `drive.pair_gap_tol_deg`), 고칠 방향 문구(`coordinator.check_pair_gap`과 같은 문장).
출발 가능(`can_start`): 로봇이 모두 연결되어 있고, 최근 30초 안에 잰 두 각도가 있으면 그 차가 `gap_ok`.
아직 못 쟀으면 막지 않는다(스캔이 출발할 때 `start_poses`가 같은 조건을 다시 본다). 연결이 끊기면 그 로봇의 미리보기는 지운다.

## 보강 촬영(focus)

입력: 마커 목록 `[{point:[x,y,z], normal:[nx,ny,nz]|null}]`(판 좌표, m). 마지막 스캔이 이 서버에서 끝났고 로봇과 물체를 옮기지 않았을 때만.

1. 계획(`focus.plan`, 순수 함수)
   - 로봇: `z ≥ webui.split_z`(0.15) 또는 법선이 수평에서 45° 넘게 위면 H, 아니면 L. 그 로봇이 연결되어 있지 않으면 다른 로봇.
   - 보는 각 `phi`: 법선의 수평 성분이 0.3 이상이면 그 방향, 아니면 판 중심 → 점 방향. 점이 중심에 있으면 지금 로봇 각.
   - 찍는 각: `phi + δ`, `δ ∈ webui.focus_offsets_deg`(기본 −10, 0, +10).
2. 진행(마커마다)
   - 두 로봇 모두 표식으로 자세를 다시 잡는다(스냅샷 한 장). 두 대면 각도 차가 180 ± `pair_gap_tol_deg` 밖일 때 움직이지 않고 실패.
   - 찍는 각마다: 맡은 로봇이 그 각까지 가야 할 각 Δ(가까운 방향, −180 ~ 180°)만큼 **두 로봇이 함께** 원 위를 이동한다(스캔과 같은 문장, 장애물 감시 같음, 우회 없음 — 막히면 `block_timeout_s` 뒤 실패). 두 로봇의 각도 차가 늘 그대로라 서로 마주치지 않는다.
     - 두 로봇 모두 원호 문장을 알 때: 반시계(Δ > 0)면 `webui.transit_max_deg`(기본 90°) 이하의 원호 하나씩 간다(원호 하나는 서지 않고 끝까지). 원호 하나를 마치고 갈 길이 남았으면 두 로봇 모두 표식으로 자세를 다시 잡는다(스냅샷 한 장씩). 시계 방향으로 45°보다 멀면 Δ + 360°만큼 반시계로 돌아서 간다(로봇은 원호를 앞으로만 달리고, 시계 방향은 15°마다 제자리 반 바퀴 회전이 두 번 든다). 시계 방향으로 45° 안이면 15° 이하씩 회전 + 직진.
     - 원호를 모르는 로봇이 있거나 `transit_max_deg`가 15일 때: 가까운 방향으로 15° 이하씩(현이 원 안으로 들어가지 않게).
     - 원호로 떠날 수 없는 자세(원에서 5 cm, 접선에서 20° 넘게 벗어남, `Driver.on_orbit`)면 먼저 15° 이하로 한 번 가서 자리를 잡는다.
   - 맡은 로봇이 `settle_s` 뒤 스냅샷 → 표식 자세. 스냅샷은 스캔 폴더의 `<role>/raw/`에 `use: "view"`, `k ≥ 100`으로 더한다(번호는 이어서).
   - 점을 그 사진에 투영해 `webui.crop_px`(480×360) 창으로 자르고 고리를 그려 `focus/<k>_<role>.jpg`로 남긴다. 점이 카메라 뒤거나 사진 밖이면 자르지 않고 그 이유를 남긴다.
3. 끝나면 `dataset.build`로 두 역할을 다시 만들고 `recon.reconstruct`로 합쳐 `out_focus/`(mesh.ply, cloud.ply, report.txt)에 쓴다. `out/`은 그대로 둔다.

## 덮인 정도(coverage)

시점마다(데이터셋의 `views.json` 표식 자세, K, 정렬 깊이 `depth/`) 꼭짓점을 투영해 사진 안이고 깊이 차가 1.2 cm 안이면
보인 것으로 센다. 점수 = 보인 시점마다 |cos(법선, 꼭짓점 → 카메라)|의 합(0.2 미만은 세지 않음). 화면은 1 미만 빨강, 3 미만 노랑,
3 이상 초록. 깊이 사진이 없는 결과(results/ 보관본)는 계산하지 않는다(404).

## 영역(regions)

화면에서 사각형으로 드래그한 영역: 사각형 안에 중심이 들고, 가로 6 px 깊이 칸에서 가장 앞(+1 cm)에 있는 삼각형(뒤에 가린 면 제외)의
중심과 보는 쪽 법선(최대 1500개). 서버(`focus.plan_region`)는 높이 `split_z` 이상이거나 위를 향한 면은 H, 아래는 L로 나누고,
로봇마다 면들이 바라보는 방향(위를 향한 면은 판 중심 → 점)을 가장 짧게 덮는 각 범위 ± 5°를 10° 간격(최대 12장)으로 찍는다.
찍기와 이동은 마커와 같다(두 로봇이 함께 이동). 자른 사진은 영역 대표점(중심에 가장 가까운 점) 둘레.

## 구멍 후보(holes)

메시 삼각형에서 한 삼각형에만 쓰인 변을 모아 연결된 고리로 묶는다. 모든 꼭짓점이 `floor_cut + 0.01 m` 아래인 고리(바닥 테두리)와 변이 8개 미만인 고리는 뺀다. 고리마다 중심, 둘레(m), 바깥 방향(판 중심 → 중심의 수평 방향)을 둘레가 긴 순서로 최대 10개.

## HTTP

모든 응답은 JSON(UTF-8). 화면 파일은 `/`, `/app.js` 등.
POST는 `Content-Type: application/json`과 JSON 객체 본문만 받고, `Origin`이 붙어 오면 `Host`와 같아야 하며, `Host` 이름은
IP나 `localhost`여야 한다(아니면 403). 다른 사이트를 연 브라우저 탭이나 DNS 재바인딩으로 로봇을 움직이지 못하게 한다.

| 요청 | 응답·동작 |
|---|---|
| `GET /api/state` | `{mode, fake, robots:{L:{addr, connected, serial, arc, odom_age_s, scan_age_s, front_m, obstacle, stack, stack_msg, stack_busy, error, preview:{…}, preview_seq, pitch_cfg_deg, z_cfg_m}}, pair:{gap_deg, gap_ok, hint, can_start, reasons}, preview_on, job:{kind, state, paused, recon, message, scan_id, progress:{phase, …}, elapsed_s, result}, last_scan, limits, recipe:{current, list}, alarms, judge, log:[…]}` |
| `POST /api/connect` | 끊긴 로봇에 다시 연결(작업 중이면 409) |
| `POST /api/robot/stack` `{"role","action":"start"\|"stop"\|"status"}` | 로봇 구동 스택 `scripts/robot.sh -H <host> <action> robot_scan`(ssh, 뒤에서). 작업 중 stop은 409, 가짜 로봇은 409 |
| `POST /api/robot/stack/start_all` | 켜져 있지 않은 로봇을 모두 켠다 |
| `POST /api/recipe` `{"name"}` | 레시피 고르기(`config/recipes.yaml`, 작업 중 409) |
| `POST /api/alarm/ack` `{"id"?}` | 알람 확인(id 없으면 전부) |
| `POST /api/preview` `{"on":bool}` | 미리보기 켜기·끄기 |
| `GET /api/preview/<role>.jpg` | 마지막 미리보기 사진(표식·가운데 십자) |
| `POST /api/scan/start` | 스캔 시작(`can_start`가 아니면 409와 이유) |
| `POST /api/job/pause`, `/api/job/resume`, `/api/job/cancel` | 진행 중인 작업에 |
| `POST /api/focus/start` `{"markers":[…], "regions":[{points, normals}…]}` | 보강 촬영 시작(400 형식, 409 작업 중·스캔 없음) |
| `GET /api/coverage?id=<결과 id>` | `{scores:[꼭짓점 순서], good, n_views, low_frac}`(404 깊이 사진 없음) |
| `GET /api/focus/<n>.jpg` | 보강 촬영 자른 사진(`job.result.crops[i].url`) |
| `GET /api/holes?id=<결과 id>` | 구멍 후보 |
| `GET /api/scans`, `GET /files?id=&name=` | 결과 목록과 파일(`tools/view_web.py`와 같음) |

## 완료 테스트(`tests/test_webui.py`, 가짜 에이전트)

| 테스트 | 통과 조건 |
|---|---|
| `test_preview_values` | 가짜 로봇 출발 자세에서 `phi_deg`가 참값과 1° 안, `d_m` 1 cm 안, `pitch_deg`가 mount 피치와 1° 안, `gap_ok` |
| `test_scan_pause_resume` | 시작 → 일시정지 → 진행 시점이 더 늘지 않음 → 재개 → 메시와 치수 |
| `test_scan_cancel_reconnect` | 시작 → 취소 → `cancelled`, 두 로봇이 `abort`를 받음 → 다시 연결되어 `ready` |
| `test_focus_plan` | 옆면 낮은 점 → L·법선 방향, 높은 점·윗면 → H, 맡은 로봇이 없으면 다른 로봇 |
| `test_focus_run` | 스캔 뒤 마커 1개 → 두 로봇이 같은 각만큼 함께(각도 차 그대로), 맡은 로봇이 3장 더 찍음, `views.json`에 `k ≥ 100`, `out_focus/mesh.ply`, 자른 사진 안에 점. 시계로 140° 떨어진 마커 → 원호 문장만으로 6번 안에 도착(제자리 회전 없음) |
| `test_focus_transit` | 이동 나누기: 반시계 170° → 85° 둘, 시계 120° → 반시계 80° 셋(사이마다 자세 다시 잡기), 시계 30° → 15° 둘, 원호를 모르는 로봇이 있거나 `transit_max_deg` 15 → 15°씩, 원에서 벗어난 자세 → 15° 뒤 나머지 |
| `test_holes` | 구멍 낸 합성 메시 → 그 구멍이 후보, 바닥 테두리는 빠짐 |
| `test_coverage_score` | 정면 평면 → 시점마다 1, 앞에서 가리면 0, 비스듬하면 덜 |
| `test_plan_region` | 낮은 옆면 → L·법선 방향 ± 5°, 높이 경계를 걸치면 L과 H, 모서리(두 면) → 0 ~ 90° 이상을 10° 간격 |
| `test_http` | 각 요청의 응답 형식, 작업 중 409, 없는 파일 404 |

## 레시피·판정·알람·구동

- 레시피(`config/recipes.yaml`, `scan3d/web/recipe.py`): `scene`·`drive`·`recon`의 정해진 값만 덮는다. 고른 레시피를 덮은 설정을
  그 스캔의 `config.yaml`에 쓰고 `recipe.json`을 남긴다(다시 만들기·다시 합치기가 같은 값을 쓴다). 보강 촬영은 그 스캔의 레시피를 쓴다.
- 판정: 레시피에 `spec_mm`·`tol_mm`이 있으면 스캔·보강이 끝날 때 잰 치수(가로·세로·높이)와 비교해 축마다 OK/NG, 전체 OK/NG →
  `judge.json`, `state.judge`.
- 알람(`scan3d/web/alarms.py`): E01 표식 미검출, E02 주행 경로 막힘, E03 로봇 통신 끊김, E04 두 로봇 각도 차, E05 재구성 실패,
  W06 장애물 감지(경고), E09 기타. 상태로 켜지고 꺼지는 것(통신 끊김, 장애물)은 key로 하나만 두고 조건이 풀리면 해제한다.
  한 번 일어난 것(작업 실패)은 확인하면 해제한다. 스스로 해제된 경고는 확인을 기다리지 않는다.
- 장애물: 로봇마다 진행 방향 LiDAR 최소 거리(`front_m`, `drive.front_min`)를 늘 보인다. 주행이 막혀 기다리거나 서면(`drive` 이벤트
  wait·stop) W06과 `obstacle`, 길이 비거나 다시 움직이면(go·leg·view·bypass) 해제한다. 작업이 없을 때 온 이벤트는 버린다.
- 구동: 화면에서 끈 로봇은 다시 켤 때까지 다시 연결하지 않는다. `robot.sh start`가 "이미 구동 스택"(다른 사람이 띄운 LiDAR·모터)으로 거절하면 `blocked`와 E09. 일부러 끈 로봇은 통신 끊김
  알람을 내지 않는다. 켠 뒤 4초쯤 지나 다시 연결한다.
