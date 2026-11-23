# 스캔 결과 보관

스캔 한 번의 데이터는 두 곳에 나눠 둔다.

| 위치 | 내용 | 크기 | git |
|---|---|---|---|
| `scans/<scan_id>/` | 로봇이 보낸 원본(`<역할>/raw/`), 데이터셋(`color/`, `depth/`), 결과(`out/`) 전부 | 스캔당 약 330 MB | 제외(`.gitignore`) |
| `results/<scan_id>/` | 아래 파일만 고른 보관본 | 스캔당 약 1~3 MB | 커밋 |

`scans/`는 스캔 PC(또는 `tools/upload.sh`로 옮긴 개발 PC)에만 있다. 원본으로 다시 처리해야 하면 `scans/`를 지우지 않는다.

이 폴더를 만들기 전의 결과 `out/scan_20261123-211555/`(분홍 피규어, 241 s, 138.0 × 127.8 × 202.7 mm, 색 다시 입히기 비교 포함)는 그 자리에 둔다.

## 보관하기

```
tools/archive_scan.py scans/<scan_id> --note "물체·조건 메모" --log <실행 로그>
```

| 파일 | 내용 |
|---|---|
| `summary.json` | 메모, 치수(mm), 걸린 시간, 중단 이유, 역할 |
| `out/mesh.ply`, `out/report.txt` | 색 있는 메시, 시점별 표식 수·재투영 오차·거리·융합 여부(`--cloud`면 `cloud.ply`도) |
| `report.json`, `events.jsonl` | 건너뜀·정지·보정 횟수, 진행 이벤트(시각 포함) |
| `config.yaml`, `board.json` | 그 스캔에 쓴 설정과 표식 배치 |
| `<역할>/views.json`, `outcomes.json`, `camera_intrinsic.json`, `raw/calib.json` | 시점별 `T_cam_board`, 시점 결과, 내부 파라미터, 카메라 보정값 |
| `<역할>/preview_000000.jpg` | 첫 시점 컬러 사진 |
| `run.log` | `--log`로 준 실행 로그 |

## 보기

```
tools/view_web.py --open
```

`scans/`와 `results/`를 함께 목록으로 보여 준다(`summary.json`의 메모가 이름 옆에 붙는다).

## 보관한 스캔

| scan_id | 물체 | 치수(mm) | 걸린 시간 | 비고 |
|---|---|---|---|---|
| `20261123-123251` | 돼지 인형(단단함·무광) | 139 × 128 × 202 | 263 s | 두 대 페어, L·H 24/24 융합, 정지·보정 0 |
| `20261123-124006` | 분홍 봉제 인형(털) | 102 × 91 × 126 | 274 s | 두 대 페어, L·H 24/24 융합, 털 재질이라 표면이 거칠다 |
| `20261123-125715` | 큰 곰 인형(누운 채 약 49 × 40 cm) | 486 × 404 × 220 (`out_recon`) | 288 s | 두 대 페어, 24/24 융합. 물체가 설정(지름 28 cm·높이 20 cm 이하)보다 커서 `out`은 자르기 원기둥(지름 40 cm)에서 잘림. `out_recon`은 `scene.r_obj` 0.30·`obj_h` 0.35로 다시 합친 것(설정은 `out_recon/config.yaml`) |
