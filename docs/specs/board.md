# board

- 코드: `tools/make_board.py`(배치 파일과 인쇄 PDF 생성). 판 객체 만들기는 tagpose
- 테스트: `tests/test_board.py`(pdftoppm이 설치된 개발 PC. synth·tagpose를 쓰지 않는다)
- PLAN: 3.1, 4.1, 5
- 출처 예제: opencv_contrib 4.11.0 `cv2.aruco`(`DICT_APRILTAG_36h11`, `generateImageMarker`, `Board`)

## 하는 일

표식 판의 설계값을 배치 파일 `config/board.json`으로 정하고, 같은 값으로 실제 크기 인쇄 파일 `out/board_A1.pdf`를 만든다. 인쇄한 판이 판 좌표계 B를 정의한다.

## 판 설계

- 표식: AprilTag 36h11(`DICT_APRILTAG_36h11`), id 0 ~ 15
- 표식 한 변 85 mm: 검은 테두리 바깥 정사각형의 한 변(8×8칸, 한 칸 10.625 mm)
- 배치: 5×5 격자(간격 115 mm)의 바깥 둘레 16칸. 표식 중심 `(i·115, j·115)` mm, i, j = −2…2, `max(|i|, |j|) = 2`. 표식이 덮는 범위는 ±272.5 mm이고, 가운데는 ±187.5 mm까지 비어 있다(물체 바닥 윤곽 반경 0.14 m + 놓는 오차 0.03 m + 여유). 표식 사이 간격 30 mm
- id 순서: j = −2(−y 쪽 줄)부터 위로, 한 줄 안에서는 i = −2(−x 쪽)부터 +x 쪽으로 0, 1, 2, …
- 표식 방향: 위에서 내려다볼 때 표식 영상의 위쪽이 +y, 오른쪽이 +x
- 판 좌표계 B: 원점은 판 가운데의 인쇄면(z = 0, 물체가 놓이는 면), x축은 `0° +x` 화살표 방향, z축은 위, y = z × x. 용지를 바닥에 바로 펴 놓으므로 바닥도 z ≈ 0이다.
- 판 표시: 원점에서 `0° +x` 화살표(116 mm)와 `+y` 화살표(86 mm), 물체 자리 점선 원(반지름 140 mm). 표시는 표식에서 47 mm 이상 떨어져 있다.
- 용지: A1 세로(594 × 841 mm). 판은 아래쪽 594 × 594 mm이고 판 중심이 용지 왼쪽 아래 모서리에서 (297, 297) mm다. 위쪽 247 mm는 잘라 내고, 자르는 선을 점선으로 표시한다. 표식은 용지 가장자리에서 24.5 mm 안쪽에 있다. 벡터 PDF, 100% 크기로 출력. 잘라 낸 594 × 594 mm를 평평한 바닥에 바로 펴 놓는다.

## 배치 파일 `config/board.json`

```json
{"dictionary": "DICT_APRILTAG_36h11", "tag_mm": 85.0, "pitch_mm": 115.0, "grid": 5, "hole": 3,
 "board_mm": 594.0, "z0": "print_surface",
 "tags": [{"id": 0, "center": [-0.23, -0.23], "corners": [[x, y, 0.0], [x, y, 0.0], [x, y, 0.0], [x, y, 0.0]]}, …]}
```

- `center`, `corners`의 단위는 m(판 좌표 B)
- `corners` 순서는 OpenCV 검출 모서리 순서와 같은 표식 영상의 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래다. 판 좌표로는 `(cx−s/2, cy+s/2)`, `(cx+s/2, cy+s/2)`, `(cx+s/2, cy−s/2)`, `(cx−s/2, cy−s/2)`(s = 0.085)
- tagpose는 `corners`로 `cv2.aruco.Board`를 만들고, 모든 좌표에 `config` `board.tag_mm` / 배치 파일 `tag_mm`을 곱해 인쇄 배율 오차를 보정한다.

## 실행

`.venv/bin/python tools/make_board.py` → `config/board.json`, `out/board_A1.pdf`. 같은 입력이면 같은 파일이 나온다.

## 현장 확인

출력물을 잘라 바닥에 편 뒤 가로·세로 각각 바깥 표식의 바깥 끝에서 끝까지 잰다(설계 545 mm). 두 값의 차이가 2 mm를 넘으면 다시 출력한다. 두 값의 평균 × 85 / 545를 `board.tag_mm`에 적는다.

## 완료 테스트

| 테스트 함수 | 입력 | 통과 조건 |
|---|---|---|
| `test_layout` | `board.json` | 16장, id 0 ~ 15 한 번씩, 모서리가 위 식과 같음, 가운데 ±187.5 mm 안에 표식 없음 |
| `test_pdf_detect` | PDF를 pdftoppm으로 4 px/mm 래스터화(픽셀 중심 = 정수 좌표 + 0.5 → mm) | 16장 모두 검출, 검출 모서리가 `board.json`과 0.5 mm 안 |
| `test_pdf_pose` | 래스터를 알려진 자세의 핀홀 카메라로 투영(테스트 안의 작은 렌더러) | `board.json`으로 구한 PnP 자세가 위치 3 mm, 각도 1° 안 |
