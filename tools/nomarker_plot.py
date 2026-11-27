#!/usr/bin/env python3
"""tools/nomarker_compare.py 결과를 그림으로: 위에서 본 카메라 위치(표식 ● / 바퀴 주행거리 ×)와
시점마다 물체 점을 판 좌표로 겹친 옆모습(왼쪽 표식 자세, 오른쪽 바퀴 주행거리 자세).

  tools/nomarker_plot.py <출력 폴더> L=scans/<id> H=scans/<id> [--title "<제목>"]

출력: fig_camera_paths.png, fig_points_side.png. --title을 주면 한글 제목·범례를 붙인 figure.png도 만든다
(Pillow와 Noto Sans CJK 글꼴 필요).
"""
import sys, json
from pathlib import Path
import numpy as np, cv2, yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scan3d.pair import recon
argv = sys.argv[1:]
title = None
if "--title" in argv:
    i = argv.index("--title"); title = argv[i + 1]; argv = argv[:i] + argv[i + 2:]
out = Path(argv[0]); spec = dict(a.split("=", 1) for a in argv[1:]); roles = list(spec)
cfg = yaml.safe_load((Path(spec[roles[0]]) / "config.yaml").read_text())
COL = {"L": (196, 120, 20), "H": (60, 60, 220)}   # BGR: L 파랑 계열, H 빨강 계열
def center(T): T = np.asarray(T, float); return -T[:3, :3].T @ T[:3, 3]
data = {}
for r in roles:
    tv = json.loads((Path(spec[r]) / r / "views.json").read_text())
    ov = {v["k"]: v for v in json.loads((out / "odom" / r / "views.json").read_text())}
    data[r] = [(v, ov[v["k"]]) for v in tv]
# (1) 위에서 본 카메라 위치
S, W = 300, 640
img = np.full((W, W, 3), 255, np.uint8); c0 = W // 2
P = lambda x, y: (int(c0 + x * S), int(c0 - y * S))
cv2.rectangle(img, P(-0.297, 0.297), P(0.297, -0.297), (200, 200, 200), 1)
cv2.circle(img, P(0, 0), int(0.62 * S), (210, 210, 210), 1, cv2.LINE_AA)
for r in roles:
    for tv, ov in data[r]:
        a, b = center(tv["T_cam_board"]), center(ov["T_cam_board"])
        cv2.line(img, P(*a[:2]), P(*b[:2]), (170, 170, 170), 1, cv2.LINE_AA)
        cv2.circle(img, P(*a[:2]), 4, COL[r], -1, cv2.LINE_AA)
        x, y = P(*b[:2]); cv2.drawMarker(img, (x, y), COL[r], cv2.MARKER_TILTED_CROSS, 9, 2, cv2.LINE_AA)
cv2.imwrite(str(out / "fig_camera_paths.png"), img)
# (2) 물체 점 겹치기(옆모습): 시점마다 가장자리 지운 깊이를 판 좌표로, 판 위 0.01 ~ 0.3 m · 반지름 0.5 m 안
def pts(role_dir, T, k, K):
    d = cv2.imread(str(role_dir / "depth" / f"{k:06d}.png"), cv2.IMREAD_UNCHANGED)
    d = recon.drop_depth_edges(d, 10)[::4, ::4]
    v, u = np.nonzero(d); z = d[v, u] / 1000.0; u, v = u * 4, v * 4
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], -1)
    Tb = np.linalg.inv(np.asarray(T, float)); pb = pc @ Tb[:3, :3].T + Tb[:3, 3]
    keep = (pb[:, 2] > 0.01) & (pb[:, 2] < 0.3) & (np.hypot(pb[:, 0], pb[:, 1]) < 0.5)
    return pb[keep]
panels = []
for key in ("T_tag", "T_odom"):
    can = np.full((360, 520, 3), 255, np.uint8)
    for r in roles:
        rd = Path(spec[r]) / r
        K = np.asarray(json.loads((rd / "camera_intrinsic.json").read_text())["intrinsic_matrix"], float).reshape(3, 3).T
        for tv, ov in data[r]:
            T = tv["T_cam_board"] if key == "T_tag" else ov["T_cam_board"]
            p = pts(rd, T, tv["k"], K)
            x = (260 + p[:, 0] * 500).astype(int); y = (340 - p[:, 2] * 1000).astype(int)
            ok = (x >= 0) & (x < 520) & (y >= 0) & (y < 360)
            can[y[ok], x[ok]] = COL[r]
    cv2.line(can, (0, 340), (520, 340), (150, 150, 150), 1)
    panels.append(can)
cv2.imwrite(str(out / "fig_points_side.png"), np.hstack([panels[0], np.full((360, 8, 3), 230, np.uint8), panels[1]]))


def figure(out, title):
    from PIL import Image, ImageDraw, ImageFont
    F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
    f22, f18, f16 = ImageFont.truetype(FB, 24), ImageFont.truetype(F, 18), ImageFont.truetype(F, 16)
    paths = Image.open(str(out / "fig_camera_paths.png")).convert("RGB").crop((60, 20, 580, 560))
    pts = Image.open(str(out / "fig_points_side.png")).convert("RGB")
    pw = 1048 - 520 - 30; ps = pts.resize((pw, int(pts.height * pw / pts.width)))
    W, H = 1100, 660
    im = Image.new("RGB", (W, H), "white"); dr = ImageDraw.Draw(im)
    dr.text((24, 18), title, font=f22, fill=(22, 24, 29))
    im.paste(paths, (24, 70))
    dr.text((24, 618), "위에서 본 카메라 위치(시점 24개씩)", font=f16, fill=(91, 97, 112))
    x0 = 24 + 520 + 30
    im.paste(ps, (x0, 110))
    half = pw // 2
    dr.text((x0, 80), "표식으로 구한 자세", font=f18, fill=(22, 24, 29))
    dr.text((x0 + half + 6, 80), "바퀴 주행거리로 구한 자세", font=f18, fill=(22, 24, 29))
    dr.text((x0, 110 + ps.height + 8), "48장의 물체 점을 판 좌표로 겹친 옆모습", font=f16, fill=(91, 97, 112))
    ly = 110 + ps.height + 60
    L, Hc = (20, 120, 196), (220, 60, 60)
    for i, (c, name) in enumerate(((L, "L(낮은 카메라)"), (Hc, "H(높은 카메라)"))):
        y = ly + i * 30
        dr.ellipse((x0, y + 5, x0 + 10, y + 15), fill=c); dr.text((x0 + 18, y - 2), name, font=f16, fill=(22, 24, 29))
    y = ly + 70
    dr.ellipse((x0, y + 5, x0 + 10, y + 15), fill=(120, 120, 120)); dr.text((x0 + 18, y - 2), "표식으로 구한 위치", font=f16, fill=(22, 24, 29))
    y += 30
    dr.line((x0, y + 4, x0 + 10, y + 14), fill=(120, 120, 120), width=3); dr.line((x0, y + 14, x0 + 10, y + 4), fill=(120, 120, 120), width=3)
    dr.text((x0 + 18, y - 2), "바퀴 주행거리로 구한 위치(출발 위치만 앎)", font=f16, fill=(22, 24, 29))
    im.save(str(out / "figure.png"), optimize=True)


if title:
    figure(out, title)
