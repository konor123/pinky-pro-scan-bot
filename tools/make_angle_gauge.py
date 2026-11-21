#!/usr/bin/env python3
"""카메라 숙임 각도를 맞추는 A4 접이식 추 각도기 PDF(out/angle_gauge_A4.pdf)를 만든다.

각도는 config의 robots.<역할>.mount.pitch_down_deg에서 읽는다. 쪽마다 로봇 하나.
사용: /usr/bin/python3 tools/make_angle_gauge.py [--angle L=10 --angle H=35]
"""
import argparse
import math
from pathlib import Path

import cairo
import yaml

ROOT = Path(__file__).resolve().parents[1]
PAGE_MM = (210.0, 297.0)
PT_PER_MM = 72.0 / 25.4
FOLD = PAGE_MM[1] / 2
MARGIN = 10.0
FONT = "NanumBarunGothic"
RED = (0.85, 0.1, 0.1)
GRAY = (0.45, 0.45, 0.45)

PIVOT_V = 130.0
R_ARC = 96.0
R_MINOR = 91.0
R_MAJOR = 86.0
R_LABEL = R_ARC + 6
R_TARGET = R_ARC + 13
R_WEIGHT = R_ARC + 25
WEIGHT_R = 5.0
SPAN = 10
ZONE = (40.0, 14.0)


def ray(phi_deg: float) -> tuple[float, float]:
    """카드 좌표(u: 접는 선을 따라 렌즈 쪽, v: 접는 선에서 위로)에서 각 phi의 실 방향."""
    p = math.radians(phi_deg)
    return math.sin(p), -math.cos(p)


def thread_in_card(pitch_down_deg: float) -> tuple[float, float]:
    """카메라를 pitch만큼 숙였을 때 윗면에 선 카드에서 본 중력 방향."""
    t = math.radians(pitch_down_deg)
    u_axis = (math.cos(t), -math.sin(t))
    v_axis = (math.sin(t), math.cos(t))
    g = (0.0, -1.0)
    return (g[0] * u_axis[0] + g[1] * u_axis[1], g[0] * v_axis[0] + g[1] * v_axis[1])


def scale_range(theta: float) -> tuple[int, int]:
    return max(0, math.floor(theta) - SPAN), math.ceil(theta) + SPAN


def pivot_u(theta: float) -> float:
    a0, a1 = scale_range(theta)
    offs = [0.0, -ZONE[0]]
    for a in range(a0, a1 + 1):
        offs.append((R_LABEL + 3) * ray(a)[0])
    offs += [R_WEIGHT * ray(theta)[0] - WEIGHT_R, R_WEIGHT * ray(theta)[0] + WEIGHT_R]
    return PAGE_MM[0] / 2 - (min(offs) + max(offs)) / 2


def card_pt(px: float, phi: float, r: float) -> tuple[float, float]:
    du, dv = ray(phi)
    return px + r * du, PIVOT_V + r * dv


def to_sheet(u: float, v: float) -> tuple[float, float]:
    return u, FOLD - v


def check_geometry(theta: float) -> None:
    for t in (0.0, 10.0, 35.0, theta):
        gu, gv = thread_in_card(t)
        ru, rv = ray(t)
        assert abs(gu - ru) < 1e-9 and abs(gv - rv) < 1e-9, t
    assert abs(ray(0)[0]) < 1e-12 and ray(theta)[0] > 0
    px = pivot_u(theta)
    a0, a1 = scale_range(theta)
    pts = [card_pt(px, a, r) for a in range(a0, a1 + 1) for r in (R_MAJOR, R_LABEL + 3)]
    pts += [card_pt(px, theta, R_TARGET + 3)]
    assert px - ZONE[0] >= MARGIN + 12, (theta, px)
    wu, wv = card_pt(px, theta, R_WEIGHT)
    pts += [(wu - WEIGHT_R, wv - WEIGHT_R), (wu + WEIGHT_R, wv + WEIGHT_R)]
    for u, v in pts:
        assert MARGIN <= u <= PAGE_MM[0] - MARGIN and 3.0 <= v <= FOLD - MARGIN, (theta, u, v)
    for a in range(a0, a1 + 1):
        assert PIVOT_V - (R_WEIGHT + WEIGHT_R) * math.cos(math.radians(a)) >= 3.0, (theta, a)


class Page:
    def __init__(self, ctx: cairo.Context):
        self.ctx = ctx
        self.boxes: list[tuple[str, tuple[float, float, float, float]]] = []

    def text(self, s, x, y, size, bold=False, align="l", color=(0, 0, 0), middle=False, tag=None):
        c = self.ctx
        c.select_font_face(FONT, cairo.FONT_SLANT_NORMAL,
                           cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
        c.set_font_size(size)
        xb, yb, w, h, _, _ = c.text_extents(s)
        x0 = {"l": x, "c": x - w / 2, "r": x - w}[align] - xb
        y0 = y - (yb + h / 2) if middle else y
        c.set_source_rgb(*color)
        c.move_to(x0, y0)
        c.show_text(s)
        box = (x0 + xb, y0 + yb, x0 + xb + w, y0 + yb + h)
        self.boxes.append((tag or s, box))
        return box

    def wrap(self, s, x, y, size, width, leading=1.55, bold=False):
        c = self.ctx
        c.select_font_face(FONT, cairo.FONT_SLANT_NORMAL,
                           cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
        c.set_font_size(size)
        line = ""
        for word in s.split(" "):
            trial = (line + " " + word).strip()
            if line and c.text_extents(trial)[4] > width:
                self.text(line, x, y, size, bold)
                y += size * leading
                line = word
            else:
                line = trial
        if line:
            self.text(line, x, y, size, bold)
            y += size * leading
        return y

    def line(self, pts, width=0.25, color=(0, 0, 0), dash=None):
        c = self.ctx
        c.set_source_rgb(*color)
        c.set_line_width(width)
        c.set_dash(dash or [])
        c.move_to(*pts[0])
        for p in pts[1:]:
            c.line_to(*p)
        c.stroke()
        c.set_dash([])

    def poly_fill(self, pts, color):
        c = self.ctx
        c.set_source_rgb(*color)
        c.move_to(*pts[0])
        for p in pts[1:]:
            c.line_to(*p)
        c.close_path()
        c.fill()

    def circle(self, x, y, r, width=0.25, color=(0, 0, 0), dash=None):
        c = self.ctx
        c.set_source_rgb(*color)
        c.set_line_width(width)
        c.set_dash(dash or [])
        c.new_sub_path()
        c.arc(x, y, r, 0, 2 * math.pi)
        c.stroke()
        c.set_dash([])

    def arrow(self, x0, y, x1, width=0.5, head=2.5):
        self.line([(x0, y), (x1 - head, y)], width)
        self.poly_fill([(x1, y), (x1 - head * 1.6, y - head * 0.8), (x1 - head * 1.6, y + head * 0.8)], (0, 0, 0))


def draw_front(pg: Page, role: str, theta: float) -> None:
    W = PAGE_MM[0]
    px = pivot_u(theta)
    a0, a1 = scale_range(theta)
    S = lambda u, v: to_sheet(u, v)

    pg.line([S(MARGIN, 0), S(W - MARGIN, 0)], 0.35)
    pg.text("접는 선", MARGIN + 2, FOLD - 2.2, 2.8, tag="fold")
    z0, z1 = px - ZONE[0], px - ZONE[1]
    pg.poly_fill([S(z0, 0), S(z1, 0), S(z1, 1.6), S(z0, 1.6)], GRAY)
    pg.text("카메라 윗면", (z0 + z1) / 2, FOLD - 3.6, 2.8, align="c", color=GRAY, tag="zone")

    pg.text(f"{role} 로봇 카메라 숙임 각도기", W - MARGIN, 16.0, 4.5, bold=True, align="r", tag="title")
    pg.text(f"아래로 {theta:g}°", W - MARGIN, 29.0, 10.0, bold=True, align="r", color=RED, tag="title2")

    ax, ay = S(W - MARGIN - 46, 9.0)
    pg.arrow(ax, ay, W - MARGIN, 0.6, 2.4)
    pg.text("렌즈 쪽", ax + 23, ay - 3.0, 4.5, bold=True, align="c", tag="lens")

    cx, cy = S(px, PIVOT_V)
    pg.line([(cx - 3.5, cy), (cx + 3.5, cy)], 0.25)
    pg.line([(cx, cy - 3.5), (cx, cy + 3.5)], 0.25)
    pg.circle(cx, cy, 1.0, 0.2)
    pg.text("실 구멍", cx + 4.5, cy, 2.8, middle=True, tag="pivot")

    if a0 > 0:
        pg.line([S(*card_pt(px, 0, 5)), S(*card_pt(px, 0, R_ARC))], 0.25, GRAY, [1.5, 1.5])
        lx, ly = S(*card_pt(px, 0, R_ARC + 4))
        pg.text("0° 수평", lx, ly, 2.8, align="c", color=GRAY, middle=True, tag="zero")

    n = int((a1 - a0) / 0.25)
    pg.line([S(*card_pt(px, a0 + i * 0.25, R_ARC)) for i in range(n + 1)], 0.25)
    for a in range(a0, a1 + 1):
        major = a % 5 == 0
        pg.line([S(*card_pt(px, a, R_MAJOR if major else R_MINOR)), S(*card_pt(px, a, R_ARC))],
                0.3 if major else 0.2)
        if major and abs(a - theta) >= 3:
            lx, ly = S(*card_pt(px, a, R_LABEL))
            pg.text(str(a), lx, ly, 3.2, align="c", middle=True, tag=f"n{a}")

    pg.line([S(*card_pt(px, theta, 8)), S(*card_pt(px, theta, R_ARC))], 0.3, RED)
    du, dv = ray(theta)
    nu, nv = -dv, du
    tip, base = card_pt(px, theta, R_ARC + 1), card_pt(px, theta, R_ARC + 6)
    pg.poly_fill([S(*tip), S(base[0] + 2 * nu, base[1] + 2 * nv), S(base[0] - 2 * nu, base[1] - 2 * nv)], RED)
    lx, ly = S(*card_pt(px, theta, R_TARGET))
    pg.text(f"{theta:g}°", lx, ly, 4.0, bold=True, align="c", color=RED, middle=True, tag="target")

    wx, wy = S(*card_pt(px, theta, R_WEIGHT))
    pg.circle(wx, wy, WEIGHT_R, 0.25, GRAY, [1.0, 1.0])
    pg.text("클립", wx, wy, 2.6, align="c", color=GRAY, middle=True, tag="weight")


def draw_diagram(pg: Page, theta: float, box: tuple[float, float, float, float]) -> None:
    t = math.radians(theta)

    def L(d, u):
        return d * math.cos(t) + u * math.sin(t), -d * math.sin(t) + u * math.cos(t)

    piv_l = L(27, 37)
    key = [L(d, u) for d, u in [(-13, -29), (13, -29), (-13, 0), (13, 0), (-22, 0), (72, 0), (-22, 44), (72, 44)]]
    key += [(-38.0, 0.0), (80.0, 0.0), (L(0, -29)[0], L(0, -29)[1] - 12), (piv_l[0], piv_l[1] - 32)]
    lens_end = L(36, -14.5)
    key += [(lens_end[0] + 12, lens_end[1]), (63 * math.cos(-t / 2), 63 * math.sin(-t / 2))]
    xs, zs = [k[0] for k in key], [k[1] for k in key]
    bx0, by0, bx1, by1 = box
    s = min((bx1 - bx0) / (max(xs) - min(xs)), (by1 - by0) / (max(zs) - min(zs)))
    ox = bx0 + (bx1 - bx0 - s * (max(xs) - min(xs))) / 2 - s * min(xs)
    oy = by0 + s * max(zs)

    def P(d, u):
        X, Z = L(d, u)
        return ox + s * X, oy - s * Z

    def Q(X, Z):
        return ox + s * X, oy - s * Z

    pg.line([Q(-38, 0), Q(80, 0)], 0.2, GRAY, [1.2, 1.2])
    pg.text("수평", Q(-38, 0)[0], Q(-38, 0)[1] - 1.5, 2.6, color=GRAY, tag="d-level")
    pg.line([P(0, -29), (P(0, -29)[0], P(0, -29)[1] + 12 * s)], 0.8, GRAY)
    pg.line([P(-13, 0), P(13, 0), P(13, -29), P(-13, -29), P(-13, 0)], 0.4)
    lens = P(13, -14.5)
    pg.circle(lens[0], lens[1], 2.0 * s, 0.3)
    pg.line([lens, P(36, -14.5)], 0.25)
    pg.text("렌즈", P(36, -14.5)[0] + 1.0, P(36, -14.5)[1], 2.8, middle=True, tag="d-lens")
    pg.line([P(-22, 0), P(72, 0), P(72, 44), P(-22, 44), P(-22, 0)], 0.3)
    piv = P(27, 37)
    pg.circle(piv[0], piv[1], 0.7, 0.3)
    end = (piv[0], piv[1] + 28 * s)
    pg.line([piv, end], 0.3, RED)
    pg.poly_fill([(end[0] - 1.0, end[1]), (end[0] + 1.0, end[1]), (end[0] + 1.0, end[1] + 3.5), (end[0] - 1.0, end[1] + 3.5)], RED)
    r = 60.0
    pg.line([Q(r * math.cos(-a * t / 20), r * math.sin(-a * t / 20)) for a in range(21)], 0.3, RED)
    mid = Q((r + 3) * math.cos(-t / 2), (r + 3) * math.sin(-t / 2))
    pg.text(f"{theta:g}°", mid[0] + 0.8, mid[1] + 1.0, 2.8, bold=True, color=RED, middle=True, tag="d-angle")
    y = by1 + 6.0
    for ln in ("옆에서 본 모습: 카드를 카메라", "윗면에 세우면 실이 렌즈 쪽으로", "숙인 각만큼 기운다"):
        pg.text(ln, bx0, y, 2.8, color=GRAY, tag="d-cap" + ln[:2])
        y += 4.2


def draw_back(pg: Page, theta: float) -> None:
    x, width = 16.0, 102.0
    y = FOLD + 20.0
    pg.text("만들기", x, y, 5.0, bold=True)
    y += 8.0
    y = pg.wrap("준비물: 실 30 cm, 바늘이나 핀, 작은 클립, 풀이나 테이프", x, y, 3.3, width) + 1.0
    steps = [
        "1. 가운데 접는 선을 따라 인쇄면이 바깥으로 오게 반으로 접는다. 종이 끝이 아니라 선에 맞춰 접고, 두 겹을 풀이나 테이프로 붙인다.",
        "2. 앞면의 + 표시를 바늘로 뚫어 실을 꿰고, 뒤쪽에서 매듭을 지어 테이프로 붙인다.",
        "3. 실 끝에 클립을 묶고, 클립이 앞면의 점선 동그라미에 오도록 실 길이를 맞춘다.",
    ]
    for st in steps:
        y = pg.wrap(st, x, y, 3.3, width) + 1.0
    y += 5.0
    pg.text("맞추기", x, y, 5.0, bold=True)
    y += 8.0
    steps = [
        "4. 앞면의 '카메라 윗면' 표시 자리를 카메라 윗면에 앞뒤 방향으로 대고, 렌즈 쪽 화살표가 렌즈를 향하게 한다.",
        "5. 실이 종이에 살짝 닿을 만큼 카드 위쪽을 보는 사람 반대쪽으로 조금 기울이고, 실이 멈추면 눈금을 읽는다.",
        f"6. 실이 빨간 선({theta:g}°)에 오도록 카메라를 숙인다. 고정 나사를 조인 뒤 다시 읽는다.",
    ]
    for st in steps:
        y = pg.wrap(st, x, y, 3.3, width) + 1.0
    draw_diagram(pg, theta, (126.0, FOLD + 16.0, PAGE_MM[0] - MARGIN - 2, FOLD + 92.0))


def check_page(pg: Page, front_tags: set) -> None:
    W, H = PAGE_MM
    for tag, (x0, y0, x1, y1) in pg.boxes:
        assert MARGIN - 0.5 <= x0 and x1 <= W - MARGIN + 0.5 and MARGIN - 0.5 <= y0 and y1 <= H - MARGIN + 0.5, (tag, x0, y0, x1, y1)
        if tag in front_tags:
            assert y1 <= FOLD + 0.01, (tag, y1)
        else:
            assert y0 >= FOLD, (tag, y0)
    for i, (ta, a) in enumerate(pg.boxes):
        for tb, b in pg.boxes[i + 1:]:
            gap = 0.3
            overlap = a[0] < b[2] + gap and b[0] < a[2] + gap and a[1] < b[3] + gap and b[1] < a[3] + gap
            assert not overlap, (ta, tb)


def load_angles(config: Path) -> dict[str, float]:
    cfg = yaml.safe_load(config.read_text(encoding="utf-8"))
    return {role: float(r["mount"]["pitch_down_deg"]) for role, r in cfg["robots"].items()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=ROOT / "config" / "scan.yaml")
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "angle_gauge_A4.pdf")
    ap.add_argument("--angle", action="append", default=[], metavar="역할=도")
    args = ap.parse_args()

    angles = load_angles(args.config)
    for item in args.angle:
        role, val = item.split("=")
        angles[role] = float(val)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    surface = cairo.PDFSurface(str(args.out), PAGE_MM[0] * PT_PER_MM, PAGE_MM[1] * PT_PER_MM)
    surface.set_metadata(cairo.PDFMetadata.TITLE, "카메라 숙임 각도기")
    ctx = cairo.Context(surface)
    for role, theta in angles.items():
        check_geometry(theta)
        ctx.save()
        ctx.scale(PT_PER_MM, PT_PER_MM)
        pg = Page(ctx)
        draw_front(pg, role, theta)
        front_tags = {t for t, _ in pg.boxes}
        draw_back(pg, theta)
        check_page(pg, front_tags)
        ctx.restore()
        ctx.show_page()
    surface.finish()
    print(", ".join(f"{r} {a:g}°" for r, a in angles.items()), "->", args.out.relative_to(ROOT) if args.out.is_relative_to(ROOT) else args.out)


if __name__ == "__main__":
    main()
