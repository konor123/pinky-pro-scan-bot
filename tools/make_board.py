#!/usr/bin/env python3
"""표식 판 배치 파일(config/board.json)과 A1 실제 크기 인쇄 PDF(out/board_A1.pdf)를 만든다.

명세: docs/specs/board.md
사용: .venv/bin/python tools/make_board.py
"""
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
DICTIONARY = "DICT_APRILTAG_36h11"
TAG_MM = 85.0
PITCH_MM = 115.0
GRID = 5
HOLE = 3
OBJECT_R_MM = 140.0
BOARD_MM = 594.0
PAPER_MM = (594.0, 841.0)
CELLS = 8
PT_PER_MM = 72.0 / 25.4


def _m(v_mm: float) -> float:
    return round(v_mm / 1000.0, 6)


def layout() -> dict:
    half, hole = GRID // 2, HOLE // 2
    s = TAG_MM / 2
    tags = []
    for j in range(GRID):
        for i in range(GRID):
            if abs(i - half) <= hole and abs(j - half) <= hole:
                continue
            cx, cy = (i - half) * PITCH_MM, (j - half) * PITCH_MM
            corners = [(cx - s, cy + s), (cx + s, cy + s), (cx + s, cy - s), (cx - s, cy - s)]
            tags.append({
                "id": len(tags),
                "center": [_m(cx), _m(cy)],
                "corners": [[_m(x), _m(y), 0.0] for x, y in corners],
            })
    return {
        "dictionary": DICTIONARY,
        "tag_mm": TAG_MM,
        "pitch_mm": PITCH_MM,
        "grid": GRID,
        "hole": HOLE,
        "board_mm": BOARD_MM,
        "z0": "print_surface",
        "tags": tags,
    }


def board_json_text(board: dict) -> str:
    head = {k: v for k, v in board.items() if k != "tags"}
    lines = [f'  "{k}": {json.dumps(v)},' for k, v in head.items()]
    tag_lines = ",\n".join("    " + json.dumps(t) for t in board["tags"])
    return "{\n" + "\n".join(lines) + '\n  "tags": [\n' + tag_lines + "\n  ]\n}\n"


def _tag_ops(bits, cx_mm: float, cy_mm: float) -> str:
    c = TAG_MM / CELLS
    x0, top = cx_mm - TAG_MM / 2, cy_mm + TAG_MM / 2
    ops = []
    for r in range(CELLS):
        col = 0
        while col < CELLS:
            if bits[r, col] != 0:
                col += 1
                continue
            start = col
            while col < CELLS and bits[r, col] == 0:
                col += 1
            ops.append(f"{x0 + start * c:.4f} {top - (r + 1) * c:.4f} {(col - start) * c:.4f} {c:.4f} re")
    return "\n".join(ops) + "\nf"


def _text(x: float, y: float, size: float, s: str) -> str:
    return f"BT /F1 {size:g} Tf {x:.2f} {y:.2f} Td ({s}) Tj ET"


def page_content(board: dict) -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    h = BOARD_MM / 2
    ops = [
        f"{PT_PER_MM:.9f} 0 0 {PT_PER_MM:.9f} 0 0 cm",
        f"1 0 0 1 {h:g} {h:g} cm",
        "0 g",
    ]
    for t in board["tags"]:
        bits = cv2.aruco.generateImageMarker(dictionary, t["id"], CELLS, borderBits=1)
        ops.append(_tag_ops(bits, t["center"][0] * 1000.0, t["center"][1] * 1000.0))

    n = len(board["tags"])
    span = (GRID - 1) * PITCH_MM + TAG_MM
    r, k = OBJECT_R_MM, OBJECT_R_MM * 0.5523
    ops += [
        "0 -1.5 100 3 re f",
        "100 -7 m 116 0 l 100 7 l h f",
        _text(60, 10, 9, "0\\260 +x"),
        "-1.5 0 3 70 re f",
        "-7 70 m 0 86 l 7 70 l h f",
        _text(10, 72, 9, "+y"),
        "0.6 w -15 0 m 0 0 l S 0 -15 m 0 0 l S",
        "0.5 g 0.5 w [3 3] 0 d",
        (f"{r:g} 0 m {r:g} {k:.3f} {k:.3f} {r:g} 0 {r:g} c {-k:.3f} {r:g} {-r:g} {k:.3f} {-r:g} 0 c "
         f"{-r:g} {-k:.3f} {-k:.3f} {-r:g} 0 {-r:g} c {k:.3f} {-r:g} {r:g} {-k:.3f} {r:g} 0 c S"),
        "0.4 w [4 3] 0 d",
        f"{-h:g} {h:g} m {h:g} {h:g} l S",
        "[] 0 d",
        _text(-h + 15, h + 4, 5, "cut  594 x 594 mm"),
        "0 g",
        _text(-h + 15, h + 40, 9, f"AprilTag 36h11  |  {n} tags  |  tag {TAG_MM:g} mm  |  pitch {PITCH_MM:g} mm  |  "
                                  f"span {span:g} mm  |  ids 0-{n - 1}"),
        _text(-h + 15, h + 28, 9, "Print A1 at 100% (actual size), matte paper"),
    ]
    return ("\n".join(ops) + "\n").encode("latin-1")


def pdf_bytes(content: bytes) -> bytes:
    w, h = (v * PT_PER_MM for v in PAPER_MM)
    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {w:.4f} {h:.4f}] "
         f"/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>").encode(),
        b"<< /Length " + str(len(content)).encode() + b" >>\nstream\n" + content + b"endstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>",
    ]
    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = []
    for n, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += f"{n} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(out)
    out += f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode()
    out += b"".join(f"{o:010d} 00000 n \n".encode() for o in offsets)
    out += f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    return bytes(out)


def main() -> None:
    board = layout()
    (ROOT / "config").mkdir(exist_ok=True)
    (ROOT / "out").mkdir(exist_ok=True)
    (ROOT / "config" / "board.json").write_text(board_json_text(board), encoding="utf-8")
    (ROOT / "out" / "board_A1.pdf").write_bytes(pdf_bytes(page_content(board)))
    print(f"{len(board['tags'])} tags -> config/board.json, out/board_A1.pdf")


if __name__ == "__main__":
    main()
