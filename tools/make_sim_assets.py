#!/usr/bin/env python3
"""Gazebo 시뮬레이션용 표식 판 텍스처(build/sim/board.png)를 만든다. scripts/sim_scan.sh가 부른다."""
import argparse
import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scan3d.synth import board_texture  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOARD = ROOT / "config" / "board.json"
DEFAULT_OUT_DIR = ROOT / "build" / "sim"


def build_board_texture(board_path: Path, out_path: Path, px_per_mm: int = 4) -> Path:
    """`config/board.json`에서 3채널 BGR PNG 표식 판 텍스처를 만든다."""
    board = json.loads(Path(board_path).read_text(encoding="utf-8"))
    gray = board_texture(board, px_per_mm=int(px_per_mm))
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), bgr):
        raise RuntimeError(f"failed to write texture image: {out}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Gazebo 시뮬레이션용 자산 생성")
    ap.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--px-per-mm", type=int, default=4)
    args = ap.parse_args(argv)
    out_png = build_board_texture(args.board, args.out_dir / "board.png", px_per_mm=args.px_per_mm)
    print(f"texture -> {out_png}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
