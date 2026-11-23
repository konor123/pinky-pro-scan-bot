#!/usr/bin/env python3
"""스캔 결과를 git에 남길 크기로 골라 `results/<scan_id>/`에 복사한다. 표준 라이브러리만 쓴다.

`scans/<id>/`(원본 포함 수백 MB, git 제외)에서 메시·보고서·시점별 자세·설정만 골라 수 MB로 남긴다.
`out/`과 함께 `python -m scan3d.pair.recon`으로 다시 합친 `out_*/`(out_recon 등)의 메시·보고서도 넣는다.
폴더 구조는 원래와 같아서 `tools/view_web.py`와 `python -m scan3d.pair.recon`의 보고서 형식이 그대로 맞는다.
원본(raw/)이 필요하면 `scans/`를 그대로 두거나 `tools/upload.sh`로 옮긴다.

  tools/archive_scan.py scans/20261123-123251 --note "돼지 인형, 두 대 페어" --log out/pair_scan_run.log
  tools/archive_scan.py scans/<id> --cloud      cloud.ply도 넣는다
"""
import argparse
import json
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOP_FILES = ("config.yaml", "board.json", "events.jsonl", "report.json")
OUT_FILES = ("mesh.ply", "report.txt")
ROLE_FILES = ("views.json", "outcomes.json", "camera_intrinsic.json", "raw/calib.json")
OUT_DIR = re.compile(r"out(_[\w-]+)?")   # out/, 다시 합친 out_recon/·out_color/ 등


def roles(scan_dir):
    return sorted(p.name for p in scan_dir.iterdir() if p.is_dir() and (p / "raw").is_dir())


def plan(scan_dir, cloud=False):
    """복사할 (원본, 보관 폴더 기준 상대 경로) 목록. 메시가 없으면 ValueError."""
    scan_dir = Path(scan_dir)
    if not (scan_dir / "out" / "mesh.ply").is_file():
        raise ValueError(f"{scan_dir}/out/mesh.ply 없음(재구성이 끝난 스캔만 보관한다)")
    items = [(scan_dir / n, Path(n)) for n in TOP_FILES]
    outs = sorted(p.name for p in scan_dir.iterdir() if p.is_dir() and OUT_DIR.fullmatch(p.name))
    names = OUT_FILES + (("cloud.ply",) if cloud else ())
    items += [(scan_dir / o / n, Path(o) / n) for o in outs for n in names]
    for role in roles(scan_dir):
        items += [(scan_dir / role / n, Path(role) / n) for n in ROLE_FILES]
        # 미리보기: 융합한 첫 시점의 컬러 사진 한 장
        color = sorted((scan_dir / role / "color").glob("*.jpg"))
        if color:
            items.append((color[0], Path(role) / f"preview_{color[0].name}"))
    return [(src, rel) for src, rel in items if src.is_file()]


def archive(scan_dir, dest_root, note=None, logs=(), cloud=False, force=False):
    """`dest_root/<scan_id>/`를 만들고 그 경로를 돌려준다."""
    scan_dir = Path(scan_dir).resolve()
    items = plan(scan_dir, cloud)
    dest = Path(dest_root) / scan_dir.name
    if dest.exists():
        if not force:
            raise FileExistsError(f"{dest} 있음(덮어쓰려면 --force)")
        shutil.rmtree(dest)
    for src, rel in items:
        (dest / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest / rel)
    for i, log in enumerate(logs):
        shutil.copy2(log, dest / ("run.log" if i == 0 else f"run{i + 1}.log"))
    report = {}
    if (scan_dir / "report.json").is_file():
        report = json.loads((scan_dir / "report.json").read_text(encoding="utf-8"))
    summary = {"scan_id": scan_dir.name, "note": note, "dims_mm": report.get("dims_mm"),
               "elapsed_s": report.get("elapsed_s"), "aborted": report.get("aborted"), "roles": roles(scan_dir)}
    (dest / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return dest


def main(argv=None):
    ap = argparse.ArgumentParser(description="스캔 결과를 results/에 보관한다")
    ap.add_argument("scan_dir", help="scans/<id>")
    ap.add_argument("--dest", default=str(ROOT / "results"), help="보관 폴더(기본: 저장소 results/)")
    ap.add_argument("--note", help="물체·조건 메모(summary.json에 남김)")
    ap.add_argument("--log", action="append", default=[], help="함께 남길 실행 로그")
    ap.add_argument("--cloud", action="store_true", help="cloud.ply도 넣는다")
    ap.add_argument("--force", action="store_true", help="이미 있으면 덮어쓴다")
    args = ap.parse_args(argv)
    try:
        dest = archive(args.scan_dir, args.dest, args.note, args.log, args.cloud, args.force)
    except (ValueError, FileExistsError) as e:
        print(e, file=sys.stderr)
        return 1
    size = sum(p.stat().st_size for p in dest.rglob("*") if p.is_file())
    print(f"보관: {dest} ({size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
