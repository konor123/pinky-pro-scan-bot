"""스캔 결과 목록과 파일(docs/specs/webui.md). 표준 라이브러리만(tools/view_web.py가 호스트 python3로 쓴다).

`scans/<id>/out*/mesh.ply`(없으면 `cloud.ply`)와 `results/`(tools/archive_scan.py 보관본)를 최신순으로.
항목 id는 `<폴더 이름>/<스캔>/<out 폴더>`. 브라우저가 경로를 만들지 않아 폴더 밖으로 나갈 수 없다.
"""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NAMES = {"mesh.ply": "application/octet-stream", "cloud.ply": "application/octet-stream",
         "report.txt": "text/plain; charset=utf-8"}
OUT_DIR = re.compile(r"out(_[\w-]+)?")
DIMS = re.compile(r"dims_mm:\s*w\s*([\d.]+)\s+d\s*([\d.]+)\s+h\s*([\d.]+)")
ELAPSED = re.compile(r"elapsed_s:\s*([\d.]+)")


def default_roots(cfg=None):
    """[config/scan.yaml의 station.data_dir(못 읽으면 scans), results(tools/archive_scan.py 보관본)]."""
    try:
        if cfg is None:
            import yaml
            cfg = yaml.safe_load((ROOT / "config" / "scan.yaml").read_text(encoding="utf-8"))
        data = ROOT / cfg["station"]["data_dir"]
    except Exception:  # noqa: BLE001  yaml이 없어도 기본 폴더로 뜬다
        data = ROOT / "scans"
    return [data, ROOT / "results"]


def read_report(path):
    """report.txt → (dims_mm [w, d, h] 또는 None, elapsed_s 또는 None)."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError:
        return None, None
    m, e = DIMS.search(text), ELAPSED.search(text)
    return ([float(v) for v in m.groups()] if m else None), (float(e.group(1)) if e else None)


def _entry(eid, label, files, report):
    ply = files.get("mesh.ply") or files.get("cloud.ply")
    dims, elapsed = read_report(report) if report else (None, None)
    if report:
        files = {**files, "report.txt": report}
    return {"id": eid, "label": label, "files": files, "mtime": ply.stat().st_mtime,
            "dims_mm": dims, "elapsed_s": elapsed}


def _note(scan):
    """tools/archive_scan.py가 남긴 summary.json의 메모."""
    try:
        return json.loads((scan / "summary.json").read_text(encoding="utf-8")).get("note")
    except (OSError, ValueError):
        return None


def list_entries(roots, extra_files=()):
    """결과 목록(최신 먼저). roots는 폴더 하나 또는 여러 개(scans/, results/).

    files는 {이름: 경로}, 이름은 NAMES 안에서만.
    """
    entries = []
    roots = [roots] if isinstance(roots, (str, Path)) else roots
    for root in map(Path, roots):
        if not root.is_dir():
            continue
        for scan in root.iterdir():
            if not scan.is_dir() or scan.name.startswith("."):
                continue
            note = _note(scan)
            for out in scan.iterdir():
                if not (out.is_dir() and OUT_DIR.fullmatch(out.name)):
                    continue
                files = {n: out / n for n in ("mesh.ply", "cloud.ply") if (out / n).is_file()}
                if files:
                    report = out / "report.txt"
                    label = f"{root.name}/{scan.name} · {out.name}" + (f" — {note}" if note else "")
                    entries.append(_entry(f"{root.name}/{scan.name}/{out.name}", label, files,
                                          report if report.is_file() else None))
    for i, f in enumerate(extra_files):
        f = Path(f).resolve()
        if f.is_file():
            report = f.parent / "report.txt"
            entries.append(_entry(f"file{i}", str(f), {"mesh.ply": f}, report if report.is_file() else None))
    entries.sort(key=lambda e: e["mtime"], reverse=True)
    return entries


def public(entry):
    """브라우저에 보내는 항목(경로는 빼고 파일 이름만)."""
    out = {k: v for k, v in entry.items() if k != "files"}
    out["files"] = sorted(entry["files"])
    return out


def find(roots, eid, extra_files=()):
    """id → 항목(지금 목록에 있을 때만) 또는 None."""
    return next((e for e in list_entries(roots, extra_files) if e["id"] == eid), None)
