"""tools/archive_scan.py: 스캔에서 보관할 파일만 골라 results/로."""
import json

import pytest

from tools import archive_scan, view_web


def _fake_scan(root):
    scan = root / "20261123-123251"
    for rel, data in {
        "config.yaml": "scene: {}\n", "board.json": "{}", "events.jsonl": "{}\n",
        "report.json": json.dumps({"dims_mm": [138.7, 127.5, 202.0], "elapsed_s": 263.3, "aborted": None}),
        "out/mesh.ply": "ply mesh", "out/cloud.ply": "ply cloud",
        "out/report.txt": "dims_mm: w 138.7  d 127.5  h 202.0\n",
    }.items():
        (scan / rel).parent.mkdir(parents=True, exist_ok=True)
        (scan / rel).write_text(data, encoding="utf-8")
    for role in ("L", "H"):
        for rel in ("views.json", "outcomes.json", "camera_intrinsic.json", "raw/calib.json",
                    "color/000000.jpg", "color/000001.jpg", "depth/000000.png"):
            (scan / role / rel).parent.mkdir(parents=True, exist_ok=True)
            (scan / role / rel).write_text(rel, encoding="utf-8")
        (scan / role / "raw" / "000001_depth.z16").write_bytes(b"\0" * 100)
    return scan


def test_archive(tmp_path):
    scan = _fake_scan(tmp_path / "scans")
    log = tmp_path / "run.log"
    log.write_text("치수(mm): 139 × 128 × 202\n", encoding="utf-8")
    dest = archive_scan.archive(scan, tmp_path / "results", note="돼지 인형", logs=[log])
    got = sorted(str(p.relative_to(dest)) for p in dest.rglob("*") if p.is_file())
    assert got == sorted([
        "config.yaml", "board.json", "events.jsonl", "report.json", "summary.json", "run.log",
        "out/mesh.ply", "out/report.txt",
        *[f"{r}/{n}" for r in ("L", "H") for n in ("views.json", "outcomes.json", "camera_intrinsic.json",
                                                    "raw/calib.json", "preview_000000.jpg")],
    ])
    summary = json.loads((dest / "summary.json").read_text(encoding="utf-8"))
    assert summary == {"scan_id": "20261123-123251", "note": "돼지 인형", "dims_mm": [138.7, 127.5, 202.0],
                       "elapsed_s": 263.3, "aborted": None, "roles": ["H", "L"]}
    # 뷰어가 보관본을 그대로 읽는다
    entries = view_web.list_entries(tmp_path / "results")
    assert entries[0]["id"] == "results/20261123-123251/out" and entries[0]["label"].endswith("돼지 인형")


def test_archive_cloud_and_force(tmp_path):
    scan = _fake_scan(tmp_path / "scans")
    dest = archive_scan.archive(scan, tmp_path / "results")
    with pytest.raises(FileExistsError):
        archive_scan.archive(scan, tmp_path / "results")
    dest = archive_scan.archive(scan, tmp_path / "results", cloud=True, force=True)
    assert (dest / "out" / "cloud.ply").read_text() == "ply cloud"


def test_archive_recon_outputs(tmp_path):
    scan = _fake_scan(tmp_path / "scans")
    (scan / "out_recon").mkdir()
    (scan / "out_recon" / "mesh.ply").write_text("ply recon", encoding="utf-8")
    (scan / "out_recon" / "report.txt").write_text("dims_mm: w 485.8  d 403.8  h 220.0\n", encoding="utf-8")
    (scan / "outside").mkdir()
    (scan / "outside" / "mesh.ply").write_text("x", encoding="utf-8")
    dest = archive_scan.archive(scan, tmp_path / "results")
    assert (dest / "out_recon" / "mesh.ply").read_text() == "ply recon"
    assert not (dest / "outside").exists()
    ids = [e["id"] for e in view_web.list_entries(tmp_path / "results")]
    assert sorted(ids) == ["results/20261123-123251/out", "results/20261123-123251/out_recon"]


def test_archive_needs_mesh(tmp_path):
    scan = _fake_scan(tmp_path / "scans")
    (scan / "out" / "mesh.ply").unlink()
    assert archive_scan.main([str(scan), "--dest", str(tmp_path / "results")]) == 1
    assert not (tmp_path / "results").exists()
