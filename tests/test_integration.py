"""I1: 가짜 에이전트 2대로 스캔 전체 흐름(PLAN 5 통합 시험)."""
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest
import yaml

from scan3d.fake_agent import BOX, FakeFleet
from scan3d.station import Clock, execute

SCALE = 5.0


def _outcomes(scan_dir, role):
    return json.loads((scan_dir / role / "outcomes.json").read_text(encoding="utf-8"))


@pytest.mark.slow
@pytest.mark.parametrize("obstacle", [None, 2])
def test_integration(cfg, tmp_path, obstacle):
    cfg_path = tmp_path / "scan.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    clock = Clock(SCALE)
    fleet = FakeFleet(cfg, clock, obstacle=obstacle)
    addrs = fleet.start()
    try:
        code, scan_dir = execute(cfg, cfg_path, addrs, clock, no_window=True, data_dir=tmp_path / "scans")
    finally:
        fleet.stop()
    scan_dir = Path(scan_dir)
    assert code == 0
    out = scan_dir / "out"
    assert (out / "mesh.ply").exists() and (out / "cloud.ply").exists()

    mesh = o3d.io.read_triangle_mesh(str(out / "mesh.ply"))
    v = np.asarray(mesh.vertices)
    c = np.asarray(mesh.vertex_colors)
    assert len(v) > 1000
    w, d, h = BOX
    rep = json.loads((scan_dir / "report.json").read_text(encoding="utf-8"))
    text = (out / "report.txt").read_text(encoding="utf-8")
    dims = [float(x) for x in rep["dims_mm"]] if "dims_mm" in rep else None
    if dims is None:
        from scan3d.recon import measure
        dims = measure(mesh)
    assert abs(dims[0] - w * 1000) <= 10 and abs(dims[1] - d * 1000) <= 10 and abs(dims[2] - h * 1000) <= 10, dims

    # +x 면(빨강): 그 면 근처 꼭짓점의 색이 빨강(RGB 순서)
    red = (v[:, 0] > w / 2 - 0.01) & (np.abs(v[:, 1]) < d / 2 - 0.02) & (v[:, 2] > 0.02) & (v[:, 2] < h - 0.02)
    assert red.sum() > 50
    mean = c[red].mean(axis=0)
    assert mean[0] > mean[1] + 0.2 and mean[0] > mean[2] + 0.2, mean

    for role in ("L", "H"):
        oc = _outcomes(scan_dir, role)
        fused = {o["k"] for o in oc if o["fused"]}
        posed = {o["k"] for o in oc if o["pose"] and o["crop"] == "ok"}
        assert fused == posed and len(fused) >= 20
        skipped = set(rep["skipped"][role])
        assert not fused & skipped
    assert rep["aborted"] is None and rep["elapsed_s"] > 0
    assert "s" in text and str(int(rep["elapsed_s"])) in text.replace(",", "")
