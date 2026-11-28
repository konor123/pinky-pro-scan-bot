"""URDF 내보내기·파싱·Gazebo 로드 도구 테스트."""
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

from scan3d import urdf
from scan3d.config import ROOT


def _sample_box_arrays(w=0.18, d=0.14, h=0.16, offset=(0.02, -0.01, 0.005)):
    ox, oy, oz = offset
    hx, hy = w / 2.0, d / 2.0
    v = np.array([
        [ox - hx, oy - hy, oz], [ox + hx, oy - hy, oz], [ox + hx, oy + hy, oz], [ox - hx, oy + hy, oz],
        [ox - hx, oy - hy, oz + h], [ox + hx, oy - hy, oz + h], [ox + hx, oy + hy, oz + h], [ox - hx, oy + hy, oz + h],
    ], dtype=np.float64)
    t = np.array([
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
        [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7],
    ], dtype=np.int32)
    c = np.tile([0.8, 0.2, 0.3], (len(v), 1))
    return v, t, c


def test_export_and_load_urdf_roundtrip(tmp_path):
    v, t, c = _sample_box_arrays()
    info = urdf.export_urdf((v, t, c), tmp_path, name="test-box", mass_kg=0.35)
    assert info["urdf"].is_file() and info["stl"].is_file()
    assert info["name"] == "test_box"

    # XML 구조와 관성·메시 태그 확인
    root = ET.fromstring(info["urdf"].read_text(encoding="utf-8"))
    assert root.tag == "robot" and root.attrib["name"] == "test_box"
    assert root.find(".//visual/geometry/mesh").attrib["filename"] == "mesh.stl"
    assert root.find(".//collision/geometry/mesh").attrib["filename"] == "mesh.stl"
    assert math.isclose(float(root.find(".//inertial/mass").attrib["value"]), 0.35, abs_tol=1e-4)

    # STL + URDF 로더 확인: XY 중심(0,0), 바닥 z=0 정렬
    model = urdf.load_urdf(info["urdf"])
    assert model.name == "test_box"
    assert math.isclose(model.mass_kg, 0.35, abs_tol=1e-4)
    assert np.allclose(model.dims_m, [0.18, 0.14, 0.16], atol=1e-4)
    assert abs(model.vertices[:, 2].min()) < 1e-5
    assert abs((model.vertices[:, 0].min() + model.vertices[:, 0].max()) / 2) < 1e-5
    assert abs((model.vertices[:, 1].min() + model.vertices[:, 1].max()) / 2) < 1e-5
    assert len(model.triangles) == len(t)


def test_real_archived_ply_to_urdf(tmp_path):
    """실제 스캔 보관본(results/20261123-123251/out/mesh.ply)으로 URDF 생성 및 로드."""
    ply = ROOT / "results" / "20261123-123251" / "out" / "mesh.ply"
    if not ply.is_file():
        return
    # 복사 후 원본 폴더는 건드리지 않고 tmp_path에서 변환
    dst = tmp_path / "out"
    dst.mkdir()
    (dst / "mesh.ply").write_bytes(ply.read_bytes())
    urdf_path = urdf.ensure_urdf(dst, name="pig_doll")
    assert urdf_path.is_file() and (dst / "mesh.stl").is_file()
    model = urdf.load_urdf(urdf_path)
    assert model.name == "pig_doll"
    assert model.color_ply_path == dst / "mesh.ply"
    assert model.colors is not None and len(model.colors) == len(model.vertices)
    assert 0.10 < model.dims_m[0] < 0.20 and 0.15 < model.dims_m[2] < 0.25


def test_spawn_urdf_dry_run(tmp_path):
    v, t, c = _sample_box_arrays()
    info = urdf.export_urdf((v, t, c), tmp_path, name="spawn_test")
    p = subprocess.run([sys.executable, str(ROOT / "tools" / "spawn_urdf.py"),
                        str(info["urdf"]), "--world", "scan3d_room", "--dry-run"],
                       capture_output=True, text=True, check=True)
    assert "file://" in p.stdout and "mesh.stl" in p.stdout
    assert "ros_gz_sim create -world scan3d_room -name spawn_test" in p.stdout
