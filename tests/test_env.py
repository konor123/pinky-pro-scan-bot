"""개발 PC·스캔 PC 환경 확인. scripts/setup_station.sh가 이 파일만 돌린다."""
import importlib.metadata as md
import re
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _pins() -> dict:
    with open(ROOT / "pyproject.toml", "rb") as f:
        deps = tomllib.load(f)["project"]["dependencies"]
    return dict(m.groups() for d in deps if (m := re.fullmatch(r"([\w.-]+)==([\w.]+)", d)))


@pytest.mark.parametrize("dist,version", sorted(_pins().items()))
def test_pinned_version(dist, version):
    assert md.version(dist) == version


def test_aruco_apriltag_36h11():
    import cv2

    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    assert d.bytesList.shape[0] == 587
    cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def test_realsense_software_device_and_align():
    import pyrealsense2 as rs

    dev = rs.software_device()
    dev.add_sensor("depth")
    rs.align(rs.stream.color)


def test_open3d_raycast_and_tsdf():
    import open3d as o3d

    box = o3d.geometry.TriangleMesh.create_box(0.2, 0.2, 0.18)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(box))
    rays = o3d.core.Tensor([[0.1, 0.1, 1.0, 0.0, 0.0, -1.0]], dtype=o3d.core.float32)
    assert scene.cast_rays(rays)["t_hit"].numpy()[0] == pytest.approx(0.82, abs=1e-5)
    o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
        attr_channels=((1), (1), (3)),
        voxel_size=0.003,
        block_resolution=16,
        block_count=1000,
        device=o3d.core.Device("CPU:0"),
    )


def test_vendor_commands():
    from scan3d.vendor.pinky_commands import parse, sector_min

    assert [s.kind for s in parse("turn 7.5; forward 0.17; turn 7.5")] == ["turn", "move", "turn"]
    with pytest.raises(ValueError):
        parse("turn 0; forward 0.17")
    # 장착 yaw π: 스캔 각 π(라이다 뒤쪽)가 로봇 정면
    ranges = [5.0] * 360
    ranges[180] = 0.5
    assert sector_min(ranges, 0.0, 3.14159265 / 180, 0.0, 0.1, mount_yaw=3.14159265) == 0.5


def test_config_groups(cfg):
    groups = {"board", "scene", "capture", "drive", "lidar", "robots", "recon", "station"}
    assert groups <= cfg.keys()
    assert set(cfg["robots"]) == {"L", "H"}
