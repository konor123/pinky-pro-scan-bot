#!/usr/bin/env python3
"""스캔 산출물 URDF(`object.urdf`)를 실행 중인 Gazebo 월드에 로드한다.

  tools/spawn_urdf.py scans/<id>/out/object.urdf --world scan3d_room --at 0 0
  tools/spawn_urdf.py scans/<id>/out/mesh.ply --world scan3d_room --dry-run
"""
import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scan3d import urdf as urdf_mod


def resolve_urdf_xml(urdf_path):
    """URDF 안의 상대 <mesh filename="..."> 경로를 Gazebo가 바로 찾을 수 있게 file:// 절대 경로로 바꾼다."""
    urdf_path = Path(urdf_path).resolve()
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    for mesh in root.findall(".//mesh"):
        fn = mesh.attrib.get("filename")
        if fn and not fn.startswith(("file://", "package://", "model://")):
            p = Path(fn)
            abs_p = p if p.is_absolute() else (urdf_path.parent / p).resolve()
            mesh.attrib["filename"] = abs_p.as_uri()
    name = root.attrib.get("name") or urdf_path.stem
    return name, ET.tostring(root, encoding="unicode")


def prepare_urdf(path):
    """입력 경로(object.urdf, mesh.ply, 또는 out/ 폴더) → object.urdf 경로."""
    p = Path(path).resolve()
    if p.is_dir():
        return urdf_mod.ensure_urdf(p)
    if p.suffix.lower() == ".ply":
        return urdf_mod.export_urdf(p, p.parent)["urdf"]
    if not p.is_file():
        raise FileNotFoundError(f"URDF 파일 없음: {p}")
    return p


def main(argv=None):
    ap = argparse.ArgumentParser(description="스캔 산출물 URDF를 Gazebo에 로드(spawn)한다")
    ap.add_argument("urdf", help="object.urdf, mesh.ply, 또는 out/ 폴더 경로")
    ap.add_argument("--world", default="scan3d_room", help="Gazebo world 이름(기본: scan3d_room)")
    ap.add_argument("--name", default=None, help="Gazebo 모델 이름(기본: URDF robot name)")
    ap.add_argument("--at", nargs=2, type=float, default=[0.0, 0.0], metavar=("X", "Y"), help="배치 위치(m)")
    ap.add_argument("--z", type=float, default=0.002, help="바닥 위 높이(m, 표식 판 두께 기본 0.002)")
    ap.add_argument("--yaw", type=float, default=0.0, help="회전 각(도)")
    ap.add_argument("--dry-run", action="store_true", help="실행하지 않고 준비된 URDF XML과 명령만 출력")
    args = ap.parse_args(argv)

    urdf_path = prepare_urdf(args.urdf)
    robot_name, xml = resolve_urdf_xml(urdf_path)
    model_name = args.name or robot_name
    yaw_rad = math.radians(args.yaw)

    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", args.world,
        "-name", model_name,
        "-string", xml,
        "-x", f"{args.at[0]:.4f}",
        "-y", f"{args.at[1]:.4f}",
        "-z", f"{args.z:.4f}",
        "-Y", f"{yaw_rad:.5f}",
    ]
    if args.dry_run:
        print(xml)
        print("# cmd:", " ".join(cmd[:8]), "-string <urdf>", " ".join(cmd[10:]))
        return 0

    env = os.environ.copy()
    prev = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{urdf_path.parent}:{prev}" if prev else str(urdf_path.parent)
    return subprocess.call(cmd, env=env)


if __name__ == "__main__":
    sys.exit(main())
