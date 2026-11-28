"""스캔 메시(PLY/Open3D) → URDF(+STL) 내보내기 및 URDF 로더.

스캔 재구성 산출물(`mesh.ply`)에서 표준 URDF(`object.urdf`)와 바이너리 STL(`mesh.stl`)을 만든다.
외부 라이브러리 없이 NumPy와 표준 라이브러리만으로 동작해 macOS/Linux/Docker 어디서나
URDF 생성·파싱·시뮬레이터 로드가 가능하다.

  python -m scan3d.urdf scans/<id>/out/mesh.ply [--out DIR] [--name NAME] [--mass KG]
"""
import argparse
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

import numpy as np

DEFAULT_RGBA = (0.76, 0.78, 0.82, 1.0)
_PLY_DTYPES = {
    "char": "i1", "int8": "i1",
    "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2",
    "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4",
    "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4",
    "double": "f8", "float64": "f8",
}
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_]+")
_OFFSET_RE = re.compile(r"scan3d_ply_offset:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)")


@dataclass
class UrdfModel:
    """파싱된 URDF 물체 모델."""
    name: str
    urdf_path: Path
    mesh_path: Path | None
    color_ply_path: Path | None
    vertices: np.ndarray          # (N, 3) float64, 바닥 z=0 기준
    triangles: np.ndarray         # (M, 3) int32
    colors: np.ndarray | None     # (N, 3) float64 [0, 1] 또는 None
    dims_m: tuple[float, float, float]
    r_obj: float
    obj_h: float
    mass_kg: float
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    rgba: tuple[float, float, float, float]
    ply_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def public(self, mesh_url=None, urdf_url=None, entry_id=None):
        return {
            "name": self.name,
            "urdf_path": str(self.urdf_path),
            "mesh_file": self.mesh_path.name if self.mesh_path else None,
            "has_color_ply": self.color_ply_path is not None and self.color_ply_path.is_file(),
            "dims_mm": [round(v * 1000.0, 1) for v in self.dims_m],
            "r_obj": round(self.r_obj, 4),
            "obj_h": round(self.obj_h, 4),
            "mass_kg": round(self.mass_kg, 4),
            "origin_xyz": [round(v, 5) for v in self.origin_xyz],
            "origin_rpy": [round(v, 5) for v in self.origin_rpy],
            "rgba": [round(v, 3) for v in self.rgba],
            "ply_offset": [round(v, 5) for v in self.ply_offset],
            "n_vertices": int(len(self.vertices)),
            "n_triangles": int(len(self.triangles)),
            "mesh_url": mesh_url,
            "urdf_url": urdf_url,
            "entry_id": entry_id,
        }

    def to_open3d_mesh(self):
        import open3d as o3d
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(np.asarray(self.vertices, dtype=np.float64))
        m.triangles = o3d.utility.Vector3iVector(np.asarray(self.triangles, dtype=np.int32))
        if self.colors is not None and len(self.colors) == len(self.vertices):
            m.vertex_colors = o3d.utility.Vector3dVector(np.clip(self.colors, 0.0, 1.0))
        else:
            c = np.tile(np.asarray(self.rgba[:3], dtype=np.float64), (len(self.vertices), 1))
            m.vertex_colors = o3d.utility.Vector3dVector(c)
        m.compute_vertex_normals()
        return m


# ---------- PLY 읽기 ----------

def read_ply(path):
    """PLY(ascii / binary_little_endian / binary_big_endian) → (vertices, triangles, colors|None)."""
    data = Path(path).read_bytes()
    end = data.find(b"end_header")
    if end < 0:
        raise ValueError(f"PLY 헤더 없음: {path}")
    nl = data.find(b"\n", end)
    if nl < 0:
        raise ValueError(f"PLY end_header 줄바꿈 없음: {path}")
    header = data[:nl].decode("ascii", errors="replace").splitlines()
    body = data[nl + 1:]

    fmt = None
    elements = []
    cur = None
    for raw in header:
        parts = raw.strip().split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            cur = {"name": parts[1], "count": int(parts[2]), "props": []}
            elements.append(cur)
        elif parts[0] == "property" and cur is not None:
            if parts[1] == "list":
                cur["props"].append(("list", parts[2], parts[3], parts[4]))
            else:
                cur["props"].append(("scalar", parts[1], parts[2]))

    if fmt not in ("ascii", "binary_little_endian", "binary_big_endian"):
        raise ValueError(f"지원하지 않는 PLY 형식: {fmt}")

    verts = np.zeros((0, 3), dtype=np.float64)
    tris = np.zeros((0, 3), dtype=np.int32)
    cols = None

    if fmt == "ascii":
        lines = body.decode("ascii", errors="replace").splitlines()
        idx = 0
        for el in elements:
            n = el["count"]
            if el["name"] == "vertex":
                names = [p[2] for p in el["props"] if p[0] == "scalar"]
                types = {p[2]: p[1] for p in el["props"] if p[0] == "scalar"}
                arr = np.loadtxt(lines[idx:idx + n], dtype=np.float64).reshape(n, len(names)) if n else np.zeros((0, len(names)))
                xi, yi, zi = names.index("x"), names.index("y"), names.index("z")
                verts = arr[:, [xi, yi, zi]].astype(np.float64)
                if all(c in names for c in ("red", "green", "blue")):
                    ri, gi, bi = names.index("red"), names.index("green"), names.index("blue")
                    c = arr[:, [ri, gi, bi]].astype(np.float64)
                    if types["red"] in ("uchar", "uint8") or (len(c) and c.max() > 1.0):
                        c = c / 255.0
                    cols = np.clip(c, 0.0, 1.0)
            elif el["name"] == "face":
                out_t = []
                for line in lines[idx:idx + n]:
                    toks = line.split()
                    if not toks:
                        continue
                    cnt = int(toks[0])
                    ids = [int(t) for t in toks[1:1 + cnt]]
                    for j in range(1, len(ids) - 1):
                        out_t.append((ids[0], ids[j], ids[j + 1]))
                tris = np.asarray(out_t, dtype=np.int32).reshape(-1, 3) if out_t else np.zeros((0, 3), dtype=np.int32)
            idx += n
        return verts, tris, cols

    endian = "<" if fmt == "binary_little_endian" else ">"
    off = 0
    for el in elements:
        n = el["count"]
        has_list = any(p[0] == "list" for p in el["props"])
        if not has_list:
            dt = np.dtype([(p[2], endian + _PLY_DTYPES[p[1]]) for p in el["props"]])
            arr = np.frombuffer(body, dtype=dt, count=n, offset=off)
            off += n * dt.itemsize
            if el["name"] == "vertex":
                verts = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
                names = dt.names or ()
                if all(c in names for c in ("red", "green", "blue")):
                    c = np.column_stack([arr["red"], arr["green"], arr["blue"]]).astype(np.float64)
                    if arr["red"].dtype.kind in ("u", "i") or (len(c) and c.max() > 1.0):
                        c = c / 255.0
                    cols = np.clip(c, 0.0, 1.0)
        elif el["name"] == "face" and len(el["props"]) == 1 and el["props"][0][0] == "list":
            _, cnt_t, idx_t, _ = el["props"][0]
            cnt_dt = np.dtype(endian + _PLY_DTYPES[cnt_t])
            idx_dt = np.dtype(endian + _PLY_DTYPES[idx_t])
            # 빠른 경로: 모든 면이 삼각형(count == 3)인 경우 한 번에 디코딩
            tri_rec = np.dtype([("c", cnt_dt), ("v", idx_dt, (3,))])
            if n > 0 and off + n * tri_rec.itemsize <= len(body):
                rec = np.frombuffer(body, dtype=tri_rec, count=n, offset=off)
                if np.all(rec["c"] == 3):
                    tris = rec["v"].astype(np.int32)
                    off += n * tri_rec.itemsize
                    continue
            out_t = []
            c_fmt = endian + _struct_code(cnt_t)
            i_fmt_base = _struct_code(idx_t)
            c_sz = cnt_dt.itemsize
            i_sz = idx_dt.itemsize
            for _ in range(n):
                cnt = struct.unpack_from(c_fmt, body, off)[0]
                off += c_sz
                ids = struct.unpack_from(f"{endian}{cnt}{i_fmt_base}", body, off)
                off += cnt * i_sz
                for j in range(1, cnt - 1):
                    out_t.append((ids[0], ids[j], ids[j + 1]))
            tris = np.asarray(out_t, dtype=np.int32).reshape(-1, 3) if out_t else np.zeros((0, 3), dtype=np.int32)
        else:
            # 기타 list 요소 건너뛰기
            for _ in range(n):
                for p in el["props"]:
                    if p[0] == "scalar":
                        off += np.dtype(endian + _PLY_DTYPES[p[1]]).itemsize
                    else:
                        c_dt = np.dtype(endian + _PLY_DTYPES[p[1]])
                        cnt = int(np.frombuffer(body, dtype=c_dt, count=1, offset=off)[0])
                        off += c_dt.itemsize + cnt * np.dtype(endian + _PLY_DTYPES[p[2]]).itemsize

    return verts, tris, cols


def _struct_code(t):
    return {
        "char": "b", "int8": "b", "uchar": "B", "uint8": "B",
        "short": "h", "int16": "h", "ushort": "H", "uint16": "H",
        "int": "i", "int32": "i", "uint": "I", "uint32": "I",
        "float": "f", "float32": "f", "double": "d", "float64": "d",
    }[t]


# ---------- STL 쓰기 / 읽기 ----------

_STL_TRI_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("v0", "<f4", (3,)),
    ("v1", "<f4", (3,)),
    ("v2", "<f4", (3,)),
    ("attr", "<u2"),
])


def write_stl(path, vertices, triangles, header_text="scan3d binary STL"):
    """바이너리 STL 파일을 원자적으로 쓴다."""
    path = Path(path)
    v = np.asarray(vertices, dtype=np.float64)
    t = np.asarray(triangles, dtype=np.int32)
    if len(t) == 0:
        raise ValueError("삼각형이 없는 메시는 STL로 쓸 수 없다")
    v0, v1, v2 = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    nrm = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-12)

    rec = np.zeros(len(t), dtype=_STL_TRI_DTYPE)
    rec["normal"] = nrm.astype(np.float32)
    rec["v0"] = v0.astype(np.float32)
    rec["v1"] = v1.astype(np.float32)
    rec["v2"] = v2.astype(np.float32)

    hdr = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b"\0")
    tmp = path.parent / f".{path.name}.tmp"
    with open(tmp, "wb") as f:
        f.write(hdr)
        f.write(struct.pack("<I", len(t)))
        f.write(rec.tobytes())
    os.replace(tmp, path)
    return path


def read_stl(path):
    """바이너리/ASCII STL → (vertices (N,3) float64, triangles (M,3) int32)."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(data, dtype=_STL_TRI_DTYPE, count=n, offset=84)
            verts = np.stack([rec["v0"], rec["v1"], rec["v2"]], axis=1).reshape(-1, 3).astype(np.float64)
            tris = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)
            return verts, tris
    # ASCII STL
    pts = []
    for line in data.decode("ascii", errors="replace").splitlines():
        s = line.strip().split()
        if len(s) == 4 and s[0].lower() == "vertex":
            pts.append((float(s[1]), float(s[2]), float(s[3])))
    n_tri = len(pts) // 3
    verts = np.asarray(pts[:n_tri * 3], dtype=np.float64).reshape(-1, 3)
    tris = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)
    return verts, tris


# ---------- URDF 내보내기 ----------

def _safe_name(name):
    s = _SAFE_NAME.sub("_", str(name or "scanned_object")).strip("_")
    if not s or s[0].isdigit():
        s = "obj_" + s
    return s


def _mesh_arrays(mesh_or_path):
    """Open3D TriangleMesh, (verts, tris, cols) 튜플, 또는 PLY/STL 경로 → (verts, tris, cols)."""
    if isinstance(mesh_or_path, (str, Path)):
        p = Path(mesh_or_path)
        if p.suffix.lower() == ".stl":
            v, t = read_stl(p)
            return v, t, None
        return read_ply(p)
    if isinstance(mesh_or_path, tuple):
        v, t = mesh_or_path[0], mesh_or_path[1]
        c = mesh_or_path[2] if len(mesh_or_path) > 2 else None
        return np.asarray(v, np.float64), np.asarray(t, np.int32), (None if c is None else np.asarray(c, np.float64))
    # Open3D TriangleMesh
    v = np.asarray(mesh_or_path.vertices, dtype=np.float64)
    t = np.asarray(mesh_or_path.triangles, dtype=np.int32)
    c = np.asarray(mesh_or_path.vertex_colors, dtype=np.float64) if mesh_or_path.has_vertex_colors() else None
    return v, t, c


def _estimate_mass(dims_m, mass_kg=None):
    if mass_kg is not None:
        return max(0.01, float(mass_kg))
    vol = float(dims_m[0] * dims_m[1] * dims_m[2])
    # 경량 소비재·모형 기준 겉보기 밀도 ~180 kg/m³, [0.10, 2.0] kg 범위로 제한
    return round(min(2.0, max(0.10, vol * 180.0)), 3)


def _box_inertia(mass_kg, dims_m):
    sx, sy, sz = dims_m
    ixx = max(1e-6, mass_kg * (sy * sy + sz * sz) / 12.0)
    iyy = max(1e-6, mass_kg * (sx * sx + sz * sz) / 12.0)
    izz = max(1e-6, mass_kg * (sx * sx + sy * sy) / 12.0)
    return ixx, iyy, izz


def export_urdf(mesh_or_path, out_dir, name=None, mass_kg=None, static=True,
                mesh_filename="mesh.stl", urdf_filename="object.urdf"):
    """스캔 메시를 바닥(z=0)·XY 중심 정렬된 STL과 URDF(`object.urdf`)로 내보낸다.

    반환: {"urdf": Path, "stl": Path, "name": str, "dims_m": [...], "mass_kg": float, "offset": [...]}
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    verts, tris, cols = _mesh_arrays(mesh_or_path)
    if len(verts) == 0 or len(tris) == 0:
        raise ValueError("빈 메시로는 URDF를 만들 수 없다")

    lo = verts.min(axis=0)
    hi = verts.max(axis=0)
    dims = hi - lo
    offset = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, lo[2]], dtype=np.float64)
    shifted = verts - offset

    stl_path = out_dir / mesh_filename
    write_stl(stl_path, shifted, tris)

    robot_name = _safe_name(name or (out_dir.parent.name if out_dir.name.startswith("out") else out_dir.name))
    mass = _estimate_mass(dims, mass_kg)
    ixx, iyy, izz = _box_inertia(mass, dims)
    com = shifted.mean(axis=0)

    if cols is not None and len(cols) > 0:
        avg = np.clip(cols.mean(axis=0), 0.0, 1.0)
        rgba = (float(avg[0]), float(avg[1]), float(avg[2]), 1.0)
    else:
        rgba = DEFAULT_RGBA

    xml = f"""<?xml version="1.0"?>
<!-- Generated by scan3d.urdf -->
<!-- scan3d_dims_mm: {dims[0] * 1000:.1f} {dims[1] * 1000:.1f} {dims[2] * 1000:.1f} -->
<!-- scan3d_ply_offset: {offset[0]:.6f} {offset[1]:.6f} {offset[2]:.6f} -->
<robot name="{robot_name}">
  <link name="base_link">
    <inertial>
      <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>
      <mass value="{mass:.4f}"/>
      <inertia ixx="{ixx:.6e}" ixy="0" ixz="0" iyy="{iyy:.6e}" iyz="0" izz="{izz:.6e}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_filename}" scale="1 1 1"/>
      </geometry>
      <material name="scanned_material">
        <color rgba="{rgba[0]:.4f} {rgba[1]:.4f} {rgba[2]:.4f} {rgba[3]:.4f}"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_filename}" scale="1 1 1"/>
      </geometry>
    </collision>
  </link>
  <gazebo reference="base_link">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo>
    <static>{"true" if static else "false"}</static>
  </gazebo>
</robot>
"""
    urdf_path = out_dir / urdf_filename
    tmp = out_dir / f".{urdf_filename}.tmp"
    tmp.write_text(xml, encoding="utf-8")
    os.replace(tmp, urdf_path)
    return {
        "urdf": urdf_path,
        "stl": stl_path,
        "name": robot_name,
        "dims_m": [float(v) for v in dims],
        "mass_kg": mass,
        "offset": [float(v) for v in offset],
        "rgba": list(rgba),
    }


def ensure_urdf(out_dir, name=None):
    """결과 폴더(`out/`)에 `object.urdf`와 `mesh.stl`이 없으면 `mesh.ply`에서 만든다."""
    out_dir = Path(out_dir)
    urdf_path = out_dir / "object.urdf"
    stl_path = out_dir / "mesh.stl"
    if urdf_path.is_file() and stl_path.is_file():
        return urdf_path
    ply_path = out_dir / "mesh.ply"
    if not ply_path.is_file():
        raise FileNotFoundError(f"메시 파일 없음: {ply_path}")
    return export_urdf(ply_path, out_dir, name=name)["urdf"]


# ---------- URDF 파싱 및 시뮬레이터 로드 ----------

def _parse_vec(text, default):
    if not text:
        return tuple(default)
    vals = [float(x) for x in text.strip().split()]
    return tuple(vals[:len(default)]) if len(vals) >= len(default) else tuple(default)


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _resolve_mesh_path(filename, base_dir):
    if filename.startswith("file://"):
        return Path(urlparse(filename).path)
    if filename.startswith("package://"):
        rel = filename[len("package://"):].split("/", 1)[-1]
        p = base_dir / rel
        if p.is_file():
            return p
    p = Path(filename)
    return p if p.is_absolute() else (base_dir / p).resolve()


def _box_geometry(size):
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    v = np.array([
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ], dtype=np.float64)
    t = np.array([
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
        [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7],
    ], dtype=np.int32)
    return v, t


def load_urdf(urdf_path):
    """URDF XML 파일을 파싱해 시뮬레이터에서 쓸 `UrdfModel`을 돌려준다."""
    urdf_path = Path(urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF 파일 없음: {urdf_path}")
    text = urdf_path.read_text(encoding="utf-8")
    m_off = _OFFSET_RE.search(text)
    ply_offset = tuple(float(v) for v in m_off.groups()) if m_off else (0.0, 0.0, 0.0)

    root = ET.fromstring(text)
    if root.tag != "robot":
        raise ValueError(f"URDF 루트 태그가 robot이 아님: {root.tag}")
    name = root.attrib.get("name") or urdf_path.stem

    link = None
    for cand in root.findall("link"):
        if cand.find("visual") is not None:
            link = cand
            break
    if link is None:
        raise ValueError(f"URDF에 visual이 있는 link가 없음: {urdf_path}")

    mass_kg = 0.25
    inertial = link.find("inertial")
    if inertial is not None:
        m_el = inertial.find("mass")
        if m_el is not None and "value" in m_el.attrib:
            mass_kg = float(m_el.attrib["value"])

    visual = link.find("visual")
    orig_el = visual.find("origin")
    origin_xyz = _parse_vec(orig_el.attrib.get("xyz") if orig_el is not None else None, (0.0, 0.0, 0.0))
    origin_rpy = _parse_vec(orig_el.attrib.get("rpy") if orig_el is not None else None, (0.0, 0.0, 0.0))

    rgba = DEFAULT_RGBA
    mat_el = visual.find("material")
    if mat_el is not None:
        col_el = mat_el.find("color")
        if col_el is not None and "rgba" in col_el.attrib:
            rgba = _parse_vec(col_el.attrib["rgba"], DEFAULT_RGBA)

    geom = visual.find("geometry")
    if geom is None:
        raise ValueError(f"URDF visual에 geometry 없음: {urdf_path}")

    mesh_path = None
    color_ply_path = None
    cols = None
    mesh_el = geom.find("mesh")
    box_el = geom.find("box")
    if mesh_el is not None:
        filename = mesh_el.attrib.get("filename")
        if not filename:
            raise ValueError("URDF mesh 태그에 filename 없음")
        scale = _parse_vec(mesh_el.attrib.get("scale"), (1.0, 1.0, 1.0))
        mesh_path = _resolve_mesh_path(filename, urdf_path.parent)
        if not mesh_path.is_file():
            raise FileNotFoundError(f"URDF가 참조하는 메시 파일 없음: {mesh_path}")
        # scan3d가 내보낸 mesh.stl 옆에 색이 있는 mesh.ply가 있으면 색을 함께 불러온다
        sibling_ply = mesh_path.with_suffix(".ply")
        if mesh_path.suffix.lower() == ".ply":
            verts, tris, cols = read_ply(mesh_path)
            color_ply_path = mesh_path
        elif sibling_ply.is_file() and m_off is not None:
            verts, tris, cols = read_ply(sibling_ply)
            verts = verts - np.asarray(ply_offset, dtype=np.float64)
            color_ply_path = sibling_ply
        else:
            verts, tris = read_stl(mesh_path)
        verts = verts * np.asarray(scale, dtype=np.float64)
    elif box_el is not None:
        size = _parse_vec(box_el.attrib.get("size"), (0.18, 0.14, 0.16))
        verts, tris = _box_geometry(size)
        if origin_xyz == (0.0, 0.0, 0.0):
            verts[:, 2] += size[2] / 2.0
    else:
        raise ValueError(f"지원하지 않는 URDF geometry: {[c.tag for c in geom]}")

    if origin_rpy != (0.0, 0.0, 0.0):
        verts = verts @ _rpy_matrix(origin_rpy).T
    if origin_xyz != (0.0, 0.0, 0.0):
        verts = verts + np.asarray(origin_xyz, dtype=np.float64)

    lo = verts.min(axis=0)
    hi = verts.max(axis=0)
    dims_m = tuple(float(v) for v in (hi - lo))
    r_obj = float(np.hypot(verts[:, 0], verts[:, 1]).max()) if len(verts) else 0.14
    obj_h = float(hi[2]) if len(verts) else 0.16

    return UrdfModel(
        name=name,
        urdf_path=urdf_path,
        mesh_path=mesh_path,
        color_ply_path=color_ply_path,
        vertices=verts,
        triangles=tris,
        colors=cols,
        dims_m=dims_m,
        r_obj=r_obj,
        obj_h=obj_h,
        mass_kg=mass_kg,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        rgba=rgba,
        ply_offset=ply_offset,
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description="스캔 메시(PLY)에서 URDF(object.urdf + mesh.stl)를 만든다")
    ap.add_argument("mesh", help="입력 mesh.ply(또는 mesh.stl) 경로")
    ap.add_argument("--out", default=None, help="출력 폴더(기본: 입력 파일과 같은 폴더)")
    ap.add_argument("--name", default=None, help="URDF robot 이름")
    ap.add_argument("--mass", type=float, default=None, help="질량 kg(기본: 크기에서 자동 추정)")
    ap.add_argument("--dynamic", action="store_true", help="Gazebo static=false(움직이는 물체)")
    args = ap.parse_args(argv)
    src = Path(args.mesh).resolve()
    out_dir = Path(args.out).resolve() if args.out else src.parent
    info = export_urdf(src, out_dir, name=args.name, mass_kg=args.mass, static=not args.dynamic)
    d = [v * 1000.0 for v in info["dims_m"]]
    print(f"URDF: {info['urdf']}")
    print(f"STL:  {info['stl']}")
    print(f"크기: {d[0]:.1f} × {d[1]:.1f} × {d[2]:.1f} mm, 질량 {info['mass_kg']:.3f} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
