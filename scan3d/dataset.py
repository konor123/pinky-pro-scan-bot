"""스캔 기록 `scans/<scan_id>/`와 Open3D 예제 형식 데이터셋(docs/specs/dataset.md, PLAN 4.5).

`<role>/color/%06d.jpg`, `<role>/depth/%06d.png`, `<role>/camera_intrinsic.json`은
Open3D v0.20.0 `reconstruction_system/sensors/realsense_recorder.py`가 만드는 형식과 같고,
번호는 시점 번호 k다.
"""
import json
import math
import shutil
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from scan3d.robot import align, tagpose
from scan3d.protocol import PAYLOAD_KEYS

JPEG_QUALITY = 95


def _dump(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")


class RawWriter:
    """역할 하나의 `raw/` 기록. 줄마다 flush."""

    def __init__(self, role_dir):
        self.role_dir = Path(role_dir)
        self.raw = self.role_dir / "raw"
        self.raw.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._n = 0
        self._stream = open(self.raw / "stream.jsonl", "a", encoding="utf-8")

    def calib(self, hello):
        """`hello` 응답 그대로 → `raw/calib.json`."""
        _dump(self.raw / "calib.json", hello)

    def snapshot(self, header, payloads, k, use, phi_target):
        """스냅샷 머리말 + station 기록과 원본 바이트를 쓰고 번호 `NNNNNN`을 돌려준다."""
        with self._lock:
            snap = f"{self._n:06d}"
            self._n += 1
        hdr = json.loads(json.dumps(header))
        hdr["station"] = {"k": k, "use": use, "phi_target": phi_target}
        items = [key for key in PAYLOAD_KEYS if isinstance(hdr.get(key), dict) and "bytes" in hdr[key]]
        if len(items) != len(payloads):
            raise ValueError(f"payloads {len(payloads)} != header streams {items}")
        for key, data in zip(items, payloads):
            if len(data) != hdr[key]["bytes"]:
                raise ValueError(f"{key} bytes {len(data)} != header {hdr[key]['bytes']}")
            (self.raw / f"{snap}_{key}.{hdr[key]['fmt']}").write_bytes(data)
        _dump(self.raw / f"{snap}.json", hdr)
        return snap

    def stream(self, msg, direction, pc_t):
        line = dict(msg)
        line["pc_t"] = pc_t
        line["dir"] = direction
        text = json.dumps(line, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            self._stream.write(text + "\n")
            self._stream.flush()

    def close(self):
        self._stream.close()


class ScanWriter:
    """`scans/<scan_id>/`를 만들고 config·배치 파일을 복사한다. 폴더는 `scan_dir`."""

    def __init__(self, root, cfg_path, board_path, scan_id=None):
        self.scan_id = scan_id or time.strftime("%Y%m%d-%H%M%S")
        self.scan_dir = Path(root) / self.scan_id
        self.scan_dir.mkdir(parents=True, exist_ok=False)
        shutil.copyfile(cfg_path, self.scan_dir / "config.yaml")
        shutil.copyfile(board_path, self.scan_dir / "board.json")
        self._lock = threading.Lock()
        self._events = open(self.scan_dir / "events.jsonl", "a", encoding="utf-8")
        self._roles = {}

    def role(self, role):
        with self._lock:
            if role not in self._roles:
                self._roles[role] = RawWriter(self.scan_dir / role)
            return self._roles[role]

    def event(self, obj):
        text = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            self._events.write(text + "\n")
            self._events.flush()

    def report(self, obj):
        _dump(self.scan_dir / "report.json", obj)

    def close(self):
        self._events.close()
        for w in self._roles.values():
            w.close()


def _read_events(scan_dir, role):
    """역할의 시점별 마지막 `view` 이벤트와 `skip`(우회) 시점."""
    views, skips = {}, set()
    path = Path(scan_dir) / "events.jsonl"
    if not path.exists():
        return views, skips
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        e = json.loads(line)
        if e.get("role") != role:
            continue
        if e.get("type") == "view":
            views[int(e["k"])] = e
        elif e.get("type") == "skip" and e.get("reason", "bypass") == "bypass":
            skips.add(int(e["k"]))
    return views, skips


def _raw_snapshots(raw):
    out = []
    for p in sorted(raw.glob("[0-9]" * 6 + ".json")):
        hdr = json.loads(p.read_text(encoding="utf-8"))
        out.append((p.stem, hdr))
    return out


def _payloads(raw, snap, hdr):
    return [(raw / f"{snap}_{key}.{hdr[key]['fmt']}").read_bytes()
            for key in PAYLOAD_KEYS if isinstance(hdr.get(key), dict) and "bytes" in hdr[key]]


def depth_invalid(depth, K, T_cam_board, r_eff, obj_h):
    """품질 원기둥(반지름 r_eff, 높이 obj_h)을 투영한 영역 안에서 깊이가 0인 픽셀 비율."""
    a = np.linspace(0.0, 2 * math.pi, 72, endpoint=False)
    ring = np.stack([r_eff * np.cos(a), r_eff * np.sin(a)], -1)
    pts = np.concatenate([np.column_stack([ring, np.full(len(a), z)]) for z in (0.0, obj_h)])
    T = np.asarray(T_cam_board, float)
    pc = pts @ T[:3, :3].T + T[:3, 3]
    pc = pc[pc[:, 2] > 1e-3]
    if len(pc) < 3:
        return None
    uv = (pc[:, :2] / pc[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    h, w = depth.shape
    uv = np.clip(uv, -4 * w, 4 * w)
    mask = np.zeros((h, w), np.uint8)
    hull = cv2.convexHull(np.round(uv * 16).astype(np.int32))
    cv2.fillConvexPoly(mask, hull, 1, cv2.LINE_8, shift=4)
    n = int(mask.sum())
    if n == 0:
        return None
    return float(((depth == 0) & (mask > 0)).sum() / n)


def build(scan_dir, role):
    """`config.yaml`, `board.json`, `events.jsonl`, `<role>/raw/`만으로 Open3D 형식 데이터셋을 다시 만든다."""
    scan_dir = Path(scan_dir)
    role_dir = scan_dir / role
    raw = role_dir / "raw"
    cfg = yaml.safe_load((scan_dir / "config.yaml").read_text(encoding="utf-8"))
    board_json = json.loads((scan_dir / "board.json").read_text(encoding="utf-8"))
    board = tagpose.load_board(board_json, cfg["board"]["tag_mm"])
    scene = cfg["scene"]
    r_eff = float(scene["r_obj"]) + float(scene["place_err"])

    for name in ("color", "depth"):
        shutil.rmtree(role_dir / name, ignore_errors=True)
    for name in ("views.json", "outcomes.json", "camera_intrinsic.json"):
        (role_dir / name).unlink(missing_ok=True)
    (role_dir / "color").mkdir()
    (role_dir / "depth").mkdir()

    hello = json.loads((raw / "calib.json").read_text(encoding="utf-8"))
    calib = hello.get("calib", hello)
    cc = calib["color"]
    K = align.K_of(cc)
    _dump(role_dir / "camera_intrinsic.json",
          {"width": int(cc["w"]), "height": int(cc["h"]),
           "intrinsic_matrix": [float(v) for v in K.T.reshape(-1)]})

    by_k = {}
    for snap, hdr in _raw_snapshots(raw):
        st = hdr.get("station", {})
        if st.get("use") in ("view", "retake") and st.get("k") is not None:
            by_k.setdefault(int(st["k"]), []).append((snap, hdr))

    views = []
    for k in sorted(by_k):
        for snap, hdr in sorted(by_k[k], key=lambda s: s[0], reverse=True):
            fr = align.prepare(hdr, _payloads(raw, snap, hdr), calib)
            pose = tagpose.estimate(fr.color, fr.K, board)
            if pose is None:
                continue
            name = f"{k:06d}"
            cv2.imwrite(str(role_dir / "color" / f"{name}.jpg"), fr.color, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            cv2.imwrite(str(role_dir / "depth" / f"{name}.png"), fr.depth)
            T = pose.T_cam_board
            views.append({
                "k": k, "snap": snap,
                "phi": tagpose.phi_deg(T), "d": tagpose.distance_d(T, r_eff),
                "T_cam_board": T.tolist(), "n_tags": pose.n_tags, "reproj_px": pose.reproj_px,
                "depth_invalid": depth_invalid(fr.depth, fr.K, T, r_eff, float(scene["obj_h"])),
            })
            break
    _dump(role_dir / "views.json", views)

    ev_views, ev_skips = _read_events(scan_dir, role)
    posed = {v["k"]: v for v in views}
    ks = set(range(int(scene.get("n_views", 0)))) | set(by_k) | set(ev_views) | ev_skips
    outcomes = []
    for k in sorted(ks):
        o = {"k": k, "captured": False, "pose": False, "d": None, "d_source": None, "reason": None,
             "crop": None, "fused": None}
        if k in posed:
            o.update(captured=True, pose=True, d=posed[k]["d"], d_source="tag")
        elif k in ev_views or k in by_k:
            e = ev_views.get(k, {})
            o.update(captured=True, d=e.get("d"), d_source=e.get("d_source"), reason="no_tags")
        elif k in ev_skips:
            o.update(reason="bypass")
        outcomes.append(o)
    _dump(role_dir / "outcomes.json", outcomes)
    return views


def load(role_dir):
    """(K 3×3, width, height, views). views의 `T_cam_board`는 4×4 배열."""
    role_dir = Path(role_dir)
    intr = json.loads((role_dir / "camera_intrinsic.json").read_text(encoding="utf-8"))
    K = np.asarray(intr["intrinsic_matrix"], float).reshape(3, 3).T
    views = json.loads((role_dir / "views.json").read_text(encoding="utf-8"))
    for v in views:
        v["T_cam_board"] = np.asarray(v["T_cam_board"], float)
    return K, int(intr["width"]), int(intr["height"]), views
