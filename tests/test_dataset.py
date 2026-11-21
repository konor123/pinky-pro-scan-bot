"""스캔 기록과 Open3D 형식 데이터셋(docs/specs/dataset.md)."""
import json
import math
import shutil
import socket
import threading

import cv2
import numpy as np
import open3d as o3d
import pytest
import yaml

from scan3d import dataset

from scan3d.robot import tagpose

from scan3d.sim import synth
from scan3d.protocol import Channel, pack_snapshot

NOISE_MM = 2.0


def through_channel(header, streams, compress):
    """protocol로 보내고 받은 (머리말, 바이트 목록). 받는 쪽이 zlib을 풀고 머리말을 정리한다."""
    hdr, payloads = pack_snapshot(header, streams, compress)
    a, b = socket.socketpair()
    tx, rx = Channel(a), Channel(b)
    t = threading.Thread(target=tx.send, args=(hdr, payloads))
    t.start()
    try:
        return rx.recv(30.0)
    finally:
        t.join()
        a.close()
        b.close()


def snapshot_streams(scene, T_cam_board, calib, rng, n_depth=5, color=None):
    """합성 스냅샷 머리말과 원본 바이트(`yuyv`, `z16` × n)."""
    cc, dc = calib["color"], calib["depth"]
    if color is None:
        color = scene.render_color(T_cam_board, cc)
    Td = synth.depth_pose(T_cam_board, calib)
    depth = b"".join(synth.to_z16(scene.render_depth(Td, dc, rng, NOISE_MM)) for _ in range(n_depth))
    header = {"ok": True, "t": 0.0, "odom": [0.0, 0.0, 0.0],
              "color": {"w": cc["w"], "h": cc["h"], "fmt": "yuyv"},
              "depth": {"w": dc["w"], "h": dc["h"], "fmt": "z16", "n": n_depth}}
    return header, {"color": synth.to_yuyv(color), "depth": depth}


def gray_color(calib, value=128):
    return np.full((calib["color"]["h"], calib["color"]["w"], 3), value, np.uint8)


def new_scan(tmp_path, repo_root, cfg=None, scan_id="20261120-120000"):
    cfg_path = tmp_path / "scan.yaml"
    if cfg is None:
        shutil.copyfile(repo_root / "config" / "scan.yaml", cfg_path)
    else:
        cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    return dataset.ScanWriter(tmp_path / "scans", cfg_path, repo_root / "config" / "board.json", scan_id=scan_id)


def write_role(writer, role, scene, cfg, shots, rng, compress=False):
    """shots: [(k, use, phi_deg, no_tags)] → raw/. 참 자세 {snap: T}를 돌려준다."""
    calib = synth.default_calib()
    rw = writer.role(role)
    rw.calib({"ok": True, "role": role, "serial": f"SYN-{role}", "calib": calib})
    m = cfg["robots"][role]["mount"]
    truth = {}
    for k, use, phi, no_tags in shots:
        T = synth.pose_on_circle(phi, cfg["scene"]["R"], m["z"], m["pitch_down_deg"])
        hdr, streams = snapshot_streams(scene, T, calib, rng, cfg["capture"]["n_depth"],
                                        gray_color(calib) if no_tags else None)
        hdr, payloads = through_channel(hdr, streams, compress)
        snap = rw.snapshot(hdr, payloads, k, use, phi)
        truth[snap] = T
    return truth


def pose_error(T_est, T_true):
    p = np.subtract(tagpose.cam_position(T_est), tagpose.cam_position(T_true))
    rv, _ = cv2.Rodrigues(np.asarray(T_est)[:3, :3] @ np.asarray(T_true)[:3, :3].T)
    return float(np.linalg.norm(p) * 1000.0), float(math.degrees(np.linalg.norm(rv)))


@pytest.fixture(scope="module")
def board_json(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def built_base(tmp_path_factory, repo_root, board_json):
    """L의 k = 0, 1, 3(표식 있음)과 k = 2(표식 없음) 원본."""
    with open(repo_root / "config" / "scan.yaml", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    tmp = tmp_path_factory.mktemp("built")
    w = new_scan(tmp, repo_root)
    scene = synth.Scene(board_json, synth.box_mesh(0.19, 0.13, 0.16))
    shots = [(0, "view", 0.0, False), (1, "view", 15.0, False), (2, "view", 30.0, True), (3, "view", 45.0, False)]
    truth = write_role(w, "L", scene, cfg, shots, np.random.default_rng(1))
    w.close()
    return w.scan_dir, truth


@pytest.fixture
def scan_dir(built_base, tmp_path):
    src, truth = built_base
    dst = tmp_path / src.name
    shutil.copytree(src, dst)
    return dst, truth


def test_raw_roundtrip(tmp_path, repo_root, rng):
    w = new_scan(tmp_path, repo_root)
    rw = w.role("H")
    hello = {"ok": True, "role": "H", "serial": "X1", "calib": synth.default_calib()}
    rw.calib(hello)
    sent = []
    for i, (use, k, phi, compress) in enumerate([("view", 0, 0.0, False), ("retake", 0, 0.0, False),
                                                  ("view", 1, 15.0, True)]):
        color = rng.integers(0, 256, 64 * 48 * 2, np.uint8).tobytes()
        depth = np.repeat(np.arange(32 * 24, dtype="<u2"), 3).tobytes()
        hdr = {"ok": True, "t": 10.0 + i, "odom": [0.1 * i, 0.0, 0.0],
               "color": {"w": 64, "h": 48, "fmt": "yuyv"}, "depth": {"w": 32, "h": 24, "fmt": "z16", "n": 3}}
        got_hdr, payloads = through_channel(hdr, {"color": color, "depth": depth}, compress)
        assert payloads == [color, depth]
        snap = rw.snapshot(got_hdr, payloads, k, use, phi)
        assert snap == f"{i:06d}"
        sent.append((snap, got_hdr, payloads, k, use, phi))
    lines = [({"type": "odom", "t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0}, "in", 100.0),
             ({"type": "status", "t": 1.1, "text": "done: move +0.17 m (모든 명령 완료)"}, "in", 100.1),
             ({"cmd": "move", "text": "speed 0.06; forward 0.17", "timeout_s": 60}, "out", 100.2),
             ({"type": "stop"}, "out", 100.3)]
    for msg, direction, t in lines:
        rw.stream(msg, direction, t)
    w.close()

    raw = w.scan_dir / "H" / "raw"
    names = sorted(p.name for p in raw.iterdir())
    want = ["calib.json", "stream.jsonl"]
    for snap, *_ in sent:
        want += [f"{snap}.json", f"{snap}_color.yuyv", f"{snap}_depth.z16"]
    assert names == sorted(want)
    assert json.loads((raw / "calib.json").read_text(encoding="utf-8")) == hello
    for snap, hdr, payloads, k, use, phi in sent:
        assert "zlib" not in hdr["color"] and "raw_bytes" not in hdr["color"]
        stored = json.loads((raw / f"{snap}.json").read_text(encoding="utf-8"))
        assert stored == dict(hdr, station={"k": k, "use": use, "phi_target": phi})
        for key, data in zip(("color", "depth"), payloads):
            f = raw / f"{snap}_{key}.{hdr[key]['fmt']}"
            assert f.stat().st_size == hdr[key]["bytes"] == len(data)
            assert f.read_bytes() == data
    got = [json.loads(s) for s in (raw / "stream.jsonl").read_text(encoding="utf-8").splitlines()]
    assert got == [dict(m, pc_t=t, dir=d) for m, d, t in lines]
    assert (w.scan_dir / "config.yaml").read_bytes() == (tmp_path / "scan.yaml").read_bytes()
    assert (w.scan_dir / "board.json").read_bytes() == (repo_root / "config" / "board.json").read_bytes()


def test_build(scan_dir):
    sd, truth = scan_dir
    views = dataset.build(sd, "L")
    assert [v["k"] for v in views] == [0, 1, 3]
    for v in views:
        name = f"{v['k']:06d}"
        c = cv2.imread(str(sd / "L" / "color" / f"{name}.jpg"))
        d = cv2.imread(str(sd / "L" / "depth" / f"{name}.png"), cv2.IMREAD_UNCHANGED)
        assert c.shape == (720, 1280, 3) and d.shape == (720, 1280) and d.dtype == np.uint16
        dp, da = pose_error(v["T_cam_board"], truth[v["snap"]])
        print(f"build k {v['k']}: {v['n_tags']} tags, {dp:.2f} mm, {da:.3f} deg, depth_invalid {v['depth_invalid']:.4f}")
        assert dp < 3.0 and da < 1.0
        assert 0.0 <= v["depth_invalid"] < 1.0
    K, w, h, loaded = dataset.load(sd / "L")
    assert (w, h) == (1280, 720)
    assert [v["k"] for v in loaded] == [0, 1, 3]
    assert loaded[0]["T_cam_board"].shape == (4, 4)
    out = {o["k"]: o for o in json.loads((sd / "L" / "outcomes.json").read_text(encoding="utf-8"))}
    assert set(out) == set(range(24))
    assert out[0]["pose"] and out[0]["d_source"] == "tag"
    assert out[2]["captured"] and not out[2]["pose"] and out[2]["reason"] == "no_tags"
    assert not out[5]["captured"] and out[5]["reason"] is None


def _outputs(role_dir):
    files = sorted(p for p in role_dir.rglob("*") if p.is_file() and "raw" not in p.relative_to(role_dir).parts)
    return {str(p.relative_to(role_dir)): p.read_bytes() for p in files}


def test_build_deterministic(scan_dir):
    sd, _ = scan_dir
    dataset.build(sd, "L")
    first = _outputs(sd / "L")
    dataset.build(sd, "L")
    second = _outputs(sd / "L")
    assert set(first) == set(second)
    assert len(first) >= 3 + 3 + 3
    for name in first:
        assert first[name] == second[name], name


def test_build_stale(scan_dir):
    sd, truth = scan_dir
    dataset.build(sd, "L")
    raw = sd / "L" / "raw"
    snap1 = next(s for s, T in truth.items()
                 if json.loads((raw / f"{s}.json").read_text(encoding="utf-8"))["station"]["k"] == 1)
    calib = synth.default_calib()
    (raw / f"{snap1}_color.yuyv").write_bytes(synth.to_yuyv(gray_color(calib)))
    dataset.build(sd, "L")
    assert sorted(p.name for p in (sd / "L" / "color").iterdir()) == ["000000.jpg", "000003.jpg"]
    assert sorted(p.name for p in (sd / "L" / "depth").iterdir()) == ["000000.png", "000003.png"]
    out = {o["k"]: o for o in json.loads((sd / "L" / "outcomes.json").read_text(encoding="utf-8"))}
    assert out[1]["reason"] == "no_tags" and not out[1]["pose"]


@pytest.mark.parametrize("case,view_tags,retake_tags,want", [
    ("가", True, True, "retake"), ("나", True, False, "view"), ("다", False, False, None)])
def test_retake(tmp_path, repo_root, board_json, cfg, case, view_tags, retake_tags, want):
    w = new_scan(tmp_path, repo_root)
    scene = synth.Scene(board_json)
    shots = [(4, "view", 60.0, not view_tags), (4, "retake", 62.0, not retake_tags)]
    truth = write_role(w, "H", scene, cfg, shots, np.random.default_rng(2))
    w.close()
    views = dataset.build(w.scan_dir, "H")
    snaps = list(truth)
    if want is None:
        assert views == []
        out = {o["k"]: o for o in json.loads((w.scan_dir / "H" / "outcomes.json").read_text(encoding="utf-8"))}
        assert out[4]["captured"] and out[4]["reason"] == "no_tags"
    else:
        assert [v["k"] for v in views] == [4]
        assert views[0]["snap"] == snaps[0 if want == "view" else 1]
        assert pose_error(views[0]["T_cam_board"], truth[views[0]["snap"]])[0] < 3.0


def test_outcomes_events(tmp_path, repo_root, board_json, cfg):
    """events.jsonl의 우회(`skip`)와 odom `d`(`view`)."""
    w = new_scan(tmp_path, repo_root)
    write_role(w, "L", synth.Scene(board_json), cfg, [(0, "view", 0.0, False), (3, "view", 45.0, True)],
               np.random.default_rng(3))
    w.event({"type": "skip", "role": "L", "k": 1, "reason": "bypass", "pc_t": 1.0})
    w.event({"type": "skip", "role": "L", "k": 2, "reason": "bypass", "pc_t": 1.0})
    w.event({"type": "skip", "role": "H", "k": 5, "reason": "bypass", "pc_t": 1.0})
    w.event({"type": "view", "role": "L", "k": 3, "use": "view", "snap": "000001", "pose": False,
             "d": 0.47, "d_source": "odom", "pc_t": 2.0})
    w.close()
    dataset.build(w.scan_dir, "L")
    out = {o["k"]: o for o in json.loads((w.scan_dir / "L" / "outcomes.json").read_text(encoding="utf-8"))}
    assert out[0]["pose"] and out[0]["d_source"] == "tag"
    assert out[0]["d"] == pytest.approx(cfg["scene"]["R"] - cfg["scene"]["r_obj"] - cfg["scene"]["place_err"], abs=0.005)
    for k in (1, 2):
        assert out[k] == {"k": k, "captured": False, "pose": False, "d": None, "d_source": None,
                          "reason": "bypass", "crop": None, "fused": None}
    assert out[3]["captured"] and out[3]["reason"] == "no_tags"
    assert out[3]["d"] == 0.47 and out[3]["d_source"] == "odom"
    assert out[5]["reason"] is None


def test_intrinsic_open3d(scan_dir):
    sd, _ = scan_dir
    dataset.build(sd, "L")
    intr = o3d.io.read_pinhole_camera_intrinsic(str(sd / "L" / "camera_intrinsic.json"))
    K, w, h, _ = dataset.load(sd / "L")
    c = synth.default_calib()["color"]
    assert (intr.width, intr.height) == (w, h) == (c["w"], c["h"])
    assert np.allclose(intr.intrinsic_matrix, K)
    assert np.allclose(K, [[c["fx"], 0, c["ppx"]], [0, c["fy"], c["ppy"]], [0, 0, 1]])
