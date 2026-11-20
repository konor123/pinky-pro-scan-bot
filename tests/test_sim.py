"""시뮬레이션 에이전트(가짜 버스) → align, Gazebo 자산과 시뮬레이션 설정."""
import copy
import json
import queue
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
import yaml

from scan3d import align, protocol, tagpose
from scan3d_robot.agent import Agent
from scan3d_sim.sim_camera import GazeboCamera
from tools.make_sim_assets import build_board_texture


class _FakeBus:
    """RosBus와 같은 인터페이스의 최소 테스트 버스."""

    def __init__(self):
        self._status_q: queue.Queue[str] = queue.Queue()
        self._odom = (0.0, 0.0, 0.0, 0.0)

    def publish_command(self, text: str) -> None:
        pass

    def command_subscribers(self) -> int:
        return 1

    def next_status(self, timeout: float):
        try:
            return self._status_q.get(timeout=max(0.0, float(timeout)))
        except queue.Empty:
            return None

    def odom(self):
        return self._odom

    def on_odom(self, fn) -> None:
        pass

    def on_scan(self, fn) -> None:
        pass

    def on_status(self, fn) -> None:
        pass


def test_gazebo_camera_agent_and_align_prepare():
    w, h = 640, 360
    cam = GazeboCamera(serial="SIM-L", timeout_s=5.0)
    bus = _FakeBus()
    agent = Agent("L", cam, bus, port=0, stream_port=0, host="127.0.0.1")
    srv = threading.Thread(target=agent.serve_forever, daemon=True)
    srv.start()

    try:
        k = [337.3, 0.0, 319.5, 0.0, 337.3, 179.5, 0.0, 0.0, 1.0]
        cam.on_camera_info(w, h, k)

        stale_rgb = np.full((h, w, 3), 17, dtype=np.uint8)
        stale_depth = np.full((h, w), 2.5, dtype="<f4")
        cam.on_color(w, h, "rgb8", stale_rgb.tobytes())
        cam.on_depth(w, h, "32FC1", stale_depth.tobytes())

        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            # hello에서 다른 프로파일을 요청해도 카메라 실제 크기로 응답한다.
            ch.send({"cmd": "hello", "color": [1280, 720, 15], "depth": [848, 480, 15]})
            resp, pls = ch.recv(5.0)
            assert pls == []
            assert resp["ok"] is True
            assert resp["role"] == "L"
            assert resp["serial"] == "SIM-L"
            calib = resp["calib"]
            assert calib["color"]["w"] == w and calib["color"]["h"] == h
            assert calib["depth"]["w"] == w and calib["depth"]["h"] == h
            assert calib["color"]["model"] == "none"

            yy, xx = np.mgrid[0:h, 0:w]
            fresh_rgb = np.stack([
                (xx * 255 // (w - 1)).astype(np.uint8),
                (yy * 255 // (h - 1)).astype(np.uint8),
                ((xx + yy) % 256).astype(np.uint8),
            ], axis=-1)
            depth_m = (0.50 + 0.15 * (yy / (h - 1))).astype("<f4")
            depth_m[0, :4] = [np.nan, np.inf, -0.5, 0.0]

            def feed_fresh():
                time.sleep(0.05)
                cam.on_color(w, h, "rgb8", fresh_rgb.tobytes())
                for _ in range(6):
                    cam.on_depth(w, h, "32FC1", depth_m.tobytes())
                    time.sleep(0.01)

            feeder = threading.Thread(target=feed_fresh, daemon=True)
            feeder.start()

            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": False,
                "color": [1280, 720, 15],
                "depth": [848, 480, 15],
            })
            hdr, payloads = ch.recv(5.0)
            feeder.join(timeout=2.0)

            assert hdr["ok"] is True
            assert hdr["color"] == {"w": w, "h": h, "fmt": "rgb8", "bytes": w * h * 3}
            assert hdr["depth"] == {"w": w, "h": h, "fmt": "f32", "n": 5, "bytes": w * h * 4 * 5}
            assert payloads[0] == fresh_rgb.tobytes()
            assert payloads[1] == depth_m.tobytes() * 5

            fr = align.prepare(hdr, payloads, calib)
            assert np.array_equal(fr.color, fresh_rgb[..., ::-1])
            assert np.array_equal(fr.K, align.K_of(calib["color"]))
            assert (fr.depth[0, :4] == 0).all()

            valid = np.isfinite(depth_m) & (depth_m > 0)
            assert (fr.depth[valid] > 0).mean() >= 0.95
            sel = valid & (fr.depth > 0)
            want_mm = np.round(depth_m[sel] * 1000.0).astype(np.int64)
            got_mm = fr.depth[sel].astype(np.int64)
            assert np.abs(got_mm - want_mm).max() <= 1

            ch.send({"cmd": "snapshot", "ir": True, "n_skip": 5, "depth": [848, 480, 15]})
            ir_resp, _ = ch.recv(5.0)
            assert ir_resp["ok"] is False
        finally:
            ch.close()
    finally:
        agent.shutdown()
        srv.join(timeout=2.0)


def test_sim_assets_and_configs(tmp_path, cfg, repo_root):
    board_path = repo_root / "config" / "board.json"
    out_png = tmp_path / "board.png"
    res = build_board_texture(board_path, out_png)
    assert res == out_png
    assert out_png.exists()

    img = cv2.imread(str(out_png), cv2.IMREAD_UNCHANGED)
    assert img is not None
    assert img.ndim == 3 and img.shape[2] == 3
    assert img.shape[0] == img.shape[1]

    board_json = json.loads(board_path.read_text(encoding="utf-8"))
    board = tagpose.load_board(board_json, cfg["board"]["tag_mm"])
    corners, ids = tagpose.detect(img, board)
    assert len(ids) == 16
    centers = {int(tid): c.reshape(4, 2).mean(axis=0) for c, tid in zip(corners, ids.ravel())}
    h, w = img.shape[:2]
    assert centers[15][0] > w / 2 and centers[15][1] < h / 2
    assert centers[0][0] < w / 2 and centers[0][1] > h / 2

    room = yaml.safe_load((repo_root / "ros" / "scan3d_sim" / "layouts" / "scan3d_room.yaml").read_text(encoding="utf-8"))
    assert room["world_name"] == "scan3d_room"
    walls = [o for o in room["objects"] if o["type"] == "wall"]
    assert len(walls) == 4
    xs = [p[0] for w_ in walls for p in (w_["from"], w_["to"])]
    ys = [p[1] for w_ in walls for p in (w_["from"], w_["to"])]
    assert min(xs) == pytest.approx(-1.5) and max(xs) == pytest.approx(1.5)
    assert min(ys) == pytest.approx(-1.5) and max(ys) == pytest.approx(1.5)

    by_name = {o["name"]: o for o in room["objects"]}
    mb = by_name["marker_board"]
    assert mb["type"] == "box"
    assert mb["size"] == [0.594, 0.594, 0.002]
    assert mb["at"] == [0.0, 0.0]
    assert mb["texture"] == "board.png"

    targets = [o for o in room["objects"] if o["name"].startswith("target_")]
    assert len(targets) >= 5
    colors = {tuple(t["color"]) for t in targets}
    assert len(colors) == len(targets)

    boxes = []
    for t in targets:
        sx, sy, sz = t["size"]
        ax, ay = t["at"]
        boxes.append((ax - sx / 2, ax + sx / 2, ay - sy / 2, ay + sy / 2, sz))
    assert min(b[0] for b in boxes) == pytest.approx(-0.09)
    assert max(b[1] for b in boxes) == pytest.approx(0.09)
    assert min(b[2] for b in boxes) == pytest.approx(-0.07)
    assert max(b[3] for b in boxes) == pytest.approx(0.07)
    assert max(b[4] for b in boxes) == pytest.approx(0.162)
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            ax0, ax1, ay0, ay1, _ = boxes[i]
            bx0, bx1, by0, by1, _ = boxes[j]
            ox = min(ax1, bx1) - max(ax0, bx0)
            oy = min(ay1, by1) - max(ay0, by0)
            assert ox <= 1e-9 or oy <= 1e-9

    sim_cfg = yaml.safe_load((repo_root / "config" / "scan_sim.yaml").read_text(encoding="utf-8"))
    base_cmp = copy.deepcopy(cfg)
    sim_cmp = copy.deepcopy(sim_cfg)
    assert sim_cfg["capture"]["color"] == {"w": 1280, "h": 720, "fmt": "rgb8", "fps": 15}
    assert sim_cfg["capture"]["depth"] == {"w": 1280, "h": 720, "fmt": "f32", "fps": 15}
    for role in ("L", "H"):
        assert sim_cfg["robots"][role]["host"] == "127.0.0.1"
        base_cmp["robots"][role]["host"] = "127.0.0.1"
    base_cmp["capture"]["color"] = sim_cmp["capture"]["color"]
    base_cmp["capture"]["depth"] = sim_cmp["capture"]["depth"]
    assert sim_cmp == base_cmp

    xacro_path = repo_root / "ros" / "scan3d_sim" / "urdf" / "scan3d_robot.urdf.xacro"
    tree = ET.parse(xacro_path)
    root = tree.getroot()
    arg_names = {el.attrib.get("name") for el in root.iter() if el.tag.endswith("arg")}
    assert {"d455_x", "d455_y", "d455_z", "d455_yaw_deg", "d455_pitch_deg"}.issubset(arg_names)
    xacro_txt = xacro_path.read_text(encoding="utf-8")
    assert "<near>0.4</near>" in xacro_txt
    assert '<mass value="0.116"/>' in xacro_txt
