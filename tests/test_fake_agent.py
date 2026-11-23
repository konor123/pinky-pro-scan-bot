import json
import math
import threading

import numpy as np
import pytest

from scan3d import protocol

from scan3d.robot import drive
from scan3d.sim.fake_agent import FakeFleet
from scan3d.clock import Clock

SCALE = 5.0


def _hello(cfg):
    cap = cfg["capture"]
    return {"cmd": "hello",
            "color": [cap["color"]["w"], cap["color"]["h"], cap["color"]["fps"]],
            "depth": [cap["depth"]["w"], cap["depth"]["h"], cap["depth"]["fps"]]}


def _snap_req(cfg, compress=False):
    cap = cfg["capture"]
    req = _hello(cfg)
    req.update({"cmd": "snapshot", "n_depth": cap["n_depth"], "n_skip": cap["n_skip"], "compress": compress})
    return req


@pytest.fixture
def fleet_factory(cfg):
    made = []

    def make(**kw):
        clock = Clock(SCALE)
        fleet = FakeFleet(cfg, clock, **kw)
        addrs = fleet.start()
        made.append(fleet)
        return fleet, addrs, clock

    yield make
    for f in made:
        f.stop()


def _ctrl(addrs, role):
    host, port, _ = addrs[role]
    return protocol.connect(host, port)


def _stream(addrs, role):
    host, _, sport = addrs[role]
    return protocol.connect(host, sport)


def test_hello_snapshot(cfg, fleet_factory):
    fleet, addrs, _ = fleet_factory(roles=("L",))
    ch = _ctrl(addrs, "L")
    ch.send(_hello(cfg))
    hello, _ = ch.recv(30)
    assert hello["ok"] and hello["role"] == "L" and hello["serial"] == "FAKE-L"
    assert set(hello["calib"]) == {"color", "depth", "depth_to_color", "depth_scale"}
    n = cfg["capture"]["n_depth"]
    for compress in (False, True):
        ch.send(_snap_req(cfg, compress))
        hdr, pl = ch.recv(120)
        assert hdr["ok"]
        assert hdr["color"] == {"w": 1280, "h": 720, "fmt": "yuyv", "bytes": 1_843_200}
        assert hdr["depth"] == {"w": 848, "h": 480, "fmt": "z16", "n": n, "bytes": 814_080 * n}
        assert [len(p) for p in pl] == [1_843_200, 814_080 * n]
        assert len(hdr["odom"]) == 3
    ch.send({"cmd": "snapshot", "ir": True, "n_skip": 15, "depth": [848, 480, 15]})
    hdr, _ = ch.recv(30)
    assert hdr["ok"] is False
    bad = _snap_req(cfg)
    bad["color"] = [640, 480, 30]
    ch.send(bad)
    hdr, _ = ch.recv(30)
    assert hdr == {"ok": False, "error": "profile"}
    ch.close()


def test_move_stop(cfg, fleet_factory, rng):
    fleet, addrs, clock = fleet_factory(roles=("L",))
    ch, st = _ctrl(addrs, "L"), _stream(addrs, "L")
    ch.send(_hello(cfg))
    ch.recv(30)
    ch.send({"cmd": "move", "text": "speed 0.06; forward 0.3", "timeout_s": 60})
    clock.sleep(1.5)
    st.send({"type": "stop"})
    res, _ = ch.recv(30)
    assert res["ok"] and res["stopped"] is True
    assert any(r == "L" and what == "stop" for r, what, _ in fleet.stop_log)
    # stop은 이어 갈 수 있는 정지: 다음 move는 받는다
    ch.send({"cmd": "move", "text": "turn 10", "timeout_s": 60})
    res, _ = ch.recv(30)
    assert res["ok"] and res["stopped"] is False
    ch.close()
    st.close()


def test_abort(cfg, fleet_factory):
    fleet, addrs, clock = fleet_factory(roles=("L",))
    ch, st = _ctrl(addrs, "L"), _stream(addrs, "L")
    ch.send(_hello(cfg))
    ch.recv(30)
    ch.send({"cmd": "move", "text": "speed 0.06; forward 0.3", "timeout_s": 60})
    clock.sleep(1.0)
    st.send({"type": "abort"})
    res, _ = ch.recv(30)
    assert res == {"ok": False, "error": "aborted"}
    ch.send({"cmd": "move", "text": "turn 10", "timeout_s": 60})
    res, _ = ch.recv(30)
    assert res == {"ok": False, "error": "aborted"}
    ch.send(_hello(cfg))
    ch.recv(30)
    ch.send({"cmd": "move", "text": "turn 10", "timeout_s": 60})
    res, _ = ch.recv(30)
    assert res["ok"]
    ch.close()
    st.close()


def test_streams(cfg, fleet_factory):
    fleet, addrs, clock = fleet_factory(roles=("L",))
    st = _stream(addrs, "L")
    counts = {"odom": 0, "scan": 0}
    t_end = None
    while True:
        msg, _ = st.recv(10)
        if t_end is None:
            t_end = clock.now() + 2.0
        if clock.now() > t_end:
            break
        if msg["type"] in counts:
            counts[msg["type"]] += 1
        if msg["type"] == "scan":
            dec = protocol.decode_scan(msg)
            assert len(dec["ranges"]) > 0
    st.close()
    assert 32 <= counts["odom"] <= 48, counts
    assert 16 <= counts["scan"] <= 24, counts


def test_snapshot_pose(cfg, fleet_factory):
    from scan3d.robot import align, tagpose

    fleet, addrs, _ = fleet_factory(roles=("L", "H"))
    board = tagpose.load_board(fleet.board, cfg["board"]["tag_mm"])
    for role in ("L", "H"):
        ch = _ctrl(addrs, role)
        ch.send(_hello(cfg))
        hello, _ = ch.recv(30)
        ch.send(_snap_req(cfg))
        hdr, pl = ch.recv(120)
        frame = align.prepare(hdr, pl, hello["calib"])
        pose = tagpose.estimate(frame.color, frame.K, board)
        assert pose is not None and pose.n_tags >= 4
        T_true = drive.camera_pose(fleet.true_pose(role), cfg["robots"][role]["mount"])
        R_est, t_est = pose.T_cam_board[:3, :3], pose.T_cam_board[:3, 3]
        c_est = -R_est.T @ t_est
        c_true = -T_true[:3, :3].T @ T_true[:3, 3]
        assert np.linalg.norm(c_est - c_true) < 0.003, (role, c_est, c_true)
        dR = R_est @ T_true[:3, :3].T
        ang = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        assert ang < 1.0, (role, ang)
        ch.close()


def test_fail(cfg, fleet_factory):
    fleet, addrs, _ = fleet_factory(roles=("L",), fail={"role": "L", "cmd": "move", "nth": 2})
    ch = _ctrl(addrs, "L")
    ch.send(_hello(cfg))
    ch.recv(30)
    results = []
    for _ in range(3):
        ch.send({"cmd": "move", "text": "turn 5", "timeout_s": 60})
        res, _ = ch.recv(30)
        results.append(res["ok"])
    assert results == [True, False, True]
    ch.close()


def test_obstacle_position(cfg, fleet_factory):
    fleet, _, _ = fleet_factory(roles=("L",), obstacle=(2, 0.10))
    mount = cfg["robots"]["L"]["mount"]
    R = cfg["scene"]["R"]
    step = 360.0 / cfg["scene"]["n_views"]
    a = drive.view_pose(2 * step, R, mount)
    b = drive.view_pose(3 * step, R, mount)
    (x, y, w, d, yaw_deg), = fleet.world_boxes()
    assert math.hypot(x - (a[0] + b[0]) / 2, y - (a[1] + b[1]) / 2) < 1e-6
    assert w == d == pytest.approx(0.10)
    assert math.isclose(math.radians(yaw_deg) % math.pi,
                        math.atan2(b[1] - a[1], b[0] - a[0]) % math.pi, abs_tol=1e-6)
