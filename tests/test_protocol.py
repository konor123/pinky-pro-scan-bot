import math
import socket
import threading
import time

import pytest

from scan3d import protocol

KOREAN_DONE = "done: turn +8 deg (모든 명령 완료)"


def _pair():
    a, b = socket.socketpair()
    return a, b


def _send_bytewise(sock, data):
    for i in range(len(data)):
        sock.sendall(data[i:i + 1])


def _snapshot_messages():
    """4.4의 영상 응답(작은 크기). 바이트 길이 규칙을 그대로 따른다."""
    color = bytes(range(16))            # 4×2 YUY2
    depth = bytes(range(16, 16 + 32))   # 4×2 Z16 × 2장
    ir = bytes([7] * 8)                 # 4×2 Y8
    hdr, pl = protocol.pack_snapshot(
        {"ok": True, "t": 12.5, "odom": [0.1, -0.2, 0.3],
         "color": {"w": 4, "h": 2, "fmt": "yuyv"},
         "depth": {"w": 4, "h": 2, "fmt": "z16", "n": 2}},
        {"color": color, "depth": depth}, compress=False)
    hdr_ir, pl_ir = protocol.pack_snapshot(
        {"ok": True, "t": 13.0, "odom": [0.0, 0.0, 0.0],
         "ir": {"w": 4, "h": 2, "fmt": "y8"}},
        {"ir": ir}, compress=False)
    return [(hdr, pl), (hdr_ir, pl_ir)]


ALL_MESSAGES = [
    ({"cmd": "hello", "color": [1280, 720, 15], "depth": [848, 480, 15]}, []),
    ({"ok": True, "role": "L", "serial": "123456789012",
      "calib": {"color": {"w": 1280, "h": 720, "fx": 640.0, "fy": 640.0, "ppx": 639.5, "ppy": 359.5,
                          "model": "inverse_brown_conrady", "coeffs": [0.1, -0.2, 0.0, 0.001, 0.05]},
                "depth": {"w": 848, "h": 480, "fx": 447.0, "fy": 447.0, "ppx": 423.5, "ppy": 239.5,
                          "model": "brown_conrady", "coeffs": [0, 0, 0, 0, 0]},
                "depth_to_color": {"rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1], "translation": [-0.059, 0, 0]},
                "depth_scale": 0.001}}, []),
    ({"cmd": "move", "text": "speed 0.06; forward 0.17", "timeout_s": 60}, []),
    ({"ok": True, "stopped": False, "odom": [0.17, 0.0, 0.01]}, []),
    ({"ok": True, "stopped": True, "odom": [0.05, 0.0, 0.0]}, []),
    ({"cmd": "snapshot", "n_depth": 5, "n_skip": 15, "compress": False,
      "color": [1280, 720, 15], "depth": [848, 480, 15]}, []),
    ({"cmd": "snapshot", "ir": True, "n_skip": 15, "depth": [848, 480, 15]}, []),
    ({"ok": False, "error": "profile"}, []),
    ({"type": "odom", "t": 1.25, "x": 0.1, "y": 0.2, "yaw": -0.3}, []),
    (protocol.encode_scan(1.3, -math.pi, 2 * math.pi / 8, 0.05, 12.0,
                          [0.5, 1.0, math.inf, 0.2, 0.3, 0.4, 0.6, 0.7]), []),
    ({"type": "status", "t": 1.4, "text": KOREAN_DONE}, []),
    ({"type": "stop"}, []),
    ({"type": "abort"}, []),
] + _snapshot_messages()


def test_roundtrip():
    a, b = _pair()
    ch = protocol.Channel(b)
    wire = b"".join(protocol.encode(obj, pl) for obj, pl in ALL_MESSAGES)
    t = threading.Thread(target=_send_bytewise, args=(a, wire))
    t.start()
    try:
        for obj, pl in ALL_MESSAGES:
            got, got_pl = ch.recv(5.0)
            assert got == obj
            assert list(got_pl) == list(pl)
    finally:
        t.join()
        a.close()
        b.close()


def test_snapshot_sizes():
    color = bytes(1280 * 720 * 2)
    depth = bytes(848 * 480 * 2 * 5)
    hdr, pl = protocol.pack_snapshot(
        {"ok": True, "t": 1.0, "odom": [0, 0, 0],
         "color": {"w": 1280, "h": 720, "fmt": "yuyv"},
         "depth": {"w": 848, "h": 480, "fmt": "z16", "n": 5}},
        {"color": color, "depth": depth}, compress=False)
    assert hdr["color"]["bytes"] == 1_843_200
    assert hdr["depth"]["bytes"] == 4_070_400
    a, b = _pair()
    tx, rx = protocol.Channel(a), protocol.Channel(b)
    t = threading.Thread(target=tx.send, args=(hdr, pl))
    t.start()
    got, got_pl = rx.recv(10.0)
    t.join()
    assert got["color"]["bytes"] == 1_843_200 and got["depth"]["bytes"] == 4_070_400
    assert [len(p) for p in got_pl] == [1_843_200, 4_070_400]
    a.close()
    b.close()


def test_zlib():
    color = bytes(range(256)) * 7200          # 1,843,200 B
    depth = (b"\x10\x27" * 2000) * 1017 + b"\x00" * 1200 * 2   # 4,070,400 B
    assert len(color) == 1_843_200 and len(depth) == 4_070_400
    hdr, pl = protocol.pack_snapshot(
        {"ok": True, "t": 1.0, "odom": [0, 0, 0],
         "color": {"w": 1280, "h": 720, "fmt": "yuyv"},
         "depth": {"w": 848, "h": 480, "fmt": "z16", "n": 5}},
        {"color": color, "depth": depth}, compress=True)
    assert hdr["color"]["zlib"] is True and hdr["color"]["raw_bytes"] == len(color)
    assert hdr["depth"]["bytes"] == len(pl[1]) < len(depth)
    a, b = _pair()
    tx, rx = protocol.Channel(a), protocol.Channel(b)
    t = threading.Thread(target=tx.send, args=(hdr, pl))
    t.start()
    got, got_pl = rx.recv(10.0)
    t.join()
    assert got_pl[0] == color and got_pl[1] == depth
    for key, raw in (("color", color), ("depth", depth)):
        assert got[key]["bytes"] == len(raw)
        assert "zlib" not in got[key] and "raw_bytes" not in got[key]
    a.close()
    b.close()


def test_timeout():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    ch = protocol.connect("127.0.0.1", srv.getsockname()[1])
    conn, _ = srv.accept()
    ch.send({"cmd": "hello", "color": [1280, 720, 15], "depth": [848, 480, 15]})
    t0 = time.monotonic()
    with pytest.raises(TimeoutError):
        ch.recv(0.2)
    assert time.monotonic() - t0 < 1.0
    conn.close()
    ch.close()
    srv.close()


def test_timeout_keeps_partial():
    a, b = _pair()
    ch = protocol.Channel(b)
    wire = protocol.encode({"type": "odom", "t": 1.0, "x": 0, "y": 0, "yaw": 0})
    a.sendall(wire[:5])
    with pytest.raises(TimeoutError):
        ch.recv(0.1)
    a.sendall(wire[5:])
    got, _ = ch.recv(1.0)
    assert got["type"] == "odom"
    a.close()
    b.close()


def test_scan_codec():
    msg = protocol.encode_scan(2.0, -math.pi, 0.01, 0.05, 12.0, [0.5, math.inf, math.nan, 0.01, 20.0])
    assert msg["type"] == "scan"
    assert msg["ranges_mm"] == [500, 0, 0, 0, 0]
    assert msg["angle_inc"] == 0.01 and msg["range_min"] == 0.05 and msg["range_max"] == 12.0
    dec = protocol.decode_scan(msg)
    assert dec["ranges"] == [0.5, math.inf, math.inf, math.inf, math.inf]
    assert dec["angle_min"] == -math.pi and dec["angle_inc"] == 0.01 and dec["t"] == 2.0


def test_korean_text():
    a, b = _pair()
    tx, rx = protocol.Channel(a), protocol.Channel(b)
    tx.send({"type": "status", "t": 0.0, "text": KOREAN_DONE})
    got, _ = rx.recv(1.0)
    assert got["text"] == KOREAN_DONE
    a.close()
    b.close()


@pytest.mark.parametrize("cut", ["line", "payload"])
def test_disconnect(cut):
    a, b = _pair()
    ch = protocol.Channel(b)
    if cut == "line":
        a.sendall(b'{"type": "odom", "t": 1')
    else:
        hdr, pl = protocol.pack_snapshot(
            {"ok": True, "t": 1.0, "odom": [0, 0, 0], "ir": {"w": 4, "h": 2, "fmt": "y8"}},
            {"ir": bytes(8)}, compress=False)
        a.sendall(protocol.encode(hdr, pl)[:-3])
    a.close()
    with pytest.raises(ConnectionError):
        ch.recv(1.0)
    b.close()


def test_concurrent_send():
    a, b = _pair()
    tx, rx = protocol.Channel(a), protocol.Channel(b)
    n = 200

    def worker(tag):
        for i in range(n):
            tx.send({"type": "status", "t": i, "text": tag * 50})

    ts = [threading.Thread(target=worker, args=(c,)) for c in "ab"]
    for t in ts:
        t.start()
    seen = {"a": 0, "b": 0}
    for _ in range(2 * n):
        got, _ = rx.recv(5.0)
        assert got["text"] in ("a" * 50, "b" * 50)
        seen[got["text"][0]] += 1
    for t in ts:
        t.join()
    assert seen == {"a": n, "b": n}
    a.close()
    b.close()
