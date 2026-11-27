"""robot_agent 완료 테스트(docs/specs/robot_agent.md). ROS·카메라 없이 가짜 객체와 localhost TCP로 검증한다."""
import enum
import math
import os
import queue
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from scan3d import protocol

PKG = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[3]
COLOR_PROFILE = [1280, 720, 15]
DEPTH_PROFILE = [848, 480, 15]
COLOR_BYTES = 1280 * 720 * 2
DEPTH_FRAME_BYTES = 848 * 480 * 2
IR_BYTES = 848 * 480


class FakeBus:
    """node.py의 RosBus와 같은 인터페이스를 제공하는 테스트용 버스."""

    def __init__(self, subscribers: int = 1, on_publish=None):
        self.subscribers = subscribers
        self.on_publish = on_publish
        self.published: list[tuple[float, str]] = []
        self.twists: list[tuple[float, float, float]] = []
        self._status_q: queue.Queue[str] = queue.Queue()
        self._odom_cbs = []
        self._scan_cbs = []
        self._status_cbs = []
        self._odom = (0.0, 0.0, 0.0, 0.0)
        self._lock = threading.Lock()
        self.publish_event = threading.Event()

    def publish_command(self, text: str) -> None:
        with self._lock:
            self.published.append((time.monotonic(), text))
            self.publish_event.set()
        if self.on_publish is not None:
            self.on_publish(self, text)

    def command_subscribers(self) -> int:
        with self._lock:
            return self.subscribers

    def publish_twist(self, linear: float, angular: float) -> None:
        with self._lock:
            self.twists.append((time.monotonic(), float(linear), float(angular)))

    def set_subscribers(self, n: int) -> None:
        with self._lock:
            self.subscribers = n

    def next_status(self, timeout: float) -> str | None:
        try:
            return self._status_q.get(timeout=max(0.0, float(timeout)))
        except queue.Empty:
            return None

    def odom(self) -> tuple[float, float, float, float]:
        with self._lock:
            return self._odom

    def on_odom(self, fn) -> None:
        self._odom_cbs.append(fn)

    def on_scan(self, fn) -> None:
        self._scan_cbs.append(fn)

    def on_status(self, fn) -> None:
        self._status_cbs.append(fn)

    def emit_odom(self, t: float, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._odom = (float(t), float(x), float(y), float(yaw))
        for fn in list(self._odom_cbs):
            fn(float(t), float(x), float(y), float(yaw))

    def emit_scan(self, t: float, angle_min: float, angle_inc: float,
                  range_min: float, range_max: float, ranges) -> None:
        for fn in list(self._scan_cbs):
            fn(float(t), float(angle_min), float(angle_inc),
               float(range_min), float(range_max), list(ranges))

    def emit_status(self, t: float, text: str) -> None:
        self._status_q.put(text)
        for fn in list(self._status_cbs):
            fn(float(t), text)


def _sample_calib(color_profile, depth_profile) -> dict:
    cw, ch, _ = color_profile
    dw, dh, _ = depth_profile
    return {
        "color": {
            "w": cw, "h": ch, "fx": 640.0, "fy": 640.0,
            "ppx": (cw - 1) / 2.0, "ppy": (ch - 1) / 2.0,
            "model": "inverse_brown_conrady",
            "coeffs": [0.1, -0.2, 0.0, 0.001, 0.05],
        },
        "depth": {
            "w": dw, "h": dh, "fx": 447.0, "fy": 447.0,
            "ppx": (dw - 1) / 2.0, "ppy": (dh - 1) / 2.0,
            "model": "brown_conrady",
            "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "depth_to_color": {
            "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "translation": [-0.059, 0.0, 0.0],
        },
        "depth_scale": 0.001,
    }


class FakeCamera:
    """camera.py의 RealSenseCamera와 같은 인터페이스를 제공하는 가짜 카메라."""

    def __init__(self, serial_num: str = "FAKE-D455-01", snap_t: float = 12.5,
                 on_snapshot=None):
        self._serial = serial_num
        self.snap_t = snap_t
        self.on_snapshot = on_snapshot
        self.color_data = (bytes(range(256)) * (COLOR_BYTES // 256 + 1))[:COLOR_BYTES]
        self.depth_data = (
            b"".join(bytes([i % 251]) * DEPTH_FRAME_BYTES for i in range(5))
        )
        self.ir_data = bytes([42]) * IR_BYTES

    def serial(self) -> str:
        return self._serial

    def calibration(self, color_profile, depth_profile) -> dict:
        return _sample_calib(color_profile, depth_profile)

    def snapshot(self, color_profile, depth_profile, n_depth: int, n_skip: int):
        cw, ch, _ = color_profile
        dw, dh, _ = depth_profile
        if self.on_snapshot is not None:
            self.on_snapshot()
        depth_bytes = (self.depth_data[:DEPTH_FRAME_BYTES * n_depth]
                       if n_depth <= 5 else self.depth_data[:DEPTH_FRAME_BYTES] * n_depth)
        return (
            self.snap_t,
            {"w": cw, "h": ch, "fmt": "yuyv"},
            self.color_data,
            {"w": dw, "h": dh, "fmt": "z16", "n": n_depth},
            depth_bytes,
        )

    def snapshot_ir(self, depth_profile, n_skip: int):
        dw, dh, _ = depth_profile
        return (
            self.snap_t,
            {"w": dw, "h": dh, "fmt": "y8"},
            self.ir_data,
        )


def _make_fake_rs(fail_after_frames: int | None = None):
    """camera.py에 주입할 가짜 pyrealsense2 모듈."""

    class _Stream(enum.Enum):
        color = 1
        depth = 2
        infrared = 3

    class _Format(enum.Enum):
        yuyv = 1
        z16 = 2
        y8 = 3

    class _Distortion(enum.Enum):
        none = 0
        modified_brown_conrady = 1
        inverse_brown_conrady = 2
        brown_conrady = 3

    class _CameraInfo(enum.Enum):
        serial_number = 1

    class _Option(enum.Enum):
        emitter_enabled = 1

    records = {
        "configs": [],
        "starts": 0,
        "stops": 0,
        "emitter_history": [],
        "frame_calls": 0,
    }

    class _Intrinsics:
        def __init__(self, w, h, model):
            self.width = w
            self.height = h
            self.fx = 600.0
            self.fy = 600.0
            self.ppx = (w - 1) / 2.0
            self.ppy = (h - 1) / 2.0
            self.model = model
            self.coeffs = [0.1, -0.2, 0.0, 0.0, 0.0]

    class _Extrinsics:
        def __init__(self):
            self.rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.translation = [-0.059, 0.0, 0.0]

    class _StreamProfile:
        def __init__(self, stream_type, w, h, model):
            self._stream = stream_type
            self._intr = _Intrinsics(w, h, model)

        def as_video_stream_profile(self):
            return self

        def get_intrinsics(self):
            return self._intr

        def get_extrinsics_to(self, other):
            return _Extrinsics()

    class _DepthSensor:
        def __init__(self):
            self.emitter = 1.0

        def get_depth_scale(self) -> float:
            return 0.001

        def supports(self, opt) -> bool:
            return opt == _Option.emitter_enabled

        def get_option(self, opt) -> float:
            assert opt == _Option.emitter_enabled
            return self.emitter

        def set_option(self, opt, val: float) -> None:
            assert opt == _Option.emitter_enabled
            self.emitter = float(val)
            records["emitter_history"].append(float(val))

    class _Device:
        def __init__(self, sensor: _DepthSensor):
            self._sensor = sensor

        def get_info(self, key) -> str:
            assert key == _CameraInfo.serial_number
            return "944122070001"

        def first_depth_sensor(self):
            return self._sensor

    class _PipelineProfile:
        def __init__(self, enabled, device):
            self._enabled = enabled
            self._device = device

        def get_device(self):
            return self._device

        def get_stream(self, st):
            for entry in self._enabled:
                if entry[0] == st:
                    if st == _Stream.color:
                        return _StreamProfile(st, entry[2], entry[3], _Distortion.inverse_brown_conrady)
                    return _StreamProfile(st, entry[2], entry[3], _Distortion.brown_conrady)
            raise RuntimeError(f"stream {st} not enabled")

    class _Frame:
        def __init__(self, payload: bytes):
            self._payload = payload

        def get_data(self):
            return memoryview(self._payload)

    class _CompositeFrame:
        def __init__(self, idx: int):
            self.idx = idx

        def get_color_frame(self):
            return _Frame(bytes([self.idx]) * 16)

        def get_depth_frame(self):
            return _Frame(bytes([100 + self.idx]) * 12)

        def get_infrared_frame(self, index: int = 0):
            return _Frame(bytes([200 + self.idx]) * 8)

    class _Config:
        def __init__(self):
            self.enabled = []

        def enable_stream(self, *args):
            if len(args) == 5:
                st, w, h, fmt, fps = args
                self.enabled.append((st, 0, w, h, fmt, fps))
            elif len(args) == 6:
                st, idx, w, h, fmt, fps = args
                self.enabled.append((st, idx, w, h, fmt, fps))
            else:
                raise TypeError(f"unexpected enable_stream args: {args}")

    sensor = _DepthSensor()
    device = _Device(sensor)

    class _Context:
        def query_devices(self):
            return [device]

    class _Pipeline:
        def __init__(self):
            self._idx = 0

        def start(self, cfg):
            records["starts"] += 1
            records["configs"].append(list(cfg.enabled))
            self._idx = 0
            return _PipelineProfile(cfg.enabled, device)

        def wait_for_frames(self, timeout_ms: int = 5000):
            idx = self._idx
            self._idx += 1
            records["frame_calls"] += 1
            if fail_after_frames is not None and records["frame_calls"] > fail_after_frames:
                raise RuntimeError("frame timeout")
            return _CompositeFrame(idx)

        def stop(self):
            records["stops"] += 1

    class _FakeRS:
        stream = _Stream
        format = _Format
        distortion = _Distortion
        camera_info = _CameraInfo
        option = _Option
        config = _Config
        pipeline = _Pipeline
        context = _Context

    return _FakeRS, records, sensor


@contextmanager
def _running_agent(cam=None, bus=None, role: str = "L", sub_wait_s: float = 5.0):
    from scan3d_robot.agent import Agent

    cam = cam or FakeCamera()
    bus = bus or FakeBus()
    agent = Agent(role, cam, bus, port=0, stream_port=0, host="127.0.0.1", sub_wait_s=sub_wait_s)
    t = threading.Thread(target=agent.serve_forever, daemon=True)
    t.start()
    try:
        yield agent, cam, bus
    finally:
        agent.shutdown()
        t.join(timeout=3.0)


def test_hello():
    with _running_agent() as (agent, cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            resp, payloads = ch.recv(5.0)
            assert payloads == []
            assert resp["ok"] is True
            assert resp["role"] == "L"
            assert resp["serial"] == cam.serial()
            calib = resp["calib"]
            assert set(calib.keys()) == {"color", "depth", "depth_to_color", "depth_scale"}
            for k in ("color", "depth"):
                assert set(calib[k].keys()) == {"w", "h", "fx", "fy", "ppx", "ppy", "model", "coeffs"}
                assert len(calib[k]["coeffs"]) == 5
                assert isinstance(calib[k]["model"], str)
            assert set(calib["depth_to_color"].keys()) == {"rotation", "translation"}
            assert len(calib["depth_to_color"]["rotation"]) == 9
            assert len(calib["depth_to_color"]["translation"]) == 3
        finally:
            ch.close()


def test_camera_pipeline():
    from scan3d_robot.camera import RealSenseCamera

    fake_rs, records, sensor = _make_fake_rs()
    cam = RealSenseCamera(rs_module=fake_rs)

    calib = cam.calibration(COLOR_PROFILE, DEPTH_PROFILE)
    assert calib["color"]["model"] == "inverse_brown_conrady"
    assert calib["depth"]["model"] == "brown_conrady"
    assert calib["depth_scale"] == pytest.approx(0.001)
    assert cam.serial() == "944122070001"
    assert records["starts"] == 1 and records["stops"] == 1

    t, c_hdr, c_bytes, d_hdr, d_bytes = cam.snapshot(COLOR_PROFILE, DEPTH_PROFILE, n_depth=5, n_skip=3)
    assert isinstance(t, float)
    assert c_hdr == {"w": 1280, "h": 720, "fmt": "yuyv"}
    assert d_hdr == {"w": 848, "h": 480, "fmt": "z16", "n": 5}
    assert records["starts"] == 2 and records["stops"] == 2
    assert records["frame_calls"] == 8  # skip 3 + keep 5
    # 버린 0,1,2 다음 첫 세트(idx=3) 컬러와 idx=3..7 깊이
    assert c_bytes == bytes([3]) * 16
    assert d_bytes == b"".join(bytes([100 + i]) * 12 for i in range(3, 8))
    snap_cfg = records["configs"][1]
    assert (fake_rs.stream.color, 0, 1280, 720, fake_rs.format.yuyv, 15) in snap_cfg
    assert (fake_rs.stream.depth, 0, 848, 480, fake_rs.format.z16, 15) in snap_cfg

    records["frame_calls"] = 0
    t_ir, ir_hdr, ir_bytes = cam.snapshot_ir(DEPTH_PROFILE, n_skip=2)
    assert isinstance(t_ir, float)
    assert ir_hdr == {"w": 848, "h": 480, "fmt": "y8"}
    assert ir_bytes == bytes([202]) * 8
    assert records["starts"] == 3 and records["stops"] == 3
    assert records["emitter_history"] == [0.0, 1.0]
    assert sensor.emitter == 1.0
    ir_cfg = records["configs"][2]
    assert ir_cfg == [(fake_rs.stream.infrared, 1, 848, 480, fake_rs.format.y8, 15)]

    # 예외가 나도 투사기 복구와 pipeline.stop() 보장
    fail_rs, fail_rec, fail_sensor = _make_fake_rs(fail_after_frames=1)
    fail_cam = RealSenseCamera(rs_module=fail_rs)
    with pytest.raises(RuntimeError):
        fail_cam.snapshot_ir(DEPTH_PROFILE, n_skip=2)
    assert fail_rec["stops"] == 1
    assert fail_rec["emitter_history"] == [0.0, 1.0]
    assert fail_sensor.emitter == 1.0


@pytest.mark.parametrize(
    "case",
    ["done", "stopped", "blocked", "error", "timeout"],
)
def test_move_paths(case):
    def on_pub(bus: FakeBus, text: str):
        if text == "stop":
            return
        if case == "done":
            bus.emit_status(1.0, f"accepted: {text} (대기 2단계)")
            bus.emit_status(1.1, "done: move +0.10 m")
            bus.emit_status(1.2, "done: turn +8 deg (모든 명령 완료)")
        elif case == "stopped":
            bus.emit_status(1.0, f"accepted: {text} (대기 1단계)")
            bus.emit_status(1.1, "stopped: 남은 명령 취소")
        elif case == "blocked":
            bus.emit_status(1.0, "blocked: 앞 0.15 m에 장애물")
        elif case == "error":
            bus.emit_status(1.0, "error: 빈 명령입니다")

    bus = FakeBus(subscribers=1, on_publish=on_pub)
    bus.emit_odom(2.0, 0.12, -0.04, 0.5)
    with _running_agent(bus=bus) as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            timeout_s = 0.2 if case == "timeout" else 5.0
            ch.send({"cmd": "move", "text": "speed 0.06; forward 0.17", "timeout_s": timeout_s})
            resp, _ = ch.recv(5.0)
            if case == "done":
                assert resp == {"ok": True, "stopped": False, "odom": [0.12, -0.04, 0.5]}
            elif case == "stopped":
                assert resp == {"ok": True, "stopped": True, "odom": [0.12, -0.04, 0.5]}
            elif case in ("blocked", "error"):
                assert resp["ok"] is False
                assert case in resp["error"]
            elif case == "timeout":
                assert resp["ok"] is False
                assert any(txt == "stop" for _, txt in bus.published)
        finally:
            ch.close()


def test_waits_subscriber():
    def on_pub(bus: FakeBus, text: str):
        bus.emit_status(1.0, "done: move +0.10 m (모든 명령 완료)")

    bus = FakeBus(subscribers=0, on_publish=on_pub)
    t_sub_ready = None

    def enable_sub():
        nonlocal t_sub_ready
        time.sleep(1.0)
        t_sub_ready = time.monotonic()
        bus.set_subscribers(1)

    with _running_agent(bus=bus) as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            helper = threading.Thread(target=enable_sub, daemon=True)
            helper.start()
            ch.send({"cmd": "move", "text": "forward 0.10", "timeout_s": 5.0})
            resp, _ = ch.recv(5.0)
            helper.join(timeout=2.0)
            assert resp["ok"] is True
            assert len(bus.published) == 1
            assert t_sub_ready is not None
            assert bus.published[0][0] >= t_sub_ready - 0.05
        finally:
            ch.close()


def test_snapshot_bytes():
    with _running_agent() as (agent, cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)

            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": False,
                "color": COLOR_PROFILE,
                "depth": DEPTH_PROFILE,
            })
            hdr, pls = ch.recv(10.0)
            assert hdr["ok"] is True
            assert hdr["color"]["bytes"] == 1_843_200
            assert hdr["depth"]["bytes"] == 4_070_400
            assert [len(p) for p in pls] == [1_843_200, 4_070_400]
            assert pls[0] == cam.color_data
            assert pls[1] == cam.depth_data

            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": True,
                "color": COLOR_PROFILE,
                "depth": DEPTH_PROFILE,
            })
            hdr_z, pls_z = ch.recv(10.0)
            assert hdr_z["ok"] is True
            assert "zlib" not in hdr_z["color"] and "zlib" not in hdr_z["depth"]
            assert hdr_z["color"]["bytes"] == 1_843_200
            assert hdr_z["depth"]["bytes"] == 4_070_400
            assert pls_z[0] == cam.color_data
            assert pls_z[1] == cam.depth_data

            ch.send({
                "cmd": "snapshot",
                "ir": True,
                "n_skip": 5,
                "depth": DEPTH_PROFILE,
            })
            hdr_ir, pls_ir = ch.recv(10.0)
            assert hdr_ir["ok"] is True
            assert hdr_ir["ir"] == {"w": 848, "h": 480, "fmt": "y8", "bytes": 407_040}
            assert pls_ir == [cam.ir_data]
        finally:
            ch.close()


def test_stream():
    with _running_agent() as (agent, _cam, bus):
        st = protocol.connect("127.0.0.1", agent.stream_port)
        try:
            # 서버가 스트림 연결을 accept할 때까지 짧은 동기화 확인
            sync_deadline = time.monotonic() + 2.0
            while time.monotonic() < sync_deadline:
                bus.emit_status(0.0, "sync")
                try:
                    msg, _ = st.recv(0.05)
                except TimeoutError:
                    continue
                if msg.get("type") == "status" and msg.get("text") == "sync":
                    break
            else:
                pytest.fail("stream connection was not accepted in time")

            for i in range(100):
                bus.emit_odom(i * 0.01, float(i), 0.1, 0.2)
            bus.emit_scan(1.5, -math.pi, 0.01, 0.05, 12.0, [0.5, math.inf, math.nan, 0.01, 20.0])
            bus.emit_status(1.6, "done: turn +8 deg (모든 명령 완료)")

            odom_msgs = []
            scan_msg = None
            status_msg = None
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and (scan_msg is None or status_msg is None):
                msg, _ = st.recv(1.0)
                if msg["type"] == "odom":
                    odom_msgs.append(msg)
                elif msg["type"] == "scan":
                    scan_msg = msg
                elif msg["type"] == "status" and msg.get("text") != "sync":
                    status_msg = msg

            assert 10 <= len(odom_msgs) <= 21
            for a, b in zip(odom_msgs, odom_msgs[1:]):
                assert b["t"] - a["t"] >= 0.05 - 1e-6
            assert scan_msg is not None
            assert scan_msg["ranges_mm"] == [500, 0, 0, 0, 0]
            assert status_msg is not None
            assert status_msg["text"] == "done: turn +8 deg (모든 명령 완료)"

            st.send({"type": "stop"})
            assert bus.publish_event.wait(timeout=2.0)
            assert any(txt == "stop" for _, txt in bus.published)
        finally:
            st.close()


def test_disconnect():
    bus = FakeBus(subscribers=1)
    with _running_agent(bus=bus) as (agent, _cam, _bus):
        ch1 = protocol.connect("127.0.0.1", agent.port)
        ch1.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
        ch1.recv(5.0)
        ch1.send({"cmd": "move", "text": "forward 0.20", "timeout_s": 5.0})
        assert bus.publish_event.wait(timeout=2.0)
        ch1.close()

        time.sleep(0.2)
        assert [txt for _, txt in bus.published] == ["forward 0.20"]

        ch2 = protocol.connect("127.0.0.1", agent.port)
        try:
            ch2.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            resp, _ = ch2.recv(2.0)
            assert resp["ok"] is True
        finally:
            ch2.close()


def test_abort_order():
    def on_pub(bus: FakeBus, text: str):
        if text == "stop":
            bus.emit_status(1.0, "stopped: 남은 명령 취소")
        elif text == "forward 0.05":
            bus.emit_status(1.0, "done: move +0.05 m (모든 명령 완료)")

    # (가) abort가 구독자를 기다리는 move보다 먼저 옴
    bus_a = FakeBus(subscribers=0, on_publish=on_pub)
    with _running_agent(bus=bus_a) as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        st = protocol.connect("127.0.0.1", agent.stream_port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            ch.send({"cmd": "move", "text": "forward 0.17", "timeout_s": 5.0})
            time.sleep(0.05)
            st.send({"type": "abort"})
            resp, _ = ch.recv(2.0)
            assert resp == {"ok": False, "error": "aborted"}
            assert not any("forward" in txt for _, txt in bus_a.published)
        finally:
            st.close()
            ch.close()

    # (나) abort가 이동 중에 옴 -> stop 발행, aborted 실패
    bus_b = FakeBus(subscribers=1, on_publish=on_pub)
    with _running_agent(bus=bus_b) as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        st = protocol.connect("127.0.0.1", agent.stream_port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            ch.send({"cmd": "move", "text": "forward 0.17", "timeout_s": 5.0})
            assert bus_b.publish_event.wait(timeout=2.0)
            st.send({"type": "abort"})
            resp, _ = ch.recv(2.0)
            assert resp == {"ok": False, "error": "aborted"}
            assert any(txt == "stop" for _, txt in bus_b.published)
        finally:
            st.close()
            ch.close()

    # (다) abort가 완료 뒤에 옴 -> 다음 move 거절, 새 hello 뒤에는 받음
    bus_c = FakeBus(subscribers=1, on_publish=on_pub)
    with _running_agent(bus=bus_c) as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        st = protocol.connect("127.0.0.1", agent.stream_port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            ch.send({"cmd": "move", "text": "forward 0.05", "timeout_s": 5.0})
            assert ch.recv(2.0)[0]["ok"] is True

            st.send({"type": "abort"})
            time.sleep(0.05)

            ch.send({"cmd": "move", "text": "forward 0.05", "timeout_s": 5.0})
            resp_rej, _ = ch.recv(2.0)
            assert resp_rej == {"ok": False, "error": "aborted"}
            assert sum(1 for _, txt in bus_c.published if txt == "forward 0.05") == 1

            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            assert ch.recv(2.0)[0]["ok"] is True
            ch.send({"cmd": "move", "text": "forward 0.05", "timeout_s": 5.0})
            assert ch.recv(2.0)[0]["ok"] is True
        finally:
            st.close()
            ch.close()


def test_profile_mismatch():
    with _running_agent() as (agent, _cam, _bus):
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)

            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": False,
                "color": [640, 480, 15],
                "depth": DEPTH_PROFILE,
            })
            assert ch.recv(2.0)[0] == {"ok": False, "error": "profile"}

            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": False,
                "color": COLOR_PROFILE,
                "depth": [640, 480, 15],
            })
            assert ch.recv(2.0)[0] == {"ok": False, "error": "profile"}

            ch.send({
                "cmd": "snapshot",
                "ir": True,
                "n_skip": 5,
                "depth": [640, 480, 15],
            })
            assert ch.recv(2.0)[0] == {"ok": False, "error": "profile"}
        finally:
            ch.close()


def test_snapshot_odom():
    bus = FakeBus()

    def during_snapshot():
        bus.emit_odom(10.7, 0.8, 0.6, 0.7)
        bus.emit_odom(10.9, 1.2, 1.1, 1.0)

    cam = FakeCamera(snap_t=10.5, on_snapshot=during_snapshot)

    with _running_agent(cam=cam, bus=bus) as (agent, _cam, _bus):
        bus.emit_odom(10.0, 0.0, 0.0, 0.0)
        bus.emit_odom(10.5, 0.5, 0.2, 0.3)
        ch = protocol.connect("127.0.0.1", agent.port)
        try:
            ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
            ch.recv(5.0)
            ch.send({
                "cmd": "snapshot",
                "n_depth": 5,
                "n_skip": 15,
                "compress": False,
                "color": COLOR_PROFILE,
                "depth": DEPTH_PROFILE,
            })
            hdr, _ = ch.recv(5.0)
            assert hdr["ok"] is True
            assert hdr["t"] == 10.5
            assert hdr["odom"] == [0.5, 0.2, 0.3]
        finally:
            ch.close()


def test_agent_imports_without_ros_or_realsense():
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{PKG}{os.pathsep}{REPO}"
    code = (
        "import sys; "
        "sys.modules['rclpy'] = None; "
        "sys.modules['pyrealsense2'] = None; "
        "import scan3d_robot.agent; "
        "assert 'rclpy' not in sys.modules or sys.modules['rclpy'] is None; "
        "assert 'pyrealsense2' not in sys.modules or sys.modules['pyrealsense2'] is None"
    )
    res = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True)
    assert res.returncode == 0, res.stderr


class DrivingBus(FakeBus):
    """마지막 `cmd_vel`대로 odom을 적분하는 버스(원호 실행 테스트용, 실제 시간의 `rate`배로 움직인다)."""

    def __init__(self, rate: float = 4.0, **kw):
        super().__init__(**kw)
        self.rate = rate
        self._run = True
        self._pose = [0.0, 0.0, 0.0]
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        last = time.monotonic()
        while self._run:
            time.sleep(0.002)
            now = time.monotonic()
            dt, last = (now - last) * self.rate, now
            with self._lock:
                v, w = self.twists[-1][1:] if self.twists else (0.0, 0.0)
            x, y, yaw = self._pose
            self._pose = [x + v * dt * math.cos(yaw + w * dt / 2), y + v * dt * math.sin(yaw + w * dt / 2),
                          yaw + w * dt]
            self.emit_odom(now, *self._pose)

    def close(self):
        self._run = False
        self._thread.join(1.0)


def _arc_move(agent, text, timeout_s=20.0, during=None):
    ch = protocol.connect("127.0.0.1", agent.port)
    st = protocol.connect("127.0.0.1", agent.stream_port)
    try:
        ch.send({"cmd": "hello", "color": COLOR_PROFILE, "depth": DEPTH_PROFILE})
        hello, _ = ch.recv(5.0)
        ch.send({"cmd": "move", "text": text, "timeout_s": timeout_s})
        if during is not None:
            during(st)
        resp, _ = ch.recv(timeout_s + 5.0)
        return hello, resp
    finally:
        ch.close()
        st.close()


@pytest.mark.parametrize("angle", [15.0, -15.0, 90.0])     # 90°: 보강 촬영의 긴 원호
def test_arc(angle):
    bus = DrivingBus()
    try:
        with _running_agent(bus=bus) as (agent, _cam, _bus):
            hello, resp = _arc_move(agent, f"speed 0.10; arc 0.62 {angle}")
        assert hello["arc"] is True
        assert resp["ok"] is True and resp["stopped"] is False
        assert bus.published == []                      # /pinky_command로 보내지 않는다
        moving = [(v, w) for _, v, w in bus.twists if v > 0]
        assert moving and all(w / v == pytest.approx(math.copysign(1 / 0.62, angle)) for v, w in moving)
        assert max(v for v, _ in moving) == pytest.approx(0.10)
        assert bus.twists[-1][1:] == (0.0, 0.0)         # 끝나면 0을 보낸다
        x, y, yaw = resp["odom"]
        length = 0.62 * math.radians(abs(angle))
        assert length - 0.012 <= abs(yaw) * 0.62 <= length + 0.005
        assert yaw * angle > 0
        centre_y = math.copysign(0.62, angle)
        assert math.hypot(x, y - centre_y) == pytest.approx(0.62, abs=0.003)
    finally:
        bus.close()


def test_arc_stop_abort_timeout_bad():
    bus = DrivingBus()
    try:
        with _running_agent(bus=bus) as (agent, _cam, _bus):
            def stop_soon(st):
                time.sleep(0.3)
                st.send({"type": "stop"})
            _, resp = _arc_move(agent, "arc 0.62 180", during=stop_soon)
            assert resp["ok"] is True and resp["stopped"] is True
            assert bus.twists[-1][1:] == (0.0, 0.0)
            assert abs(resp["odom"][2]) < math.radians(90)

            def abort_soon(st):
                time.sleep(0.3)
                st.send({"type": "abort"})
            _, resp = _arc_move(agent, "arc 0.62 180", during=abort_soon)
            assert resp == {"ok": False, "error": "aborted"}
            assert bus.twists[-1][1:] == (0.0, 0.0)

            _, resp = _arc_move(agent, "arc 0.62 180", timeout_s=0.3)   # 새 hello로 중단 상태가 풀린다
            assert resp == {"ok": False, "error": "timeout"}
            assert bus.twists[-1][1:] == (0.0, 0.0)

            n = len(bus.twists)
            _, resp = _arc_move(agent, "turn 5; arc 0.62 15")
            assert resp["ok"] is False and resp["error"].startswith("error:")
            assert all(tw[1:] == (0.0, 0.0) for tw in bus.twists[n:])
    finally:
        bus.close()
