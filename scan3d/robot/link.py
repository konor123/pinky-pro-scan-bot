"""[로봇 1대] 스캔 PC ↔ 로봇 에이전트 연결(docs/specs/station.md, protocol.md).

제어 연결(hello·move·snapshot 요청과 응답)과 스트림 연결(odom·LiDAR·상태 수신, stop·abort 송신).
drive.RobotIO를 구현하므로 drive.Driver가 그대로 쓴다.
"""
import math
import threading

from .. import protocol
from ..clock import Clock
from . import drive

HELLO_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0
MOVE_MARGIN = 5.0
MOVE_TIMEOUT = 60.0
WATCH_PERIOD = 0.1


class RemoteError(RuntimeError):
    """로봇의 실패 응답(`ok:false`)."""


class RobotLink:
    """로봇 하나와의 연결. 받은 메시지는 raw_writer에 그대로 남긴다."""

    def __init__(self, role, host, port, stream_port, raw_writer, clock, cancel, capture=None):
        self.role = role
        self.raw = raw_writer
        self.clock = clock
        self.cancel = cancel
        self.capture = capture
        self.calib = None
        self.aborted = False
        self._odom = None
        self._odom_t = None
        self._scan = None
        self._scan_t = None
        self._raw_lock = threading.Lock()
        self.ctrl = protocol.connect(host, port)
        self.stream = protocol.connect(host, stream_port)
        self._reader = threading.Thread(target=self._read_stream, daemon=True)
        self._reader.start()

    # ---- 기록 ----
    def _record(self, msg, direction):
        if self.raw is None:
            return
        with self._raw_lock:
            self.raw.stream(msg, direction, self.clock.now())

    def _read_stream(self):
        while True:
            try:
                msg, _ = self.stream.recv(0.5)
            except TimeoutError:
                if self.aborted:
                    return
                continue
            except (ConnectionError, OSError, ValueError):
                return
            typ = msg.get("type")
            if typ == "odom":
                self._odom = (msg["x"], msg["y"], msg["yaw"])
                self._odom_t = self.clock.now()
            elif typ == "scan":
                self._scan = protocol.decode_scan(msg)
                self._scan_t = self.clock.now()
            self._record(msg, "in")

    def alive(self):
        """스트림을 아직 받고 있고 중단하지 않았는지(웹 화면 연결 표시)."""
        return self._reader.is_alive() and not self.aborted

    def ages(self):
        """(odom을 받은 뒤 지난 s, LiDAR 스캔을 받은 뒤 지난 s). 받은 적 없으면 None."""
        now = self.clock.now()
        return (None if self._odom_t is None else now - self._odom_t,
                None if self._scan_t is None else now - self._scan_t)

    def wait_streams(self, timeout=5.0):
        end = self.clock.now() + timeout
        while self._odom is None or self._scan is None:
            if self.clock.now() > end:
                raise RemoteError(f"{self.role}: odom·LiDAR 스트림이 오지 않음")
            self.clock.sleep(0.05)

    # ---- 요청 ----
    def _check_cancel(self):
        if self.aborted or self.cancelled():
            raise drive.Cancelled(f"{self.role}: 취소됨")

    def _request(self, obj, timeout, watch=None):
        self._check_cancel()
        self.ctrl.send(obj)
        end = self.clock.now() + timeout
        next_watch = self.clock.now() + WATCH_PERIOD
        stop_sent = False
        while True:
            try:
                reply, payloads = self.ctrl.recv(0.02)
                break
            except TimeoutError:
                pass
            except (ConnectionError, OSError) as e:
                self._check_cancel()
                raise ConnectionError(f"{self.role}: 연결 끊김 ({e})") from None
            now = self.clock.now()
            if now > end:
                raise TimeoutError(f"{self.role}: {obj.get('cmd')} 응답 없음({timeout:.0f} s)")
            if watch is not None and not stop_sent and now >= next_watch:
                next_watch = now + WATCH_PERIOD
                if watch():
                    self.send_stop()
                    stop_sent = True
        if not reply.get("ok"):
            self._check_cancel()
            raise RemoteError(f"{self.role}: {obj.get('cmd')} 실패: {reply.get('error')}")
        return reply, payloads

    def _profile(self):
        c, d = self.capture["color"], self.capture["depth"]
        return [c["w"], c["h"], c["fps"]], [d["w"], d["h"], d["fps"]]

    def hello(self):
        color, depth = self._profile()
        reply, _ = self._request({"cmd": "hello", "color": color, "depth": depth}, HELLO_TIMEOUT)
        self.calib = reply["calib"]
        if self.raw is not None:
            self.raw.calib(reply)
        return reply

    def move(self, text, watch=None):
        self._check_cancel()
        self._record({"type": "move", "text": text}, "out")
        reply, _ = self._request({"cmd": "move", "text": text, "timeout_s": MOVE_TIMEOUT},
                                 MOVE_TIMEOUT + MOVE_MARGIN, watch)
        return drive.MoveResult(True, bool(reply.get("stopped")), tuple(reply["odom"]))

    def send_stop(self):
        self._record({"type": "stop"}, "out")
        try:
            self.stream.send({"type": "stop"})
        except OSError:
            pass

    def snapshot(self, k, use, phi_target):
        color, depth = self._profile()
        req = {"cmd": "snapshot", "n_depth": self.capture["n_depth"], "n_skip": self.capture["n_skip"],
               "compress": bool(self.capture["compress"]), "color": color, "depth": depth}
        header, payloads = self._request(req, SNAPSHOT_TIMEOUT)
        snap = self.raw.snapshot(header, payloads, k, use, phi_target) if self.raw is not None else None
        return header, payloads, snap

    def abort(self):
        self.aborted = True
        self._record({"type": "abort"}, "out")
        try:
            self.stream.send({"type": "abort"})
        except OSError:
            pass
        self.ctrl.close()

    def close(self):
        for ch in (self.ctrl, self.stream):
            try:
                ch.close()
            except OSError:
                pass

    # ---- drive.RobotIO 나머지 ----
    def scan(self):
        if self._scan is None:
            return None, math.inf
        return self._scan, self.clock.now() - self._scan_t

    def odom(self):
        return self._odom

    def now(self):
        return self.clock.now()

    def sleep(self, s):
        self.clock.sleep(s, self.cancel)

    def say(self, text):
        print(f"[{self.role}] {text}", flush=True)

    def cancelled(self):
        return self.cancel.is_set()


def connect(cfg, role, raw_writer=None, clock=None, cancel=None, addr=None):
    """config의 로봇 하나에 연결한 RobotLink(hello 전). addr를 주면 config 주소 대신 쓴다."""
    from ..config import robot_addr

    host, port, sport = addr or robot_addr(cfg, role)
    return RobotLink(role, host, port, sport, raw_writer, clock or Clock(), cancel or threading.Event(),
                     capture=cfg["capture"])
