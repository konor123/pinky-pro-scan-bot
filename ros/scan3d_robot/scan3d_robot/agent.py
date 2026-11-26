"""로봇 측 TCP 에이전트 본체(PLAN 4.2~4.4, docs/specs/robot_agent.md).

rclpy와 pyrealsense2를 import하지 않으며, 개발 PC 테스트에서는 가짜 camera·bus로 검증한다.
"""
from collections import deque
import math
import select
import socket
import threading
import time

from scan3d import protocol

_DONE_SUFFIX = "(모든 명령 완료)"
_ARC_PERIOD = 0.05        # cmd_vel 20 Hz(모터 드라이버의 명령 끊김 정지 0.5 s 안)
_ARC_SPEED = 0.10         # 문장에 speed가 없을 때 m/s
_ARC_MIN_SPEED = 0.03
_ARC_TOL = 0.003          # 남은 거리가 이보다 작으면 끝


def _listen_sock(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(4)
    sock.settimeout(0.2)
    return sock


def _peer_closed(sock: socket.socket) -> bool:
    try:
        if sock.fileno() < 0:
            return True
        readable, _, _ = select.select([sock], [], [], 0)
        if not readable:
            return False
        data = sock.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
        return data == b""
    except (BlockingIOError, TimeoutError):
        return False
    except (OSError, ValueError):
        return True


def _norm_profile(profile) -> list[int]:
    if not isinstance(profile, (list, tuple)) or len(profile) != 3:
        raise ValueError(f"invalid profile: {profile!r}")
    return [int(v) for v in profile]


class Agent:
    """로봇 TCP 제어(7010)·스트림(7011) 서버."""

    def __init__(
        self,
        role: str,
        camera,
        bus,
        port: int = protocol.CTRL_PORT,
        stream_port: int = protocol.STREAM_PORT,
        host: str = "0.0.0.0",
        clock=time.monotonic,
        sub_wait_s: float = 5.0,
        odom_hz: float = 20.0,
        odom_history_s: float = 2.0,
    ):
        self.role = str(role)
        self._camera = camera
        self._bus = bus
        self._clock = clock
        self._sub_wait_s = float(sub_wait_s)
        self._odom_hz = float(odom_hz)
        self._odom_history_s = float(odom_history_s)

        self._ctrl_sock = _listen_sock(host, port)
        self._stream_sock = _listen_sock(host, stream_port)
        self.port = self._ctrl_sock.getsockname()[1]
        self.stream_port = self._stream_sock.getsockname()[1]

        self._stop_event = threading.Event()
        self._cmd_lock = threading.Lock()
        self._aborted = False
        self._abort_event = threading.Event()
        self._arc_stop = threading.Event()

        self._odom_lock = threading.Lock()
        self._odom_hist: deque[tuple[float, float, float, float]] = deque()

        self._stream_lock = threading.Lock()
        self._stream_items: list[dict] = []
        self._stream_threads: list[threading.Thread] = []

        self._ctrl_lock = threading.Lock()
        self._active_ctrl_ch: protocol.Channel | None = None

        self._bus.on_odom(self._on_odom)
        self._bus.on_scan(self._on_scan)
        self._bus.on_status(self._on_status)

    def _on_odom(self, t: float, x: float, y: float, yaw: float) -> None:
        t, x, y, yaw = float(t), float(x), float(y), float(yaw)
        with self._odom_lock:
            self._odom_hist.append((t, x, y, yaw))
            while len(self._odom_hist) > 1 and (t - self._odom_hist[0][0]) > self._odom_history_s:
                self._odom_hist.popleft()

        min_dt = (1.0 / self._odom_hz) - 1e-9 if self._odom_hz > 0 else 0.0
        with self._stream_lock:
            targets = []
            for item in self._stream_items:
                last_t = item["last_odom_t"]
                if last_t is None or t < last_t or (t - last_t) >= min_dt:
                    item["last_odom_t"] = t
                    targets.append(item)
        if targets:
            self._broadcast({"type": "odom", "t": t, "x": x, "y": y, "yaw": yaw}, targets)

    def _on_scan(self, t: float, angle_min: float, angle_inc: float,
                 range_min: float, range_max: float, ranges) -> None:
        msg = protocol.encode_scan(t, angle_min, angle_inc, range_min, range_max, ranges)
        self._broadcast(msg)

    def _on_status(self, t: float, text: str) -> None:
        self._broadcast({"type": "status", "t": float(t), "text": str(text)})

    def _latest_odom(self) -> list[float]:
        _, x, y, yaw = self._bus.odom()
        return [float(x), float(y), float(yaw)]

    def _odom_at(self, t: float) -> list[float]:
        with self._odom_lock:
            if self._odom_hist:
                best = min(self._odom_hist, key=lambda item: abs(item[0] - t))
                return [best[1], best[2], best[3]]
        return self._latest_odom()

    def _broadcast(self, msg: dict, targets: list[dict] | None = None) -> None:
        with self._stream_lock:
            items = list(self._stream_items) if targets is None else list(targets)
        dead = []
        for item in items:
            try:
                item["ch"].send(msg)
            except OSError:
                dead.append(item)
        if dead:
            with self._stream_lock:
                for item in dead:
                    if item in self._stream_items:
                        self._stream_items.remove(item)
            for item in dead:
                item["ch"].close()

    def _stream_accept_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                conn, _ = self._stream_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            item = {"ch": protocol.Channel(conn), "last_odom_t": None}
            with self._stream_lock:
                self._stream_items.append(item)
            t = threading.Thread(target=self._stream_reader, args=(item,), daemon=True)
            self._stream_threads.append(t)
            t.start()

    def _stream_reader(self, item: dict) -> None:
        ch = item["ch"]
        try:
            while not self._stop_event.is_set():
                try:
                    msg, _ = ch.recv(0.2)
                except TimeoutError:
                    continue
                except (ConnectionError, OSError, ValueError):
                    break
                if not isinstance(msg, dict):
                    continue
                mtype = msg.get("type")
                if mtype == "stop":
                    with self._cmd_lock:
                        self._arc_stop.set()
                        self._bus.publish_command("stop")
                elif mtype == "abort":
                    # abort 상태 설정과 stop 발행을 같은 잠금 안에서 처리해 늦은 move를 막는다.
                    with self._cmd_lock:
                        self._aborted = True
                        self._abort_event.set()
                        self._arc_stop.set()
                        self._bus.publish_command("stop")
        finally:
            with self._stream_lock:
                if item in self._stream_items:
                    self._stream_items.remove(item)
            ch.close()

    def _handle_move(self, req: dict, conn: socket.socket) -> dict | None:
        text = req.get("text")
        if not isinstance(text, str) or not text.strip():
            return {"ok": False, "error": "empty move text"}
        timeout_s = float(req.get("timeout_s", 60.0))

        try:
            arc = protocol.parse_arc(text)
        except ValueError as exc:
            return {"ok": False, "error": f"error: {exc}"}
        if arc is not None:
            return self._handle_arc(arc, timeout_s)

        with self._cmd_lock:
            if self._aborted:
                return {"ok": False, "error": "aborted"}

        while self._bus.next_status(0) is not None:
            pass

        sub_deadline = self._clock() + self._sub_wait_s
        while self._bus.command_subscribers() <= 0:
            if self._stop_event.is_set() or _peer_closed(conn):
                return None
            with self._cmd_lock:
                if self._aborted:
                    return {"ok": False, "error": "aborted"}
            rem = sub_deadline - self._clock()
            if rem <= 0:
                return {"ok": False, "error": "no subscriber"}
            self._abort_event.wait(min(0.02, rem))

        with self._cmd_lock:
            if self._aborted:
                return {"ok": False, "error": "aborted"}
            while self._bus.next_status(0) is not None:
                pass
            self._bus.publish_command(text)

        deadline = self._clock() + timeout_s
        while not self._stop_event.is_set():
            with self._cmd_lock:
                if self._aborted:
                    return {"ok": False, "error": "aborted"}
            if _peer_closed(conn):
                return None
            rem = deadline - self._clock()
            if rem <= 0:
                with self._cmd_lock:
                    if self._aborted:
                        return {"ok": False, "error": "aborted"}
                    self._bus.publish_command("stop")
                return {"ok": False, "error": "timeout"}

            status = self._bus.next_status(min(0.05, rem))
            with self._cmd_lock:
                if self._aborted:
                    return {"ok": False, "error": "aborted"}
            if status is None:
                continue
            if status.startswith("done:") and _DONE_SUFFIX in status:
                return {"ok": True, "stopped": False, "odom": self._latest_odom()}
            if status.startswith("stopped:"):
                return {"ok": True, "stopped": True, "odom": self._latest_odom()}
            if status.startswith("blocked:") or status.startswith("error:"):
                return {"ok": False, "error": status}
        return None

    def _handle_arc(self, arc, timeout_s: float) -> dict | None:
        """원호 문장을 직접 실행한다. `command_driver`는 직진·제자리 회전만 하므로 `/pinky_command`로 보내지 않는다.

        `cmd_vel`에 직진 속도와 회전 속도(직진 속도 / 반지름)를 함께 보내고, odom 이동 거리가 반지름 × 각도에 이르면 선다.
        """
        speed, radius, angle = arc
        speed = _ARC_SPEED if speed is None else speed
        length = radius * math.radians(abs(angle))
        sign = 1.0 if angle > 0 else -1.0
        with self._cmd_lock:
            if self._aborted:
                return {"ok": False, "error": "aborted"}
            self._arc_stop.clear()
        _, x, y, _ = self._bus.odom()
        travelled = 0.0
        deadline = self._clock() + timeout_s
        try:
            while not self._stop_event.is_set():
                with self._cmd_lock:
                    if self._aborted:
                        return {"ok": False, "error": "aborted"}
                    if self._arc_stop.is_set():
                        return {"ok": True, "stopped": True, "odom": self._latest_odom()}
                    _, nx, ny, _ = self._bus.odom()
                    travelled += math.hypot(nx - x, ny - y)
                    x, y = nx, ny
                    remaining = length - travelled
                    if remaining <= _ARC_TOL:
                        return {"ok": True, "stopped": False, "odom": self._latest_odom()}
                    if self._clock() >= deadline:
                        return {"ok": False, "error": "timeout"}
                    v = max(_ARC_MIN_SPEED, min(speed, 1.5 * remaining))
                    self._bus.publish_twist(v, sign * v / radius)
                self._abort_event.wait(_ARC_PERIOD)
        finally:
            self._bus.publish_twist(0.0, 0.0)
        return None

    def _handle_ctrl_conn(self, conn: socket.socket) -> None:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        ch = protocol.Channel(conn)
        with self._ctrl_lock:
            self._active_ctrl_ch = ch
        hello_color = None
        hello_depth = None

        try:
            while not self._stop_event.is_set():
                try:
                    req, _ = ch.recv(0.2)
                except TimeoutError:
                    continue
                except (ConnectionError, OSError, ValueError):
                    break

                if not isinstance(req, dict):
                    ch.send({"ok": False, "error": "invalid request"})
                    continue

                cmd = req.get("cmd")
                try:
                    if cmd == "hello":
                        color_p = _norm_profile(req["color"])
                        depth_p = _norm_profile(req["depth"])
                        calib = self._camera.calibration(color_p, depth_p)
                        serial = self._camera.serial()
                        hello_color, hello_depth = color_p, depth_p
                        with self._cmd_lock:
                            self._aborted = False
                            self._abort_event.clear()
                        ch.send({
                            "ok": True,
                            "role": self.role,
                            "serial": serial,
                            "calib": calib,
                            "arc": True,
                        })
                    elif cmd == "move":
                        resp = self._handle_move(req, conn)
                        if resp is None:
                            break
                        ch.send(resp)
                    elif cmd == "snapshot":
                        compress = bool(req.get("compress", False))
                        if req.get("ir"):
                            d_req = _norm_profile(req.get("depth"))
                            if hello_depth is None or d_req != hello_depth:
                                ch.send({"ok": False, "error": "profile"})
                                continue
                            t, ir_hdr, ir_bytes = self._camera.snapshot_ir(
                                d_req, int(req.get("n_skip", 0))
                            )
                            hdr = {
                                "ok": True,
                                "t": float(t),
                                "odom": self._odom_at(t),
                                "ir": ir_hdr,
                            }
                            hdr, pls = protocol.pack_snapshot(hdr, {"ir": ir_bytes}, compress)
                            ch.send(hdr, pls)
                        else:
                            c_req = _norm_profile(req.get("color"))
                            d_req = _norm_profile(req.get("depth"))
                            if (hello_color is None or hello_depth is None
                                    or c_req != hello_color or d_req != hello_depth):
                                ch.send({"ok": False, "error": "profile"})
                                continue
                            n_depth = int(req.get("n_depth", 5))
                            n_skip = int(req.get("n_skip", 0))
                            t, c_hdr, c_bytes, d_hdr, d_bytes = self._camera.snapshot(
                                c_req, d_req, n_depth, n_skip
                            )
                            hdr = {
                                "ok": True,
                                "t": float(t),
                                "odom": self._odom_at(t),
                                "color": c_hdr,
                                "depth": d_hdr,
                            }
                            hdr, pls = protocol.pack_snapshot(
                                hdr, {"color": c_bytes, "depth": d_bytes}, compress
                            )
                            ch.send(hdr, pls)
                    else:
                        ch.send({"ok": False, "error": f"unknown cmd: {cmd}"})
                except OSError:
                    break
                except Exception as exc:
                    try:
                        ch.send({"ok": False, "error": str(exc)})
                    except OSError:
                        break
        finally:
            with self._ctrl_lock:
                if self._active_ctrl_ch is ch:
                    self._active_ctrl_ch = None
            ch.close()

    def serve_forever(self) -> None:
        stream_thread = threading.Thread(target=self._stream_accept_loop, daemon=True)
        stream_thread.start()
        try:
            while not self._stop_event.is_set():
                try:
                    conn, _ = self._ctrl_sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self._handle_ctrl_conn(conn)
        finally:
            stream_thread.join(timeout=1.0)

    def shutdown(self) -> None:
        self._stop_event.set()
        self._abort_event.set()
        for sock in (self._ctrl_sock, self._stream_sock):
            try:
                sock.close()
            except OSError:
                pass
        with self._ctrl_lock:
            ch = self._active_ctrl_ch
        if ch is not None:
            ch.close()
        with self._stream_lock:
            items = list(self._stream_items)
            self._stream_items.clear()
        for item in items:
            item["ch"].close()
        for t in self._stream_threads:
            t.join(timeout=1.0)
