"""개발 PC용 가짜 로봇 에이전트(docs/specs/fake_agent.md).

robot_agent와 같은 메시지로 응답한다. 움직임은 sim2d, 스냅샷은 synth가 만든다.
두 로봇이 한 sim2d 세계를 같이 쓰고, 시간은 clock.Clock을 따른다.
"""
import argparse
import json
import math
import socket
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from .. import protocol
from ..robot import drive
from . import sim2d, synth

ROOT = Path(__file__).resolve().parents[2]
ODOM_PERIOD = 0.05
SCAN_PERIOD = 0.1
DEPTH_NOISE_MM = 2.0
ODOM_NOISE = {"scale": 0.01, "yaw_drift_deg_per_m": 1.0}
BOX = (0.18, 0.14, 0.16)   # 기본 시험 상자 w, d, h(m). 면 색은 synth.DEFAULT_FACE_COLORS(+x 빨강)


def default_mesh(cfg):
    """config `scene` 안에 드는 면마다 색이 다른 상자(빨강 면은 +x)."""
    return synth.box_mesh(*BOX)


def scale_calib(calib, color, depth):
    """`default_calib`을 요청 프로파일 크기로 바꾼다(시야각 유지, 정수 픽셀 중심)."""
    out = json.loads(json.dumps(calib))
    for key, (w, h, _) in (("color", color), ("depth", depth)):
        c = out[key]
        sx, sy = w / c["w"], h / c["h"]
        c.update(w=w, h=h, fx=c["fx"] * sx, fy=c["fy"] * sy,
                 ppx=(c["ppx"] + 0.5) * sx - 0.5, ppy=(c["ppy"] + 0.5) * sy - 0.5)
    return out


class _Bot:
    def __init__(self, role, robot, mount, phi0):
        self.role = role
        self.robot = robot
        self.mount = mount
        self.phi0 = phi0
        self.lock = threading.Lock()          # 문장 발행과 abort를 한 잠금 안에서
        self.cond = threading.Condition()
        self.statuses = deque()
        self.stream = None
        self.aborted = False
        self.profile = None
        self.counts = {}
        self.servers = []


class FakeFleet:
    def __init__(self, cfg, clock, mesh=None, board=None, roles=("L", "H"), starts=None,
                 obstacle=None, fail=None, rng_seed=0, serials=None, hosts=None, ports=None, snapshot_s=0.0):
        self.cfg = cfg
        self.clock = clock
        self.snapshot_s = float(snapshot_s)   # 스냅샷 하나에 걸리는 clock 초(실물 약 3.3 s)
        self.board = board if board is not None else json.loads((ROOT / cfg["board"]["layout"]).read_text())
        self.mesh = mesh if mesh is not None else default_mesh(cfg)
        self.roles = tuple(roles)
        self.fail = dict(fail) if fail else None
        self.serials = serials or {r: f"FAKE-{r}" for r in self.roles}
        self.ports = ports or {r: (0, 0) for r in self.roles}
        self.rng = np.random.default_rng(rng_seed)
        self.hello_log, self.move_log, self.stop_log = [], [], []
        self.snapshot_log, self.arrive_log, self.fail_log = [], [], []
        self._log_lock = threading.Lock()
        self._sim_lock = threading.Lock()
        self._render_lock = threading.Lock()
        self._running = False
        self._threads = []

        scene = cfg["scene"]
        self.R = scene["R"]
        self.n_views = scene["n_views"]
        starts = dict(starts or {"L": 0.0, "H": 180.0})
        self.world = sim2d.World(circles=[(0.0, 0.0, scene["r_obj"])])
        self.bots = {}
        robots = []
        for role in self.roles:
            s = starts[role]
            phi, yaw_off = (s, 0.0) if np.isscalar(s) else s
            mount = cfg["robots"][role]["mount"]
            yaw = math.radians(phi + 90.0 + yaw_off + self.rng.uniform(-5.0, 5.0))
            r = self.R + 0.08
            cx, cy = r * math.cos(math.radians(phi)), r * math.sin(math.radians(phi))
            ox, oy = drive._rotate(mount["x"], mount["y"], yaw)
            robot = sim2d.Robot(self.world, (cx - ox, cy - oy, drive.wrap_angle(yaw)), role,
                                linear_speed=0.10, odom_noise=ODOM_NOISE)
            robots.append(robot)
            self.bots[role] = _Bot(role, robot, mount, phi)
        self.sim = sim2d.Sim(self.world, robots, dt=0.05)
        if obstacle is not None:
            k, size = (obstacle, 0.10) if np.isscalar(obstacle) else obstacle
            self.world.add_box(*self.obstacle_box(k, size))
        self.scene = synth.Scene(self.board, self.mesh)
        self.calib = synth.default_calib()

    # ---- 시험용 조회 ----
    def obstacle_box(self, k, size=0.10, role="L"):
        bot = self.bots[role]
        step = 360.0 / self.n_views
        a = drive.view_pose(bot.phi0 + k * step, self.R, bot.mount)
        b = drive.view_pose(bot.phi0 + (k + 1) * step, self.R, bot.mount)
        yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, size, size, yaw)

    def world_boxes(self):
        with self._sim_lock:
            return list(self.world.boxes)

    def true_pose(self, role):
        with self._sim_lock:
            return self.bots[role].robot.true_pose()

    def view_target(self, role, k):
        bot = self.bots[role]
        return drive.view_pose(bot.phi0 + k * 360.0 / self.n_views, self.R, bot.mount)

    def _log(self, lst, item):
        with self._log_lock:
            lst.append(item)

    # ---- 실행 ----
    def start(self):
        self._running = True
        addrs = {}
        for role, bot in self.bots.items():
            host = "127.0.0.1"
            ctrl = self._listen(host, self.ports[role][0])
            strm = self._listen(host, self.ports[role][1])
            bot.servers = [ctrl, strm]
            addrs[role] = (host, ctrl.getsockname()[1], strm.getsockname()[1])
            self._spawn(self._serve_ctrl, bot, ctrl)
            self._spawn(self._serve_stream, bot, strm)
        self._spawn(self._sim_loop)
        return addrs

    def stop(self):
        self._running = False
        for bot in self.bots.values():
            for s in bot.servers:
                try:
                    s.close()
                except OSError:
                    pass
            if bot.stream is not None:
                bot.stream.close()
        for t in self._threads:
            t.join(timeout=2.0)

    def _spawn(self, fn, *args):
        t = threading.Thread(target=fn, args=args, daemon=True)
        t.start()
        self._threads.append(t)

    @staticmethod
    def _listen(host, port):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(1)
        s.settimeout(0.2)
        return s

    def _accept(self, srv):
        while self._running:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return None
            conn.settimeout(None)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return protocol.Channel(conn)
        return None

    # ---- sim2d 진행과 스트림 ----
    def _sim_loop(self):
        next_odom = next_scan = 0.0
        while self._running:
            target = self.clock.now()
            msgs = {role: [] for role in self.bots}
            with self._sim_lock:
                if target > self.sim.now:
                    self.sim.run_until(lambda: False, target - self.sim.now)
                now = self.sim.now
                for role, bot in self.bots.items():
                    for text in bot.robot.statuses():
                        msgs[role].append({"type": "status", "t": now, "text": text})
                        with bot.cond:
                            bot.statuses.append(text)
                            bot.cond.notify_all()
                if now >= next_odom:
                    for role, bot in self.bots.items():
                        x, y, yaw = bot.robot.odom()
                        msgs[role].append({"type": "odom", "t": now, "x": x, "y": y, "yaw": yaw})
                    next_odom = max(next_odom + ODOM_PERIOD, now - ODOM_PERIOD / 2)
                if now >= next_scan:
                    for role, bot in self.bots.items():
                        sc = bot.robot.scan()
                        msgs[role].append(protocol.encode_scan(now, sc["angle_min"], sc["angle_inc"],
                                                               sc["range_min"], sc["range_max"], sc["ranges"]))
                    next_scan = max(next_scan + SCAN_PERIOD, now - SCAN_PERIOD / 2)
            for role, lst in msgs.items():
                ch = self.bots[role].stream
                if ch is None:
                    continue
                try:
                    for m in lst:
                        ch.send(m)
                except OSError:
                    self.bots[role].stream = None
            time.sleep(0.004)

    def _serve_stream(self, bot, srv):
        while self._running:
            ch = self._accept(srv)
            if ch is None:
                return
            old, bot.stream = bot.stream, ch
            if old is not None:
                old.close()
            self._spawn(self._read_stream, bot, ch)

    def _read_stream(self, bot, ch):
        while self._running:
            try:
                msg, _ = ch.recv(0.5)
            except TimeoutError:
                continue
            except (ConnectionError, OSError, ValueError):
                return
            if msg.get("type") == "stop":
                self._log(self.stop_log, (bot.role, "stop", self.clock.now()))
                with self._sim_lock:
                    bot.robot.command("stop")
            elif msg.get("type") == "abort":
                self._log(self.stop_log, (bot.role, "abort", self.clock.now()))
                with bot.lock:
                    bot.aborted = True
                    with self._sim_lock:
                        bot.robot.command("stop")

    # ---- 제어 연결 ----
    def _serve_ctrl(self, bot, srv):
        while self._running:
            ch = self._accept(srv)
            if ch is None:
                return
            while self._running:
                try:
                    req, _ = ch.recv(0.5)
                except TimeoutError:
                    continue
                except (ConnectionError, OSError, ValueError):
                    break
                try:
                    reply, payloads = self._handle(bot, req)
                except Exception as e:  # 가짜 쪽 오류도 실패 응답으로 돌려준다
                    reply, payloads = {"ok": False, "error": f"fake: {e!r}"}, []
                try:
                    ch.send(reply, payloads)
                except OSError:
                    break
            ch.close()

    def _injected(self, bot, cmd):
        """`fail`에 맞는 요청이면 True. `after` 조건(다른 로봇이 움직이는 중·서 있음)을 최대 30 s 기다린다."""
        n = bot.counts.get(cmd, 0) + 1
        bot.counts[cmd] = n
        f = self.fail
        if not f or f.get("role") != bot.role or f.get("cmd") != cmd or f.get("nth") != n:
            return False
        busy = None
        cond = f.get("after")
        if cond:
            other = self.bots[cond.split("_")[0]].robot
            want_busy = cond.endswith("_moving")
            deadline = self.clock.now() + 30.0
            while self._running and self.clock.now() < deadline:
                with self._sim_lock:
                    busy = other.busy()
                if busy == want_busy:
                    break
                time.sleep(0.002)
        self._log(self.fail_log, (bot.role, self.clock.now(), busy))
        return True

    def _handle(self, bot, req):
        cmd = req.get("cmd")
        if cmd == "hello":
            self._log(self.hello_log, (bot.role, self.clock.now()))
            if self._injected(bot, cmd):
                return {"ok": False, "error": "injected"}, []
            with bot.lock:
                bot.aborted = False
                bot.profile = (list(req["color"]), list(req["depth"]))
            calib = scale_calib(self.calib, req["color"], req["depth"])
            return {"ok": True, "role": bot.role, "serial": self.serials[bot.role], "calib": calib}, []
        if cmd == "move":
            self._log(self.move_log, (bot.role, req["text"], self.clock.now()))
            if self._injected(bot, cmd):
                return {"ok": False, "error": "injected"}, []
            return self._move(bot, req["text"], float(req.get("timeout_s", 60))), []
        if cmd == "snapshot":
            if self._injected(bot, cmd):
                return {"ok": False, "error": "injected"}, []
            if req.get("ir"):
                return {"ok": False, "error": "ir not supported"}, []
            if bot.profile is None or (list(req["color"]), list(req["depth"])) != bot.profile:
                return {"ok": False, "error": "profile"}, []
            return self._snapshot(bot, req)
        return {"ok": False, "error": f"unknown cmd {cmd}"}, []

    def _move(self, bot, text, timeout_s):
        with bot.lock:
            if bot.aborted:
                return {"ok": False, "error": "aborted"}
            with bot.cond:
                bot.statuses.clear()
            with self._sim_lock:
                bot.robot.command(text)
        deadline = self.clock.now() + timeout_s
        result = None
        while result is None:
            with bot.cond:
                while not bot.statuses and self._running:
                    if self.clock.now() > deadline:
                        break
                    bot.cond.wait(0.02)
                texts = list(bot.statuses)
                bot.statuses.clear()
            if not self._running:
                return {"ok": False, "error": "shutdown"}
            for s in texts:
                if s.endswith("(모든 명령 완료)"):
                    result = {"ok": True, "stopped": False}
                elif s.startswith("stopped:"):
                    result = {"ok": True, "stopped": True}
                elif s.startswith(("error:", "blocked:")):
                    result = {"ok": False, "error": s}
                if result:
                    break
            if result is None and self.clock.now() > deadline:
                with self._sim_lock:
                    bot.robot.command("stop")
                result = {"ok": False, "error": "timeout"}
        with bot.lock:
            if bot.aborted:
                return {"ok": False, "error": "aborted"}
        with self._sim_lock:
            odom = list(bot.robot.odom())
            pose = bot.robot.true_pose()
        if result["ok"]:
            result["odom"] = odom
            k = self._arrived(bot, pose)
            if k is not None:
                self._log(self.arrive_log, (bot.role, k, self.clock.now()))
        return result

    def _arrived(self, bot, pose):
        cx, cy = drive.cam_xy(pose, bot.mount)
        if abs(math.hypot(cx, cy) - self.R) > 0.025:
            return None
        step = 360.0 / self.n_views
        rel = (math.degrees(math.atan2(cy, cx)) - bot.phi0) % 360.0
        k = int(round(rel / step)) % self.n_views
        err = abs((rel - k * step + 180.0) % 360.0 - 180.0)
        return k if err <= math.degrees(0.025 / self.R) else None

    def _snapshot(self, bot, req):
        (cw, ch_, _), (dw, dh, _) = req["color"], req["depth"]
        n = int(req.get("n_depth", 5))
        with self._render_lock, self.clock.hold():
            wall0 = time.monotonic()
            t = self.clock.now()
            with self._sim_lock:
                odom = list(bot.robot.odom())
                pose = bot.robot.true_pose()
            calib = scale_calib(self.calib, req["color"], req["depth"])
            T_cam = drive.camera_pose(pose, bot.mount)
            color = self.scene.render_color(T_cam, calib["color"])
            T_depth = synth.depth_pose(T_cam, calib)
            rng = np.random.default_rng(self.rng.integers(1 << 31))
            depth = b"".join(synth.to_z16(self.scene.render_depth(T_depth, calib["depth"], rng=rng,
                                                                  noise_mm=DEPTH_NOISE_MM))
                             for _ in range(n))
            hdr = {"ok": True, "t": t, "odom": odom,
                   "color": {"w": cw, "h": ch_, "fmt": "yuyv"},
                   "depth": {"w": dw, "h": dh, "fmt": "z16", "n": n}}
            hdr, payloads = protocol.pack_snapshot(hdr, {"color": synth.to_yuyv(color), "depth": depth},
                                                   bool(req.get("compress")))
        if self.snapshot_s > 0:
            self.clock.sleep(self.snapshot_s)
        self._log(self.snapshot_log, (bot.role, wall0, time.monotonic(), t))
        return hdr, payloads


def main(argv=None):
    import yaml
    from ..clock import Clock

    ap = argparse.ArgumentParser(description="가짜 로봇 에이전트 2대(localhost)")
    ap.add_argument("--config", default=str(ROOT / "config" / "scan.yaml"))
    ap.add_argument("--obstacle", type=int, default=None)
    ap.add_argument("--time-scale", type=float, default=1.0)
    args = ap.parse_args(argv)
    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    ports, used = {}, set()
    for role in ("L", "H"):
        p, s = cfg["robots"][role]["port"], cfg["robots"][role]["stream_port"]
        while p in used or s in used:
            p, s = p + 2, s + 2
        used |= {p, s}
        ports[role] = (p, s)
    fleet = FakeFleet(cfg, Clock(args.time_scale), obstacle=args.obstacle, ports=ports)
    for role, (host, p, s) in fleet.start().items():
        print(f"{role}: {host} {p} {s}", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        fleet.stop()


if __name__ == "__main__":
    main()
