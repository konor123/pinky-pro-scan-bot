"""스캔 PC 주 프로그램(docs/specs/station.md, PLAN 3.3·4.4).

두 로봇(또는 한 대)과 연결해 출발, 24시점 맞춤 진행, 스냅샷, 원본 저장, 데이터셋, 재구성, 메시 표시까지 한다.
"""
import argparse
import contextlib
import json
import math
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

from . import align, dataset, drive, protocol, recon, tagpose
from .vendor.pinky_commands import wrap_angle

ROOT = Path(__file__).resolve().parents[1]
ORDER = ("L", "H")
HELLO_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0
MOVE_MARGIN = 5.0
MOVE_TIMEOUT = 60.0
WATCH_PERIOD = 0.1


class Clock:
    """모든 기다림과 한도가 쓰는 시계. time_scale배 빠르게 간다. `hold()` 동안은 멈춘다."""

    def __init__(self, time_scale=1.0):
        self.time_scale = float(time_scale)
        self._t0 = time.monotonic()
        self._lock = threading.Lock()
        self._held = 0
        self._held_at = 0.0
        self._paused = 0.0

    def _raw(self):
        return (time.monotonic() - self._t0) * self.time_scale - self._paused

    def now(self):
        with self._lock:
            return self._held_at if self._held else self._raw()

    def sleep(self, s, cancel=None):
        end = self.now() + s
        while True:
            left = end - self.now()
            if left <= 0 or (cancel is not None and cancel.is_set()):
                return
            time.sleep(min(left / self.time_scale, 0.02))

    @contextlib.contextmanager
    def hold(self):
        with self._lock:
            if self._held == 0:
                self._held_at = self._raw()
            self._held += 1
        try:
            yield
        finally:
            with self._lock:
                self._held -= 1
                if self._held == 0:
                    self._paused += self._raw() - self._held_at


class StartError(RuntimeError):
    """움직이기 전에 멈추는 출발 조건 실패."""


class RemoteError(RuntimeError):
    """로봇의 실패 응답(`ok:false`)."""


class RobotLink:
    """drive.RobotIO 구현. 제어 연결(요청·응답)과 스트림 연결(odom·scan·status, stop·abort)."""

    def __init__(self, role, host, port, stream_port, raw_writer, clock, cancel, capture=None):
        self.role = role
        self.raw = raw_writer
        self.clock = clock
        self.cancel = cancel
        self.capture = capture
        self.calib = None
        self.aborted = False
        self._odom = None
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
            elif typ == "scan":
                self._scan = protocol.decode_scan(msg)
                self._scan_t = self.clock.now()
            self._record(msg, "in")

    def wait_streams(self, timeout=5.0):
        end = self.clock.now() + timeout
        while self._odom is None or self._scan is None:
            if self.clock.now() > end:
                raise RemoteError(f"{self.role}: odom·LiDAR 스트림이 오지 않음")
            self.clock.sleep(0.05)

    # ---- 요청 ----
    def _request(self, obj, timeout, watch=None):
        if self.aborted or self.cancelled():
            raise drive.Cancelled(f"{self.role}: 취소됨")
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
                if self.aborted or self.cancelled():
                    raise drive.Cancelled(f"{self.role}: 취소됨") from None
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
            if self.aborted or self.cancelled():
                raise drive.Cancelled(f"{self.role}: 취소됨")
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
        msg = {"cmd": "move", "text": text, "timeout_s": MOVE_TIMEOUT}
        if self.aborted or self.cancelled():
            raise drive.Cancelled(f"{self.role}: 취소됨")
        self._record({"type": "move", "text": text}, "out")
        reply, _ = self._request(msg, MOVE_TIMEOUT + MOVE_MARGIN, watch)
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


def resolve_host(host):
    """`~/.ssh/config` 별칭을 `ssh -G`의 hostname으로 바꾼다. ssh가 없거나 실패하면 그대로."""
    try:
        out = subprocess.run(["ssh", "-G", host], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return host
    for line in out.splitlines():
        key, _, value = line.partition(" ")
        if key == "hostname" and value:
            return value.strip()
    return host


def load_config(path=None):
    path = Path(path or ROOT / "config" / "scan.yaml")
    return path, yaml.safe_load(path.read_text(encoding="utf-8"))


def load_board(cfg):
    board_path = ROOT / cfg["board"]["layout"]
    board_json = json.loads(board_path.read_text(encoding="utf-8"))
    return board_path, tagpose.load_board(board_json, cfg["board"]["tag_mm"])


def connect_robot(cfg, role, raw_writer=None, clock=None, cancel=None):
    """현장 도구용: config의 로봇 하나에 연결한 RobotLink(hello 전)."""
    rc = cfg["robots"][role]
    return RobotLink(role, resolve_host(rc["host"]), rc["port"], rc["stream_port"], raw_writer,
                     clock or Clock(), cancel or threading.Event(), capture=cfg["capture"])


def _parallel(jobs, cancel):
    """{role: fn}을 로봇마다 스레드로 함께 실행한다. 예외는 끝나는 대로 바로 올린다."""
    results, done = {}, queue.Queue()

    def run(role, fn):
        try:
            done.put((role, fn(), None))
        except BaseException as e:  # noqa: BLE001  스레드 예외를 주 스레드로 넘긴다
            done.put((role, None, e))

    threads = {role: threading.Thread(target=run, args=(role, fn), daemon=True) for role, fn in jobs.items()}
    for t in threads.values():
        t.start()
    while len(results) < len(jobs):
        role, value, err = done.get()
        if err is not None:
            raise err
        results[role] = value
    return results


class Scan:
    """한 번의 스캔 진행(PLAN 4.4 진행 순서)."""

    def __init__(self, cfg, links, writer, board, cancel):
        self.cfg = cfg
        self.links = links
        self.writer = writer
        self.board = board
        self.cancel = cancel
        self.roles = [r for r in ORDER if r in links]
        scene = cfg["scene"]
        self.r_eff = scene["r_obj"] + scene["place_err"]
        self.mounts = {r: cfg["robots"][r]["mount"] for r in self.roles}
        self.drivers = {r: drive.Driver(links[r], cfg, self.mounts[r], on_event=self._driver_event(r))
                        for r in self.roles}
        self.cur = {r: -1 for r in self.roles}
        self.skipped = {r: [] for r in self.roles}
        self.stops = {r: 0 for r in self.roles}
        self.corrections = {r: 0 for r in self.roles}
        self.calib = {}

    def event(self, ev):
        self.writer.event(ev)

    def _driver_event(self, role):
        def on_event(ev):
            self.event({"pc_t": self.links[role].now(), "role": role, "type": "drive", **ev})
        return on_event

    def _ev(self, role, typ, **kw):
        self.event({"pc_t": self.links[self.roles[0]].now(), "role": role, "type": typ, **kw})

    # ---- 단계 ----
    def hello(self):
        serials = {}
        for r in self.roles:
            reply = self.links[r].hello()
            if reply.get("role") != r:
                raise StartError(f"{r} 자리의 로봇이 role {reply.get('role')}로 응답함(config robots 확인)")
            serials[r] = reply.get("serial")
            self.calib[r] = reply["calib"]
        if len(set(serials.values())) != len(serials):
            raise StartError(f"두 로봇의 카메라 일련번호가 같음: {serials}")
        for r in self.roles:
            self.links[r].wait_streams()

    def _estimate(self, role, header, payloads):
        frame = align.prepare(header, payloads, self.calib[role])
        return tagpose.estimate(frame.color, frame.K, self.board)

    def start_poses(self):
        phis = {}
        for r in self.roles:
            header, payloads, snap = self.links[r].snapshot(None, "start", None)
            pose = self._estimate(r, header, payloads)
            if pose is None:
                self._ev(r, "start", snap=snap, pose=False, phi=None, d=None, d_source=None)
                raise StartError(f"{r}: 표식이 보이지 않음. 카메라가 판을 보게 놓는다")
            T = pose.T_cam_board
            self.drivers[r].set_fix(drive.base_from_cam(T, self.mounts[r]), header["odom"])
            phis[r] = tagpose.phi_deg(T)
            self._ev(r, "start", snap=snap, pose=True, phi=phis[r], d=tagpose.distance_d(T, self.r_eff),
                     d_source="tag")
        if len(self.roles) == 2:
            gap = math.degrees(wrap_angle(math.radians(phis["H"] - phis["L"])))
            tol = self.cfg["drive"]["pair_gap_tol_deg"]
            if abs(abs(gap) - 180.0) > tol:
                need = math.degrees(wrap_angle(math.radians(phis["L"] + 180.0 - phis["H"])))
                way = "반시계" if need > 0 else "시계"
                raise StartError(f"두 로봇 각도 차 {abs(gap):.0f}°(180 ± {tol}° 밖): "
                                 f"H를 판 둘레로 {way} 방향 {abs(need):.0f}° 옮긴다")

    def _leg(self, role, k, leg):
        self.stops[role] += leg.stops
        self._ev(role, "leg", k=k, view=leg.view, skipped=list(leg.skipped), stops=leg.stops,
                 waited_s=leg.waited_s)
        for j in leg.skipped:
            self.skipped[role].append(j)
            self._ev(role, "skip", k=j, reason="bypass")
        self.cur[role] = leg.view

    def enter(self):
        legs = _parallel({r: self.drivers[r].enter for r in self.roles}, self.cancel)
        for r, leg in legs.items():
            self._leg(r, 0, leg)

    def _d_now(self, role, header, payloads):
        pose = self._estimate(role, header, payloads)
        drv = self.drivers[role]
        if pose is not None:
            T = pose.T_cam_board
            drv.set_fix(drive.base_from_cam(T, self.mounts[role]), header["odom"])
            return tagpose.distance_d(T, self.r_eff), "tag", True
        cx, cy = drive.cam_xy(drv.pose(), self.mounts[role])
        return math.hypot(cx, cy) - self.r_eff, "odom", False

    def capture(self, role, k):
        link, drv, dcfg = self.links[role], self.drivers[role], self.cfg["drive"]
        phi = drv.phi_deg(k)
        header, payloads, snap = link.snapshot(k, "view", phi)
        d, src, has_pose = self._d_now(role, header, payloads)
        self._ev(role, "view", k=k, use="view", snap=snap, pose=has_pose, d=d, d_source=src)
        if dcfg["d_min"] <= d <= dcfg["d_max"]:
            return
        leg = drv.correct(k)
        self.corrections[role] += 1
        self.stops[role] += leg.stops
        link.sleep(self.cfg["capture"]["settle_s"])
        header, payloads, snap = link.snapshot(k, "retake", phi)
        d2, src2, has_pose = self._d_now(role, header, payloads)
        self._ev(role, "view", k=k, use="retake", snap=snap, pose=has_pose, d=d2, d_source=src2)
        self._ev(role, "correct", k=k, d_before=d, src_before=src, d_after=d2, src_after=src2)

    def views(self):
        n = self.cfg["scene"]["n_views"]
        for k in range(n):
            movers = {r: (lambda r=r: self.drivers[r].go_to_view(k)) for r in self.roles if self.cur[r] < k}
            if movers:
                for r, leg in _parallel(movers, self.cancel).items():
                    self._leg(r, k, leg)
            self.links[self.roles[0]].sleep(self.cfg["capture"]["settle_s"])
            if self.cancel.is_set():
                raise drive.Cancelled("취소됨")
            for r in self.roles:
                if self.cur[r] == k:
                    self.capture(r, k)

    def report(self, aborted=None, elapsed_s=None):
        return {"skipped": dict(self.skipped), "stops": dict(self.stops),
                "corrections": dict(self.corrections), "aborted": aborted, "elapsed_s": elapsed_s}


def run(cfg, links, writer, board, cancel=None, t0=None):
    """links로 스캔 한 번. 성공하면 (scan_dir, recon 결과, 보고서)."""
    cancel = cancel or threading.Event()
    t0 = time.monotonic() if t0 is None else t0
    scan = Scan(cfg, links, writer, board, cancel)
    try:
        scan.hello()
        scan.start_poses()
        scan.enter()
        scan.views()
    except BaseException as e:
        cancel.set()
        for link in links.values():
            link.abort()
        reason = f"{type(e).__name__}: {e}"
        scan._ev(None, "abort", reason=reason)
        report = scan.report(aborted=reason, elapsed_s=time.monotonic() - t0)
        writer.report(report)
        raise
    scan_dir = writer.scan_dir
    roles = scan.roles
    for r in roles:
        dataset.build(scan_dir, r)
    result = recon.reconstruct([Path(scan_dir) / r for r in roles], cfg)
    missing = [r for r in roles if not result.used.get(r)]
    report = scan.report(elapsed_s=time.monotonic() - t0)
    if missing:
        report["aborted"] = f"융합한 시점 없음: {missing}"
        writer.report(report)
        raise RuntimeError(report["aborted"])
    report["dims_mm"] = [round(float(x), 1) for x in result.dims_mm]
    report["elapsed_s"] = round(time.monotonic() - t0, 1)
    recon.write_outputs(scan_dir, result, report)
    writer.report(report)
    return scan_dir, result, report


def show_mesh(mesh):
    import open3d as o3d
    o3d.visualization.draw_geometries([mesh], window_name="scan3d")


def execute(cfg, cfg_path, addrs, clock, roles=ORDER, no_window=True, data_dir=None):
    """연결부터 끝까지. 반환 (종료 코드, scan_dir)."""
    t0 = time.monotonic()
    board_path, board = load_board(cfg)
    root = Path(data_dir or ROOT / cfg["station"]["data_dir"])
    writer = dataset.ScanWriter(root, cfg_path, board_path)
    cancel = threading.Event()
    links = {}
    try:
        for r in roles:
            host, port, sport = addrs[r]
            links[r] = RobotLink(r, host, port, sport, writer.role(r), clock, cancel, capture=cfg["capture"])
        scan_dir, result, report = run(cfg, links, writer, board, cancel, t0)
    except StartError as e:
        print(f"시작하지 않음: {e}", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    except KeyboardInterrupt:
        print("중단: Ctrl-C", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    except Exception as e:  # noqa: BLE001  원인을 화면과 report.json에 남기고 끝낸다
        print(f"중단: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    finally:
        for link in links.values():
            link.close()
    dims = result.dims_mm
    print(f"치수(mm): {dims[0]:.0f} × {dims[1]:.0f} × {dims[2]:.0f}, 걸린 시간 {report['elapsed_s']:.0f} s",
          flush=True)
    if not no_window and cfg["station"].get("show_mesh", True):
        show_mesh(result.mesh)
    return 0, scan_dir


def hello_only(cfg, roles):
    code = 0
    for r in roles:
        try:
            link = connect_robot(cfg, r)
        except OSError as e:
            print(f"{r}: 연결 실패 {e}", flush=True)
            code = 1
            continue
        try:
            reply = link.hello()
            c, d = reply["calib"]["color"], reply["calib"]["depth"]
            print(f"{r}: role {reply['role']}, serial {reply['serial']}, "
                  f"color {c['w']}x{c['h']} fx {c['fx']:.1f} fy {c['fy']:.1f} {c['model']}, "
                  f"depth {d['w']}x{d['h']} fx {d['fx']:.1f}, depth_scale {reply['calib']['depth_scale']}",
                  flush=True)
            if reply["role"] != r:
                print(f"{r}: role이 config와 다름", flush=True)
                code = 1
        except Exception as e:  # noqa: BLE001
            print(f"{r}: hello 실패 {e}", flush=True)
            code = 1
        finally:
            link.close()
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(description="두 로봇 3D 스캔")
    ap.add_argument("--config", default=str(ROOT / "config" / "scan.yaml"))
    ap.add_argument("--only", choices=ORDER)
    ap.add_argument("--fake", action="store_true", help="가짜 에이전트 2대로 실행")
    ap.add_argument("--obstacle", type=int, default=None, help="--fake에서 L 시점 k·k+1 사이에 상자")
    ap.add_argument("--time-scale", type=float, default=None)
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--hello", action="store_true", help="연결과 hello만 하고 role·일련번호·보정값을 출력")
    args = ap.parse_args(argv)
    cfg_path, cfg = load_config(args.config)
    roles = (args.only,) if args.only else ORDER
    if args.hello:
        return hello_only(cfg, roles)
    fleet = None
    if args.fake:
        from .fake_agent import FakeFleet
        clock = Clock(args.time_scale or 5.0)
        fleet = FakeFleet(cfg, clock, roles=roles, obstacle=args.obstacle)
        addrs = fleet.start()
    else:
        clock = Clock(args.time_scale or 1.0)
        addrs = {}
        for r in roles:
            rc = cfg["robots"][r]
            addrs[r] = (resolve_host(rc["host"]), rc["port"], rc["stream_port"])
    try:
        code, scan_dir = execute(cfg, cfg_path, addrs, clock, roles, no_window=args.no_window)
    finally:
        if fleet is not None:
            fleet.stop()
    print(f"데이터: {scan_dir}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
