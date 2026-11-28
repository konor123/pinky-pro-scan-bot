"""[로봇 1대] 한 로봇의 스캔 단계(PLAN 4.4 진행 순서 중 로봇마다 하는 일).

  hello → start_pose(표식으로 출발 자세) → enter(원 진입) → go_to_view(k) → capture(k)(거리 벗어나면 보정·재촬영)

두 로봇을 함께 돌리는 순서와 조건은 pair.coordinator가 정한다. 이 모듈은 다른 로봇을 모른다.
"""
import contextlib
import math
from dataclasses import dataclass, field

from ..protocol import parse_arc
from . import align, drive, mountfit, tagpose


class StartError(RuntimeError):
    """움직이기 전에 멈추는 출발 조건 실패."""


@dataclass
class UnitStats:
    skipped: list = field(default_factory=list)   # 우회로 건너뛴 시점
    stops: int = 0                                # 장애물 정지 횟수
    corrections: int = 0                          # 거리 보정 횟수


class RobotScan:
    """로봇 하나의 스캔. link는 RobotLink(또는 같은 인터페이스), emit(type, **kw)는 이벤트 기록."""

    def __init__(self, role, link, cfg, board, emit, on_capture=None):
        self.role = role
        self.link = link
        self.cfg = cfg
        self.board = board
        self.emit = emit
        self.on_capture = on_capture
        # 처음 값. x·y·yaw_deg는 원호를 달릴 때마다 표식으로 다시 맞춘다(drive.mount_fit, mountfit)
        self.mount = dict(cfg["robots"][role]["mount"])
        self._fit = None
        self._fit_at = None    # 마지막으로 자세를 잡은 (표식 자세, odom yaw)
        scene = cfg["scene"]
        self.r_eff = scene["r_obj"] + scene["place_err"]
        self.driver = drive.Driver(link, cfg, self.mount,
                                   on_event=lambda ev: self.emit("drive", **ev))
        self.calib = None
        self.serial = None
        self.cur = -1          # 마지막으로 도착한 시점(아직 없으면 −1)
        self.phi0 = None       # 출발 때 판 좌표 각(도)
        self.activity = "idle"  # idle | moving | capturing(화면 표시용)
        self.stats = UnitStats()

    # ---- 1. 연결 ----
    def hello(self):
        reply = self.link.hello()
        if reply.get("role") != self.role:
            raise StartError(f"{self.role} 자리의 로봇이 role {reply.get('role')}로 응답함(config robots 확인)")
        self.serial = reply.get("serial")
        self.calib = reply["calib"]
        if self.driver.arc and not reply.get("arc"):
            # 원호 문장을 모르는 에이전트(배포 전)에는 보내지 않는다.
            self.driver.arc = False
            self.link.say("로봇 에이전트가 원호 문장을 모릅니다. 시점 이동을 회전 + 직진으로 합니다")
        if self.cfg["drive"].get("mount_fit", True):
            known = mountfit.recall(self.serial)        # 이 서버에서 앞선 스캔이 맞춘 값
            if known:
                self._set_mount(known)
            self._fit = mountfit.MountFit(self.mount)
        self.emit("hello", arc=self.driver.arc)
        return reply

    def wait_streams(self):
        self.link.wait_streams()

    # ---- 2. 출발 자세 ----
    def _prepare_and_estimate(self, header, payloads):
        frame = align.prepare(header, payloads, self.calib)
        return frame, tagpose.estimate(frame.color, frame.K, self.board)

    def _estimate(self, header, payloads):
        return self._prepare_and_estimate(header, payloads)[1]

    def _notify_capture(self, use, k, frame, pose, header):
        if self.on_capture is None:
            return
        try:
            self.on_capture(self.role, use, k, frame, pose, header)
        except Exception:  # noqa: BLE001  화면 콜백 오류가 스캔을 멈추지 않게
            pass

    def _set_mount(self, values):
        self.mount = dict(self.mount, **values)
        self.driver.mount = self.mount

    def _fit_mount(self, T, odom):
        """지난 표식 자세와 이번 사이에 원호 문장 하나만 끝까지 달렸으면 그 회전 중심으로 장착값을 다시 맞춘다."""
        sent = self.driver.take_sent()
        before, self._fit_at = self._fit_at, (T, odom[2])
        if self._fit is None or before is None or len(sent) != 1 or sent[0][1]:
            return
        arc = parse_arc(sent[0][0])
        seen = mountfit.observe(before[0], T, before[1], odom[2], arc[1], arc[2]) if arc is not None else None
        if seen is not None and self._fit.add(*seen):
            m = self._fit.mount()
            self._set_mount({k: m[k] for k in ("x", "y", "yaw_deg")})
            mountfit.remember(self.serial, m)
            self.emit("mount", x=m["x"], y=m["y"], yaw_deg=m["yaw_deg"], n=self._fit.n)

    def _fix(self, T, header):
        self._fit_mount(T, header["odom"])
        self.driver.set_fix(drive.base_from_cam(T, self.mount), header["odom"])

    def start_pose(self):
        """표식으로 출발 자세를 잡고 판 좌표 각 phi(도)를 돌려준다."""
        self.activity = "capturing"
        try:
            header, payloads, snap = self.link.snapshot(None, "start", None)
        finally:
            self.activity = "idle"
        frame, pose = self._prepare_and_estimate(header, payloads)
        self._notify_capture("start", None, frame, pose, header)
        if pose is None:
            self.emit("start", snap=snap, pose=False, phi=None, d=None, d_source=None)
            raise StartError(f"{self.role}: 표식이 보이지 않음. 카메라가 판을 보게 놓는다")
        T = pose.T_cam_board
        self._fix(T, header)
        self.phi0 = tagpose.phi_deg(T)
        self.emit("start", snap=snap, pose=True, phi=self.phi0, d=tagpose.distance_d(T, self.r_eff),
                  d_source="tag")
        return self.phi0

    # ---- 3. 이동 ----
    def _record_leg(self, k, leg):
        self.stats.stops += leg.stops
        self.emit("leg", k=k, view=leg.view, skipped=list(leg.skipped), stops=leg.stops, waited_s=leg.waited_s)
        for j in leg.skipped:
            self.stats.skipped.append(j)
            self.emit("skip", k=j, reason="bypass")
        self.cur = leg.view

    def enter(self):
        self.activity = "moving"
        try:
            self._record_leg(0, self.driver.enter())
        finally:
            self.activity = "idle"

    def needs_move(self, k):
        return self.cur < k

    def go_to_view(self, k):
        self.activity = "moving"
        try:
            self._record_leg(k, self.driver.go_to_view(k))
        finally:
            self.activity = "idle"

    def at_view(self, k):
        return self.cur == k

    # ---- 4. 촬영 ----
    def _distance(self, header, payloads, use="view", k=None):
        """→ (d, 출처 'tag'|'odom', 표식 자세 있음). 표식이 보이면 자세도 고친다."""
        frame, pose = self._prepare_and_estimate(header, payloads)
        self._notify_capture(use, k, frame, pose, header)
        if pose is not None:
            T = pose.T_cam_board
            self._fix(T, header)
            return tagpose.distance_d(T, self.r_eff), "tag", True
        cx, cy = drive.cam_xy(self.driver.pose(), self.mount)
        return math.hypot(cx, cy) - self.r_eff, "odom", False

    def capture(self, k, turn=contextlib.nullcontext):
        """시점 k 촬영. 거리가 [d_min, d_max] 밖이면 한 번 보정 이동 뒤 다시 찍는다.

        turn()은 스냅샷 동안 잡는 촬영 차례(두 대일 때 한 번에 한 로봇만 카메라를 켠다). 보정 이동 중에는 놓는다.
        """
        dcfg = self.cfg["drive"]
        phi = self.driver.phi_deg(k)
        with turn():
            self.activity = "capturing"
            try:
                header, payloads, snap = self.link.snapshot(k, "view", phi)
            finally:
                self.activity = "idle"
        d, src, has_pose = self._distance(header, payloads, use="view", k=k)
        self.emit("view", k=k, use="view", snap=snap, pose=has_pose, d=d, d_source=src)
        if dcfg["d_min"] <= d <= dcfg["d_max"]:
            return
        self.activity = "moving"
        try:
            leg = self.driver.correct(k)
        finally:
            self.activity = "idle"
        self.stats.corrections += 1
        self.stats.stops += leg.stops
        self.link.sleep(self.cfg["capture"]["settle_s"])
        with turn():
            self.activity = "capturing"
            try:
                header, payloads, snap = self.link.snapshot(k, "retake", phi)
            finally:
                self.activity = "idle"
        d2, src2, has_pose = self._distance(header, payloads, use="retake", k=k)
        self.emit("view", k=k, use="retake", snap=snap, pose=has_pose, d=d2, d_source=src2)
        self.emit("correct", k=k, d_before=d, src_before=src, d_after=d2, src_after=src2)

    # ---- 5. 보강 촬영(웹 화면) ----
    def refix(self):
        """그 자리에서 한 장 찍어 표식으로 자세를 다시 잡는다. 표식이 안 보이면 StartError."""
        self.activity = "capturing"
        try:
            header, payloads, _ = self.link.snapshot(None, "fix", None)
        finally:
            self.activity = "idle"
        frame, pose = self._prepare_and_estimate(header, payloads)
        self._notify_capture("fix", None, frame, pose, header)
        if pose is None:
            raise StartError(f"{self.role}: 표식이 보이지 않아 자세를 다시 잡지 못함")
        self._fix(pose.T_cam_board, header)
        return pose

    def capture_extra(self, k, phi, turn=contextlib.nullcontext):
        """시점 번호 k(스캔 시점 밖)로 한 장 더 찍는다. → (frame, 표식 자세 또는 None, snap)."""
        with turn():
            self.activity = "capturing"
            try:
                header, payloads, snap = self.link.snapshot(k, "view", phi)
            finally:
                self.activity = "idle"
        frame = align.prepare(header, payloads, self.calib)
        pose = tagpose.estimate(frame.color, frame.K, self.board)
        self._notify_capture("focus", k, frame, pose, header)
        if pose is not None:
            self._fix(pose.T_cam_board, header)
        d = None if pose is None else tagpose.distance_d(pose.T_cam_board, self.r_eff)
        self.emit("view", k=k, use="focus", snap=snap, pose=pose is not None, d=d,
                  d_source="tag" if pose is not None else None)
        return frame, pose, snap

    # ---- 중단 ----
    def abort(self):
        self.link.abort()
