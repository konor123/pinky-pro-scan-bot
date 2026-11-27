"""[로봇 1대] 한 로봇의 스캔 단계(PLAN 4.4 진행 순서 중 로봇마다 하는 일).

  hello → start_pose(표식으로 출발 자세) → enter(원 진입) → go_to_view(k) → capture(k)(거리 벗어나면 보정·재촬영)

두 로봇을 함께 돌리는 순서와 조건은 pair.coordinator가 정한다. 이 모듈은 다른 로봇을 모른다.
"""
import contextlib
import math
from dataclasses import dataclass, field

from . import align, drive, tagpose


class StartError(RuntimeError):
    """움직이기 전에 멈추는 출발 조건 실패."""


@dataclass
class UnitStats:
    skipped: list = field(default_factory=list)   # 우회로 건너뛴 시점
    stops: int = 0                                # 장애물 정지 횟수
    corrections: int = 0                          # 거리 보정 횟수


class RobotScan:
    """로봇 하나의 스캔. link는 RobotLink(또는 같은 인터페이스), emit(type, **kw)는 이벤트 기록."""

    def __init__(self, role, link, cfg, board, emit):
        self.role = role
        self.link = link
        self.cfg = cfg
        self.board = board
        self.emit = emit
        self.mount = cfg["robots"][role]["mount"]
        scene = cfg["scene"]
        self.r_eff = scene["r_obj"] + scene["place_err"]
        self.driver = drive.Driver(link, cfg, self.mount,
                                   on_event=lambda ev: self.emit("drive", **ev))
        self.calib = None
        self.serial = None
        self.cur = -1          # 마지막으로 도착한 시점(아직 없으면 −1)
        self.phi0 = None       # 출발 때 판 좌표 각(도)
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
        self.emit("hello", arc=self.driver.arc)
        return reply

    def wait_streams(self):
        self.link.wait_streams()

    # ---- 2. 출발 자세 ----
    def _estimate(self, header, payloads):
        frame = align.prepare(header, payloads, self.calib)
        return tagpose.estimate(frame.color, frame.K, self.board)

    def _fix(self, T, header):
        self.driver.set_fix(drive.base_from_cam(T, self.mount), header["odom"])

    def start_pose(self):
        """표식으로 출발 자세를 잡고 판 좌표 각 phi(도)를 돌려준다."""
        header, payloads, snap = self.link.snapshot(None, "start", None)
        pose = self._estimate(header, payloads)
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
        self._record_leg(0, self.driver.enter())

    def needs_move(self, k):
        return self.cur < k

    def go_to_view(self, k):
        self._record_leg(k, self.driver.go_to_view(k))

    def at_view(self, k):
        return self.cur == k

    # ---- 4. 촬영 ----
    def _distance(self, header, payloads):
        """→ (d, 출처 'tag'|'odom', 표식 자세 있음). 표식이 보이면 자세도 고친다."""
        pose = self._estimate(header, payloads)
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
            header, payloads, snap = self.link.snapshot(k, "view", phi)
        d, src, has_pose = self._distance(header, payloads)
        self.emit("view", k=k, use="view", snap=snap, pose=has_pose, d=d, d_source=src)
        if dcfg["d_min"] <= d <= dcfg["d_max"]:
            return
        leg = self.driver.correct(k)
        self.stats.corrections += 1
        self.stats.stops += leg.stops
        self.link.sleep(self.cfg["capture"]["settle_s"])
        with turn():
            header, payloads, snap = self.link.snapshot(k, "retake", phi)
        d2, src2, has_pose = self._distance(header, payloads)
        self.emit("view", k=k, use="retake", snap=snap, pose=has_pose, d=d2, d_source=src2)
        self.emit("correct", k=k, d_before=d, src_before=src, d_after=d2, src_after=src2)

    # ---- 5. 보강 촬영(웹 화면) ----
    def refix(self):
        """그 자리에서 한 장 찍어 표식으로 자세를 다시 잡는다. 표식이 안 보이면 StartError."""
        header, payloads, _ = self.link.snapshot(None, "fix", None)
        pose = self._estimate(header, payloads)
        if pose is None:
            raise StartError(f"{self.role}: 표식이 보이지 않아 자세를 다시 잡지 못함")
        self._fix(pose.T_cam_board, header)
        return pose

    def capture_extra(self, k, phi, turn=contextlib.nullcontext):
        """시점 번호 k(스캔 시점 밖)로 한 장 더 찍는다. → (frame, 표식 자세 또는 None, snap)."""
        with turn():
            header, payloads, snap = self.link.snapshot(k, "view", phi)
        frame = align.prepare(header, payloads, self.calib)
        pose = tagpose.estimate(frame.color, frame.K, self.board)
        if pose is not None:
            self._fix(pose.T_cam_board, header)
        d = None if pose is None else tagpose.distance_d(pose.T_cam_board, self.r_eff)
        self.emit("view", k=k, use="focus", snap=snap, pose=pose is not None, d=d,
                  d_source="tag" if pose is not None else None)
        return frame, pose, snap

    # ---- 중단 ----
    def abort(self):
        self.link.abort()
