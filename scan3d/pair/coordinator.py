"""[두 대 합침] 두 로봇(또는 한 대)의 스캔을 맞춰 진행한다(PLAN 4.4).

로봇마다 하는 일은 robot.unit.RobotScan에 있고, 여기는 두 대를 묶는 규칙만 둔다.
  - 카메라 일련번호가 서로 달라야 한다(같은 카메라를 두 자리에 꽂은 실수)
  - 두 로봇의 출발 각도 차가 180° ± pair_gap_tol_deg 안이어야 한다
  - 진입은 두 대가 동시에. 그 뒤 로봇마다 스레드로 24시점을 돈다
  - 스냅샷은 한 번에 한 로봇만(촬영 차례), 다른 로봇이 시점 k − 1에 도착하기 전에는 시점 k로 출발하지 않는다(앞섬 제한).
    그래서 한 로봇이 찍는 동안 다른 로봇은 다음 시점으로 이동한다(엇갈림)
  - 한 대라도 실패하면 둘 다 중단
"""
import contextlib
import math
import queue
import threading
import time

from ..config import ORDER
from ..robot import drive
from ..robot.unit import RobotScan, StartError
from ..vendor.pinky_commands import wrap_angle

WAIT_POLL_S = 0.01   # 촬영 차례·앞섬 제한을 기다릴 때 취소를 확인하는 주기(실제 초)


def run_parallel(jobs):
    """{role: fn}을 로봇마다 스레드로 함께 실행한다. 예외는 끝나는 대로 바로 올린다."""
    results, done = {}, queue.Queue()

    def run(role, fn):
        try:
            done.put((role, fn(), None))
        except BaseException as e:  # noqa: BLE001  스레드 예외를 주 스레드로 넘긴다
            done.put((role, None, e))

    for role, fn in jobs.items():
        threading.Thread(target=run, args=(role, fn), daemon=True).start()
    while len(results) < len(jobs):
        role, value, err = done.get()
        if err is not None:
            raise err
        results[role] = value
    return results


def check_serials(serials):
    """{role: 일련번호}가 서로 달라야 한다."""
    if len(set(serials.values())) != len(serials):
        raise StartError(f"두 로봇의 카메라 일련번호가 같음: {serials}")


def check_pair_gap(phi_l, phi_h, tol_deg):
    """두 로봇이 판 둘레에서 서로 반대편(180° ± tol)인지. 아니면 H를 옮길 방향과 각도를 알려 준다."""
    gap = math.degrees(wrap_angle(math.radians(phi_h - phi_l)))
    if abs(abs(gap) - 180.0) > tol_deg:
        need = math.degrees(wrap_angle(math.radians(phi_l + 180.0 - phi_h)))
        way = "반시계" if need > 0 else "시계"
        raise StartError(f"두 로봇 각도 차 {abs(gap):.0f}°(180 ± {tol_deg}° 밖): "
                         f"H를 판 둘레로 {way} 방향 {abs(need):.0f}° 옮긴다")


class PairScan:
    """links({role: RobotLink})로 한 번의 스캔 진행. writer는 dataset.ScanWriter(이벤트 기록)."""

    def __init__(self, cfg, links, writer, board, cancel):
        self.cfg = cfg
        self.writer = writer
        self.cancel = cancel
        self.roles = [r for r in ORDER if r in links]
        self.clock_link = links[self.roles[0]]
        self.units = {r: RobotScan(r, links[r], cfg, board, self._emitter(r)) for r in self.roles}

    # ---- 이벤트 ----
    def event(self, role, typ, **kw):
        link = self.units[role].link if role in self.units else self.clock_link
        self.writer.event({"pc_t": link.now(), "role": role, "type": typ, **kw})

    def _emitter(self, role):
        return lambda typ, **kw: self.event(role, typ, **kw)

    # ---- 단계 ----
    def hello(self):
        for u in self.units.values():
            u.hello()
        check_serials({r: u.serial for r, u in self.units.items()})
        for u in self.units.values():
            u.wait_streams()

    def start_poses(self):
        phis = {r: u.start_pose() for r, u in self.units.items()}
        if len(self.roles) == 2:
            check_pair_gap(phis["L"], phis["H"], self.cfg["drive"]["pair_gap_tol_deg"])

    def enter(self):
        run_parallel({r: u.enter for r, u in self.units.items()})

    def _check_cancel(self):
        if self.cancel.is_set():
            raise drive.Cancelled("취소됨")

    def views(self):
        """로봇마다 스레드로 시점 0…n−1. 촬영 차례와 앞섬 제한으로 두 로봇이 엇갈려 찍고 이동한다."""
        n = self.cfg["scene"]["n_views"]
        settle = self.cfg["capture"]["settle_s"]
        camera = threading.Lock()

        @contextlib.contextmanager
        def turn():
            while not camera.acquire(timeout=WAIT_POLL_S):
                self._check_cancel()
            try:
                yield
            finally:
                camera.release()

        def others_reached(role, k):
            return all(u.cur >= k for r, u in self.units.items() if r != role)

        def walk(role, u):
            for k in range(n):
                if u.needs_move(k):
                    while not others_reached(role, k - 1):
                        self._check_cancel()
                        time.sleep(WAIT_POLL_S)
                    u.go_to_view(k)
                if not u.at_view(k):
                    continue
                u.link.sleep(settle)
                self._check_cancel()
                u.capture(k, turn)

        run_parallel({r: (lambda r=r, u=u: walk(r, u)) for r, u in self.units.items()})

    def abort_all(self, reason):
        self.cancel.set()
        for u in self.units.values():
            u.abort()
        self.event(None, "abort", reason=reason)

    def report(self, aborted=None, elapsed_s=None):
        s = {r: u.stats for r, u in self.units.items()}
        return {"skipped": {r: list(x.skipped) for r, x in s.items()},
                "stops": {r: x.stops for r, x in s.items()},
                "corrections": {r: x.corrections for r, x in s.items()},
                "aborted": aborted, "elapsed_s": elapsed_s}

    def run(self):
        """hello → 출발 자세 → 진입 → 24시점. 실패하면 두 대 모두 중단하고 예외를 올린다."""
        self.hello()
        self.start_poses()
        self.enter()
        self.views()
