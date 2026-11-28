"""근접 촬영: 고른 지점 쪽으로 로봇이 원 위에서 물체에 다가가 찍고 원 위로 돌아온다(docs/specs/closeup.md).

원 위에서는 표식으로 자세를 잡고(E), 원 안쪽은 바퀴 주행거리(odom)로 가며 정면 LiDAR 거리가 줄어든 양으로 전진을 확인한다.
가까운 사진에서는 표식을 찾지 않는다. 카메라 자세는 E의 표식 자세를 로봇이 바닥 위에서 움직인 만큼 옮겨 구한다.
원 위 이동과 두 로봇의 각도 차 지키기는 보강 촬영(focus)과 같다. 다가가는 것은 찍을 로봇 한 대뿐이다.
"""
import math
import statistics

import cv2
import numpy as np

from ..robot import align, drive, tagpose
from ..vendor.pinky_commands import sector_min
from . import focus

STANDOFF_MIN = 0.20      # 카메라–지점 수평 거리의 아래 한계(이보다 가까우면 H의 낮은 점이 사진 밖)
STANDOFF_STEP = 0.02
EDGE_PX = 40             # 지점이 사진 가장자리에서 이만큼 안쪽이어야 한다
FRONT_SAMPLES = 5
FRONT_FLOOR = 0.10       # 정면 거리가 이보다 작아지면 선다(LiDAR에 물체가 안 보일 때의 한계)
GAIN_MIN = 0.03          # 원 위보다 이만큼도 못 다가가면 원 위에서 찍는다


def settings(cfg):
    c = cfg.get("closeup") or {}
    return {"standoff": max(float(c.get("standoff", 0.25)), STANDOFF_MIN),
            "clear_obj": float(c.get("clear_obj", 0.18)),
            "front_half_deg": float(c.get("front_half_deg", 10.0)),
            "front_tol": float(c.get("front_tol", 0.08)),
            "fix_tol": float(c.get("fix_tol", 0.02)),
            "front_sensor": str(c.get("front_sensor", "lidar"))}


def plan(markers, cfg, roles):
    """마커 목록 → 마커마다 {i, role, point, normal, phi}(로봇과 보는 방향은 보강 촬영과 같은 규칙)."""
    return [{k: p[k] for k in ("i", "role", "point", "normal", "phi")} for p in focus.plan(markers, cfg, roles)]


def shot_pose(point, phi_deg, standoff, mount):
    """지점을 방향 phi에서 standoff 떨어져 정면으로 보는 카메라 자리의 로봇 자세 (x, y, yaw)와 카메라 위치."""
    phi = math.radians(phi_deg)
    cam = (point[0] + standoff * math.cos(phi), point[1] + standoff * math.sin(phi))
    yaw = phi + math.pi - math.radians(mount["yaw_deg"])            # 카메라가 지점을 보게
    dx, dy = drive._rotate(mount["x"], mount["y"], yaw)
    return (cam[0] - dx, cam[1] - dy, drive.wrap_angle(yaw)), cam


def moved(T_cam_board, pose_from, pose_to):
    """표식 자세 T(로봇이 pose_from에 있을 때) → 로봇이 바닥 위에서 pose_to로 옮겨 간 뒤의 카메라 자세."""
    def flat(p):
        c, s = math.cos(p[2]), math.sin(p[2])
        return np.array([[c, -s, 0, p[0]], [s, c, 0, p[1]], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    step = flat(pose_to) @ np.linalg.inv(flat(pose_from))            # 판 좌표에서 로봇에 붙은 점이 옮겨 간 변환
    return np.asarray(T_cam_board, float) @ np.linalg.inv(step)


def choose_standoff(point, phi_deg, mount, T_at, pose_at, K, size, st, base_radius):
    """→ (standoff, 로봇 자세, 카메라 위치) 또는 (None, 이유, None).

    물체 쪽 여유(clear_obj)와 사진 틀(EDGE_PX)을 지킬 때까지 standoff를 늘린다. 원 위보다 GAIN_MIN도 못 다가가면 None.
    """
    w, h = size
    s = st["standoff"]
    while True:
        pose, cam = shot_pose(point, phi_deg, s, mount)
        if math.hypot(pose[0], pose[1]) > base_radius - GAIN_MIN:
            return None, "원 위보다 가까이 갈 자리가 없음", None
        clear = math.hypot(pose[0] - point[0], pose[1] - point[1]) >= st["clear_obj"]
        uv = focus.project(point, moved(T_at, pose_at, pose), K)
        framed = uv is not None and EDGE_PX <= uv[0] < w - EDGE_PX and EDGE_PX <= uv[1] < h - EDGE_PX
        if clear and framed:
            return s, pose, cam
        s += STANDOFF_STEP


class CloseupRun(focus.FocusRun):
    """마커마다: 원 위 이동(보강 촬영과 같음) → 표식으로 자세(E) → 다가가기 → 촬영·자르기 → 원 위로 돌아가 다시 표식.

    units·hold·cancelled·progress·log는 FocusRun과 같다. 다시 합치지 않는다(가까운 사진은 메시에 넣지 않는다).
    """

    def __init__(self, cfg, units, scan_dir, plans, hold, cancelled, progress, log):
        super().__init__(cfg, units, scan_dir, plans, hold, cancelled, progress, log)
        self.st = settings(cfg)

    # ---- 정면 거리 ----
    def _front_once(self, u):
        scan, age = u.link.scan()
        if scan is None or age > self.cfg["drive"]["scan_timeout_s"]:
            return None
        lidar = self.cfg["lidar"]
        mount = math.radians(lidar["mount_yaw_deg"])
        ranges = scan["ranges"]
        if lidar["mask"]:
            ranges = [math.inf if drive.in_mask(math.degrees(scan["angle_min"] + i * scan["angle_inc"] + mount),
                                                lidar["mask"]) else r for i, r in enumerate(ranges)]
        return sector_min(ranges, scan["angle_min"], scan["angle_inc"], 0.0,
                          math.radians(self.st["front_half_deg"]), mount, scan["range_min"])

    def _front(self, u):
        """멈춘 채 잰 정면 LiDAR 거리(여러 번의 중앙값) 또는 None(스캔이 없거나 정면에 아무것도 없음)."""
        if self.st["front_sensor"] != "lidar":
            return None
        vals = []
        for _ in range(FRONT_SAMPLES):
            v = self._front_once(u)
            if v is not None and math.isfinite(v):
                vals.append(v)
            u.link.sleep(0.12)
        return statistics.median(vals) if len(vals) >= 3 else None

    # ---- 다가가기 ----
    def _approach(self, u, target, phi):
        """지금 자리(E)에서 target 자세로: 회전 → 한 번에 전진 → 정면 거리로 확인·한 번 보정 → 회전.

        정면 거리는 지켜보는 데와, 보낸 것보다 더 다가갔을 때 물러나는 데만 쓴다. 덜 줄었다고 더 밀어 넣지 않는다.

        → (정면 거리 출처 'lidar'|'odom', 도중에 섰으면 이유 또는 None)
        """
        d = u.driver
        speed = self.cfg["drive"]["speed"]
        here = d.pose()
        heading = math.atan2(target[1] - here[1], target[0] - here[0])
        dist = math.hypot(target[0] - here[0], target[1] - here[1])
        skew = abs(drive.wrap_angle(heading - (math.radians(phi) + math.pi)))
        turn = math.degrees(drive.wrap_angle(heading - here[2]))
        if abs(turn) >= 1:
            u.link.move(f"turn {turn:.1f}")
        self._check()
        u.link.sleep(self.cfg["capture"]["settle_s"])
        r0 = self._front(u)
        limit = max(FRONT_FLOOR, r0 - dist - 0.05) if r0 is not None else FRONT_FLOOR

        def watch():
            if self.cancelled():
                return True
            v = self._front_once(u)
            return v is None or v < limit                   # 스캔이 끊기거나 예상보다 가까워지면 선다

        result = u.link.move(f"speed {speed}; forward {dist:.3f}", watch=watch)
        self._check()
        if result.stopped:
            return "odom", "다가가는 중 앞이 막혀 섬"
        source = "odom"
        u.link.sleep(self.cfg["capture"]["settle_s"])
        r1 = self._front(u)
        if r0 is not None and r1 is not None and abs((r0 - r1) - dist) <= self.st["front_tol"]:
            over = (r0 - r1) - dist                          # 줄어든 양만 쓴다(센서의 고정 오차가 빠진다)
            if abs(over) <= self.st["fix_tol"]:
                source = "lidar"                             # 정면 거리와 odom이 같은 말을 한다
            elif over > 0:                                   # 보낸 것보다 더 다가갔다: 그만큼 물러난다
                source = "lidar"
                u.link.move(f"speed {speed}; back {over:.3f}")
                self._check()
                x, y, yaw = d.pose()                         # odom은 다 물러났다고 보므로 추정 위치를 그만큼 앞에 둔다
                odo = (x - here[0]) * math.cos(heading) + (y - here[1]) * math.sin(heading)
                slip = dist - odo
                if abs(slip) <= 0.05:
                    d.set_fix((x + slip * math.cos(heading), y + slip * math.sin(heading), yaw), u.link.odom())
            # 덜 줄었으면 더 밀어 넣지 않는다: 정면 부채꼴이 좁아 가까이 가면 면의 가장 가까운 곳이 부채꼴 밖으로 나간다
        fmt = lambda v: "없음" if v is None else f"{v:.3f}"   # noqa: E731
        self.log(f"{u.role} 접근: 전진 {dist:.3f} m(면에서 {math.degrees(skew):.0f}° 비스듬), "
                 f"정면 거리 {fmt(r0)} → {fmt(r1)} m, 확인 {source}")
        yaw_now = d.pose()[2]
        turn = math.degrees(drive.wrap_angle(target[2] - yaw_now))
        if abs(turn) >= 1:
            u.link.move(f"turn {turn:.1f}")
        self._check()
        return source, None

    def _back_to(self, u, pose_e):
        """원 위의 자리로 돌아가(보호 직진) 표식으로 자세를 다시 잡는다. → 추정 자세와 표식 자세의 차 (m, °)."""
        self.progress["step"] = "return"
        u.driver._travel(pose_e, drive.Leg(-1), "required")
        u.link.sleep(self.cfg["capture"]["settle_s"])
        est = u.driver.pose()
        u.refix()
        got = u.driver.pose()
        return (round(math.hypot(got[0] - est[0], got[1] - est[1]), 3),
                round(math.degrees(abs(drive.wrap_angle(got[2] - est[2]))), 1))

    # ---- 촬영 ----
    def _shoot_close(self, plan, u, phi, k, T_e, pose_e, source, standoff, reason):
        self.progress["step"] = "capture"
        u.link.sleep(self.cfg["capture"]["settle_s"])
        self._check()
        self.hold(u.role)
        with self._turn():
            u.activity = "capturing"
            try:
                header, payloads, snap = u.link.snapshot(k, "closeup", phi)
            finally:
                u.activity = "idle"
        self.progress["shots"] += 1
        frame = align.prepare(header, payloads, u.calib)
        u._notify_capture("closeup", k, frame, None, header)
        T = moved(T_e, pose_e, u.driver.pose())
        item = {"target": plan["i"], "role": u.role, "k": k, "phi": phi, "snap": snap, "file": None, "full": None,
                "pixel": None, "distance_m": None, "source": source, "standoff_m": standoff, "reason": reason,
                "tag_px": None, "return_err": None}
        out = self.scan_dir / "closeup"
        out.mkdir(exist_ok=True)
        full = f"{k:06d}_{u.role}_full.jpg"
        cv2.imwrite(str(out / full), frame.color, [cv2.IMWRITE_JPEG_QUALITY, 90])
        item["full"] = full
        uv = focus.project(plan["point"], T, frame.K)
        h, w = frame.color.shape[:2]
        if uv is None or not (0 <= uv[0] < w and 0 <= uv[1] < h):
            item["reason"] = item["reason"] or "점이 사진 밖"
        else:
            jpg, _ = focus.crop_ring(frame.color, uv, self.crop)
            name = f"{k:06d}_{u.role}.jpg"
            (out / name).write_bytes(jpg)
            c = -T[:3, :3].T @ T[:3, 3]
            item.update(file=name, pixel=[round(uv[0], 1), round(uv[1], 1)],
                        distance_m=round(float(np.linalg.norm(c - np.array(plan["point"]))), 3))
            seen = tagpose.estimate(frame.color, frame.K, u.board)          # 기록용: 표식이 보이면 그 자세와의 차
            if seen is not None:
                uv2 = focus.project(plan["point"], seen.T_cam_board, frame.K)
                if uv2 is not None:
                    item["tag_px"] = round(math.hypot(uv2[0] - uv[0], uv2[1] - uv[1]), 1)
        self.crops.append(item)
        return item

    def run(self):
        """→ {"crops": […]}"""
        k = focus.next_k(self.scan_dir, list(self.units))
        for t, plan in enumerate(self.plans):
            self.progress["target"] = t + 1
            u = self.units[plan["role"]]
            self.log(f"근접 {t + 1}/{len(self.plans)}: {plan['role']}가 다가가서 찍음")
            self._check()
            self.hold()
            self._refix_all(plan["role"])
            self._check_gap()
            here = u.driver.angle_deg()
            phi = here if plan["phi"] is None else plan["phi"]
            self._move_all(focus._wrap(phi - here), plan["role"])
            self.progress["step"] = "fix"
            self._check()
            self.hold(u.role)
            T_e = np.asarray(u.refix().T_cam_board, float)        # E: 이 표식 자세가 근접 구간의 출발점
            pose_e = u.driver.pose()
            frame_size = (self.cfg["capture"]["color"]["w"], self.cfg["capture"]["color"]["h"])
            K = align.K_of(u.calib["color"])
            standoff, target, _ = choose_standoff(plan["point"], phi, u.mount, T_e, pose_e, K, frame_size, self.st,
                                                  math.hypot(pose_e[0], pose_e[1]))
            source, reason = "odom", None
            if standoff is None:
                reason, standoff = target, None                   # 원 위에서 찍는다
            else:
                self.progress["step"] = "approach"
                u.activity = "moving"
                try:
                    source, reason = self._approach(u, target, phi)
                finally:
                    u.activity = "idle"
            item = self._shoot_close(plan, u, phi, k, T_e, pose_e, source, standoff, reason)
            k += 1
            if standoff is not None:
                u.activity = "moving"
                try:
                    item["return_err"] = list(self._back_to(u, pose_e))
                finally:
                    u.activity = "idle"
        self.progress["step"] = "done"
        return {"crops": self.crops}
