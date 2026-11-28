"""판 좌표의 시점 이동과 장애물 우회."""

import math
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from ..vendor.pinky_commands import sector_min, wrap_angle


Pose2 = tuple[float, float, float]


def _rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x - s*y, s*x + c*y


def cam_xy(pose, mount):
    dx, dy = _rotate(mount["x"], mount["y"], pose[2])
    return pose[0] + dx, pose[1] + dy


def camera_pose(pose, mount):
    yaw = pose[2] + math.radians(mount["yaw_deg"])
    pitch = math.radians(mount["pitch_down_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    right = [s, -c, 0.]
    forward = [cp*c, cp*s, -sp]
    rotation = np.array([right, np.cross(forward, right), forward])
    center = (*cam_xy(pose, mount), mount["z"])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ center
    return transform


def base_from_cam(T_cam_board, mount):
    rotation = T_cam_board[:3, :3]
    center = -rotation.T @ T_cam_board[:3, 3]
    yaw = wrap_angle(math.atan2(rotation[2, 1], rotation[2, 0]) - math.radians(mount["yaw_deg"]))
    dx, dy = _rotate(mount["x"], mount["y"], yaw)
    return center[0] - dx, center[1] - dy, yaw


def view_pose(phi_deg, radius, mount, yaw=None):
    """카메라가 반지름 radius 원의 각 phi에 오는 로봇 자세. yaw가 없으면 반시계 접선."""
    phi = math.radians(phi_deg)
    yaw = wrap_angle(phi + math.pi/2) if yaw is None else wrap_angle(yaw)
    dx, dy = _rotate(mount["x"], mount["y"], yaw)
    return radius*math.cos(phi) - dx, radius*math.sin(phi) - dy, yaw


def arc_yaw(phi_deg, radius, mount):
    """카메라가 반지름 radius 원의 각 phi에 있고 판 중심 둘레의 원호로 달릴 수 있는 로봇 yaw.

    두 바퀴 로봇은 바퀴 축의 연장선 위의 점을 중심으로만 돈다. 그 선이 판 중심을 지나려면 로봇 방향이
    로봇 기준점이 그리는 원의 접선이어야 한다. 카메라가 기준점보다 뒤(mount.x < 0)에 있으면 카메라 원의
    접선보다 안쪽으로 조금 더 돈 방향이다(0.08 m, 반지름 0.62 m에서 7.4°).
    """
    return wrap_angle(math.radians(phi_deg) + math.acos(max(-1., min(1., mount["x"] / radius))))


def plan_move(pose, target, speed, final_turn=True):
    commands = []
    def turn(angle):
        degrees = math.degrees(wrap_angle(angle))
        if abs(degrees) >= 1:
            commands.append(f"turn {degrees:.1f}")

    dx, dy = target[0] - pose[0], target[1] - pose[1]
    distance = math.hypot(dx, dy)
    heading = pose[2]
    if distance >= 0.02:
        heading = math.atan2(dy, dx)
        turn(heading - pose[2])
        millimeters = round(distance * 1000)
        count = math.ceil(millimeters / 250)
        length, remainder = divmod(millimeters, count)
        for i in range(count):
            meters = (length + int(i < remainder)) / 1000
            commands.append(f"speed {speed}; forward {meters:.3f}")
    if final_turn or distance < 0.02:
        turn(target[2] - heading)
    return commands


ORBIT_POLE = 0.5                    # 원호마다 남기는 오차 비율(작을수록 빨리 고치고 자세 잡음에 민감하다)
ORBIT_MAX_OFF = 0.05                # 원에서 이보다 멀거나
ORBIT_MAX_YAW = math.radians(20.0)  # 방향이 이보다 틀어져 있으면 회전 + 직진으로 자리를 다시 잡는다
ORBIT_MIN_RADIUS = 0.10
ORBIT_STRAIGHT = 20.0               # 반지름이 이보다 크면 직진


def orbit_error(pose, radius):
    """판 중심 둘레 반지름 radius의 원에서 벗어난 만큼 → (반지름 방향 m, 바깥이 양수; 접선에서 틀어진 rad, 왼쪽이 양수)."""
    off = math.hypot(pose[0], pose[1]) - radius
    return off, wrap_angle(pose[2] - (math.atan2(pose[1], pose[0]) + math.pi/2))


def plan_orbit(pose, target, speed):
    """판 중심 둘레의 원 위 자세 `target`(`arc_yaw` 방향)까지 원호 하나로 가는 문장: `speed s; arc r θ`.

    기준 경로는 판 중심 둘레, 목표의 기준점 반지름 r의 원이다. 지금 자세가 그 원에서 벗어난 만큼(반지름 방향
    δ, 바깥이 양수)과 방향이 접선에서 틀어진 만큼(e, 왼쪽이 양수)을 원호의 휨으로 조금씩 고친다:
        휨 = 1/r + k1·δ − k2·e,   k1 = (1 − p)² / s²,   k2 = (3 + p)(1 − p) / (2s)
    s는 목표까지의 원호 길이, p는 `ORBIT_POLE`이다(한 번에 다 고치려 하면 몇 mm의 자세 잡음이 몇 도의
    방향 변화로 번진다). 원호 길이는 s, 문장의 각도는 s × 휨이다. 회전 문장을 쓰지 않는다.
    목표가 2 cm 안이거나 뒤쪽이거나, 원에서 5 cm·20° 넘게 벗어나 있으면 `plan_move`의 문장을 돌려준다.
    """
    radius = math.hypot(target[0], target[1])
    sweep = wrap_angle(math.atan2(target[1], target[0]) - math.atan2(pose[1], pose[0]))
    length = radius * sweep
    off, yaw_err = orbit_error(pose, radius)
    if length < 0.02 or abs(off) > ORBIT_MAX_OFF or abs(yaw_err) > ORBIT_MAX_YAW:
        return plan_move(pose, target, speed)
    k1 = (1 - ORBIT_POLE)**2 / length**2
    k2 = (3 + ORBIT_POLE) * (1 - ORBIT_POLE) / (2*length)
    bend = 1/radius + k1*off - k2*yaw_err
    if abs(bend) < 1/ORBIT_STRAIGHT:
        return [f"speed {speed}; forward {length:.3f}"]
    if abs(bend) > 1/ORBIT_MIN_RADIUS:
        return plan_move(pose, target, speed)
    return [f"speed {speed}; arc {1/abs(bend):.3f} {math.degrees(length*bend):.2f}"]


def in_mask(angle_deg, mask):
    return any(abs(end-start) >= 360 or (angle_deg-start) % 360 <= (end-start) % 360
               for start, end in mask)


def front_min(scan, age_s, cfg):
    drive = cfg["drive"]
    if scan is None or age_s > drive["scan_timeout_s"]:
        return 0.
    lidar = cfg["lidar"]
    mount = math.radians(lidar["mount_yaw_deg"])
    start, inc = scan["angle_min"], scan["angle_inc"]
    ranges = scan["ranges"]
    if lidar["mask"]:
        ranges = [math.inf if in_mask(math.degrees(start + i*inc + mount), lidar["mask"]) else r
                  for i, r in enumerate(ranges)]
    return sector_min(ranges, start, inc, 0., math.radians(drive["sector_half_deg"]),
                      mount, scan["range_min"])


@dataclass
class MoveResult:
    ok: bool
    stopped: bool
    odom: Pose2


class RobotIO(Protocol):
    def move(self, text, watch=None) -> MoveResult: ...
    def scan(self) -> tuple[dict | None, float]: ...
    def odom(self) -> Pose2: ...
    def now(self) -> float: ...
    def sleep(self, s): ...
    def say(self, text): ...
    def cancelled(self) -> bool: ...


@dataclass
class Leg:
    view: int
    skipped: list[int] = field(default_factory=list)
    stops: int = 0
    waited_s: float = 0.


class Blocked(RuntimeError):
    pass


class Cancelled(RuntimeError):
    pass


class Driver:
    def __init__(self, io: RobotIO, cfg, mount, on_event=None):
        self.io, self.cfg, self.mount = io, cfg, mount
        self._drive = cfg["drive"]
        self._radius = cfg["scene"]["R"]
        self._views = cfg["scene"]["n_views"]
        self._on_event = on_event
        # 시점 이동을 원호로(끄면 회전 + 직진). 로봇의 에이전트가 원호 문장을 모르면 unit이 끈다.
        self.arc = bool(self._drive.get("arc", False))
        self._phi0 = None
        self._fix = None
        self._sent = []      # 지난 take_sent() 뒤에 보낸 이동 문장 (문장, 도중에 섰는지)

    def _event(self, kind, leg):
        if self._on_event:
            self._on_event(dict(event=kind, view=leg.view, t=self.io.now()))

    def set_fix(self, pose, odom_at_capture):
        self._fix = tuple(pose), tuple(odom_at_capture)

    def take_sent(self):
        """지난 호출 뒤에 보낸 이동 문장 [(문장, 도중에 섰는지)]을 돌려주고 비운다(장착값 맞춤이 쓴다)."""
        sent, self._sent = self._sent, []
        return sent

    def pose(self):
        if self._fix is None:
            raise ValueError("표식 자세가 필요합니다")
        base, origin = self._fix
        x, y, yaw = self.io.odom()
        offset = base[2] - origin[2]
        dx, dy = _rotate(x - origin[0], y - origin[1], offset)
        return base[0] + dx, base[1] + dy, wrap_angle(yaw + offset)

    def phi_deg(self, k):
        """시점 k의 목표 각(도). enter() 뒤에만 쓴다."""
        return self._phi0 + k*360/self._views

    def _angle(self):
        x, y = cam_xy(self.pose(), self.mount)
        return math.degrees(math.atan2(y, x))

    def _target(self, k, lane=0):
        phi, radius = self._phi0 + k*360/self._views, self._radius + lane
        if self.arc:
            return view_pose(phi, radius, self.mount, yaw=arc_yaw(phi, radius, self.mount))
        return view_pose(phi, radius, self.mount)

    def _check_cancel(self):
        if self.io.cancelled():
            raise Cancelled("주행 취소")

    def _clearance(self):
        return front_min(*self.io.scan(), self.cfg)

    def _wait(self, leg, mode):
        start = self.io.now()
        self._check_cancel()
        self._event("wait", leg)
        if mode == "outer":
            self.io.say("바깥 차선에 장애물이 있습니다")
        while True:
            self._check_cancel()
            clearance = self._clearance()
            if clearance >= self._drive["start_clear"]:
                self._event("go", leg)
                return True
            elapsed = self.io.now() - start
            if elapsed + 1e-9 >= self._drive["block_timeout_s"]:
                self._event("blocked", leg)
                raise Blocked("주행 경로 막힘")
            # 스캔이 끊기면 길이 비었는지 판단할 수 없다.
            if clearance > 0 and mode == "return":
                return False
            if clearance > 0 and mode == "normal" and elapsed + 1e-9 >= self._drive["wait_s"]:
                return False
            pause = min(0.2, self._drive["block_timeout_s"] - elapsed)
            self.io.sleep(pause)
            leg.waited_s += pause

    def _travel(self, target, leg, mode, final_turn=True, arc=False):
        while True:
            self._check_cancel()
            if arc:
                commands = plan_orbit(self.pose(), target, self._drive["speed"])
            else:
                commands = plan_move(self.pose(), target, self._drive["speed"], final_turn)
            for command in commands:
                self._check_cancel()
                guarded = command.startswith("speed ") and self._drive["obstacle"]
                watch = None
                if guarded:
                    clearance = self._clearance()
                    if clearance < self._drive["start_clear"]:
                        if mode == "return" and clearance > 0:
                            return False
                        if not self._wait(leg, mode):
                            return False
                    watch = lambda: self.io.cancelled() or self._clearance() < self._drive["stop_dist"]
                result = self.io.move(command, watch=watch)
                self._sent = (self._sent + [(command, bool(result.stopped))])[-50:]
                self._check_cancel()
                if not result.ok:
                    self._event("blocked", leg)
                    raise Blocked(command)
                if result.stopped:
                    leg.stops += 1
                    self._event("stop", leg)
                    if mode == "return" or not self._wait(leg, mode):
                        return False
                    break
            else:
                return True

    def enter(self):
        self._phi0 = self._angle()
        return self.correct(0)

    def angle_deg(self):
        """지금 카메라의 판 좌표 각(도)."""
        return self._angle()

    def on_orbit(self):
        """지금 자세에서 반시계 쪽 원 위의 각으로 원호 하나로 갈 수 있는지(원호 모드, 원에서 5 cm·접선에서 20° 안)."""
        if not self.arc:
            return False
        stance = view_pose(0.0, self._radius, self.mount, yaw=arc_yaw(0.0, self._radius, self.mount))
        off, yaw_err = orbit_error(self.pose(), math.hypot(stance[0], stance[1]))
        return abs(off) <= ORBIT_MAX_OFF and abs(yaw_err) <= ORBIT_MAX_YAW

    def go_to_phi(self, phi_deg):
        """원 위의 각 phi(도)로 한 번에 간다(보강 촬영).

        `on_orbit()`이고 phi가 반시계 쪽이면 원호 하나라 180° 안쪽까지 된다. 그 밖(회전 + 직진)은 15° 안쪽만
        (더 멀면 현이 원 안을 지난다). 시점 이동과 같은 문장·장애물 감시. 비켜 가지 않고 막히면
        `block_timeout_s` 뒤 Blocked.
        """
        if self.arc:
            target = view_pose(phi_deg, self._radius, self.mount, yaw=arc_yaw(phi_deg, self._radius, self.mount))
        else:
            target = view_pose(phi_deg, self._radius, self.mount)
        leg = Leg(-1)
        self._travel(target, leg, "required", arc=self.arc)
        return leg

    def correct(self, k):
        leg = Leg(k)
        self._travel(self._target(k), leg, "required")
        return leg

    def go_to_view(self, k):
        """시점 k로. 판 중심 둘레의 원호 하나로 간다(`drive.arc`, 서는 방향은 `arc_yaw`).

        원호를 끄면 마지막 회전 없이 진행 방향 그대로 서되, 그 yaw로 카메라가 원 위에 오게 간다.
        """
        leg = Leg(k)
        x, y, _ = self.pose()
        target = self._target(k)
        if self.arc:
            done = self._travel(target, leg, "normal", arc=True)
        elif math.hypot(target[0] - x, target[1] - y) < 0.02:
            done = self._travel(target, leg, "normal")
        else:
            heading = math.atan2(target[1] - y, target[0] - x)
            arrive = view_pose(self._phi0 + k*360/self._views, self._radius, self.mount, yaw=heading)
            done = self._travel(arrive, leg, "normal", final_turn=False)
        if done:
            return leg
        return self._bypass(k, leg)

    def _bypass(self, k, leg):
        """바깥 차선으로 비켜 가 시점 k부터 돌아갈 수 있는 첫 시점에 선다(회전 + 직진)."""
        self._event("bypass", leg)
        lane = self._drive["bypass_lane"]
        outer = view_pose(self._angle(), self._radius + lane, self.mount)
        self._travel(outer, leg, "outer")
        for j in range(k, self._views):
            leg.view = j
            outer = self._target(j, lane)
            self._travel(outer, leg, "outer")
            if self._travel(self._target(j), leg, "return"):
                return leg
            leg.skipped.append(j)
            self._event("skip", leg)
            self._travel(outer, leg, "outer")
        leg.view = self._views
        return leg
