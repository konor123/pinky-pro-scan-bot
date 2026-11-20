"""판 좌표의 시점 이동과 장애물 우회."""

import math
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .vendor.pinky_commands import sector_min, wrap_angle


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
    center = (*cam_xy(pose, mount), mount["z"] - 0.005)
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


def view_pose(phi_deg, radius, mount):
    phi = math.radians(phi_deg)
    yaw = wrap_angle(phi + math.pi/2)
    dx, dy = _rotate(mount["x"], mount["y"], yaw)
    return radius*math.cos(phi) - dx, radius*math.sin(phi) - dy, yaw


def plan_move(pose, target, speed):
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
    turn(target[2] - heading)
    return commands


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
        self._phi0 = None
        self._fix = None

    def _event(self, kind, leg):
        if self._on_event:
            self._on_event(dict(event=kind, view=leg.view, t=self.io.now()))

    def set_fix(self, pose, odom_at_capture):
        self._fix = tuple(pose), tuple(odom_at_capture)

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
        return view_pose(self._phi0 + k*360/self._views, self._radius + lane, self.mount)

    def _check_cancel(self):
        if self.io.cancelled():
            raise Cancelled("주행 취소")

    def _clearance(self):
        return front_min(*self.io.scan(), self.cfg)

    def _wait(self, leg, mode):
        start = self.io.now()
        self._event("wait", leg)
        if mode == "outer":
            self.io.say("바깥 차선에 장애물이 있습니다")
        while True:
            self._check_cancel()
            clearance = self._clearance()
            if clearance >= self._drive["start_clear"]:
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

    def _travel(self, target, leg, mode):
        while True:
            self._check_cancel()
            commands = plan_move(self.pose(), target, self._drive["speed"])
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

    def correct(self, k):
        leg = Leg(k)
        self._travel(self._target(k), leg, "required")
        return leg

    def go_to_view(self, k):
        leg = Leg(k)
        if self._travel(self._target(k), leg, "normal"):
            return leg
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
