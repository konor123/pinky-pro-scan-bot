"""평면 주행과 LiDAR 시뮬레이터."""

import heapq
import math
from collections import deque, namedtuple
from itertools import count

import numpy as np

from ..protocol import parse_arc
from ..robot.drive import _rotate
from ..vendor.pinky_commands import Step, parse, wrap_angle


_RADIUS = 0.07
_PERIOD = 0.05
_BEAMS = np.arange(720) * math.tau / 720
_Arc = namedtuple("_Arc", "kind radius angle")   # 원호 단계: 반지름 m, 각도 rad(양수 반시계)


class World:
    def __init__(self, circles=(), boxes=()):
        self.circles = list(circles)
        self.boxes = list(boxes)
        self.robots = []

    def add_box(self, x, y, w, d, yaw_deg):
        box = (x, y, w, d, yaw_deg)
        self.boxes.append(box)
        return box

    def remove(self, box):
        self.boxes.remove(box)

    def _circles(self, observer):
        yield from self.circles
        for robot in self.robots:
            if robot is not observer:
                yield (*robot.true_pose()[:2], _RADIUS)

    def _overlaps(self, x, y, robot):
        for cx, cy, radius in self._circles(robot):
            if math.hypot(x - cx, y - cy) < radius + _RADIUS:
                return True
        for cx, cy, w, d, yaw in self.boxes:
            bx, by = _rotate(x - cx, y - cy, -math.radians(yaw))
            if math.hypot(max(0, abs(bx) - w/2), max(0, abs(by) - d/2)) < _RADIUS:
                return True
        return False


def _slab(origin, direction, half):
    parallel = np.abs(direction) < 1e-14
    safe = np.where(parallel, 1, direction)
    a, b = (-half - origin) / safe, (half - origin) / safe
    near = np.where(parallel, -np.inf if abs(origin) <= half else np.inf, np.minimum(a, b))
    return near, np.where(parallel, np.inf, np.maximum(a, b))


class Robot:
    def __init__(self, world, pose=(0, 0, 0), name="", linear_speed=0.10,
                 odom_noise=None, rng=None):
        self.world, self.name = world, name
        world.robots.append(self)
        self._pose = tuple(pose)
        self._odom = (0., 0., 0.)
        noise = odom_noise or {}
        self._scale = 1 + noise.get("scale", 0)
        self._drift = math.radians(noise.get("yaw_drift_deg_per_m", 0))
        self.linear_speed = linear_speed
        self._queue = deque()
        self._active = None
        self._messages = []
        self._linear = self._angular = self._tick_left = 0.
        self._turned = self._travelled = 0.

    def command(self, text):
        try:
            arc = parse_arc(text)
            if arc is None:
                steps = parse(text)
            else:
                speed, radius, angle = arc
                steps = [Step("speed", speed)] if speed is not None else []
                steps.append(_Arc("arc", radius, math.radians(angle)))
        except ValueError as exc:
            self._messages.append(f"error: {exc}")
            return
        if steps[0].kind == "stop":
            self._stop()
            return
        self._queue.extend(steps)
        self._messages.append(f"accepted: {text} (대기 {len(self._queue)}단계)")

    def _stop(self):
        self._queue.clear()
        self._active = None
        self._linear = self._angular = 0.
        self._messages.append("stopped: 남은 명령 취소")

    def statuses(self):
        messages, self._messages = self._messages, []
        return messages

    def true_pose(self):
        return self._pose

    def odom(self):
        return self._odom

    def busy(self):
        """실행 중이거나 남은 문장이 있으면 True."""
        return self._active is not None or bool(self._queue)

    def _control(self):
        self._linear = self._angular = 0.
        while self._active is None and self._queue:
            step = self._queue.popleft()
            if step.kind == "speed":
                self.linear_speed = step.value
                self._messages.append(f"speed: {step.value:.2f} m/s")
            elif step.kind == "stop":       # 문장 중간의 stop도 맨 앞 stop처럼 남은 명령을 지운다
                self._stop()
                return
            else:
                self._active = step
                self._start_xy = self._odom[:2]
                self._turned = self._travelled = 0.
        step = self._active
        if step is None:
            return
        if step.kind == "move":
            remaining = abs(step.value) - math.dist(self._start_xy, self._odom[:2])
            if remaining > 0.01:
                self._linear = math.copysign(max(0.03, min(self.linear_speed, 1.5*remaining)),
                                             step.value)
                return
            what = f"move {step.value:+.2f} m"
        elif step.kind == "turn":
            remaining = step.value - self._turned
            if abs(remaining) > math.radians(2):
                self._angular = math.copysign(max(0.15, min(0.8, 2*abs(remaining))), remaining)
                return
            what = f"turn {math.degrees(step.value):+.0f} deg"
        elif step.kind == "arc":
            # robot_agent의 원호 실행과 같다: odom 이동 거리가 반지름 × 각도에 이르면 끝.
            remaining = step.radius * abs(step.angle) - self._travelled
            if remaining > 0.003:
                self._linear = max(0.03, min(self.linear_speed, 1.5*remaining))
                self._angular = math.copysign(self._linear / step.radius, step.angle)
                return
            what = f"arc {step.radius:.3f} m {math.degrees(step.angle):+.1f} deg"
        else:
            return
        last = all(s.kind == "speed" for s in self._queue)   # 뒤에 speed만 남았으면 이 동작이 마지막
        self._messages.append("done: " + what + (" (모든 명령 완료)" if last else ""))
        self._active = None

    def step(self, dt):
        while dt > 1e-12:
            if self._tick_left < 1e-12:
                self._control()
                self._tick_left = _PERIOD
            part = min(self._tick_left, dt)
            dt -= part
            self._tick_left -= part
            x, y, yaw = self._pose
            distance = part * self._linear
            turn = part * self._angular
            dx, dy = _rotate(distance, 0, yaw + turn/2)
            if distance and self.world._overlaps(x + dx, y + dy, self):
                continue
            self._pose = x + dx, y + dy, wrap_angle(yaw + turn)
            ox, oy, heading = self._odom
            drift = self._drift * abs(distance)
            dx, dy = _rotate(distance * self._scale, 0, heading + (drift + turn)/2)
            self._odom = ox + dx, oy + dy, wrap_angle(heading + drift + turn)
            if turn:
                self._turned += turn
            if distance:
                self._travelled += abs(distance * self._scale)

    def scan(self):
        x, y, yaw = self._pose
        x -= 0.017 * math.cos(yaw)
        y -= 0.017 * math.sin(yaw)
        # 첫 빔은 스캔 각 −π, 장착 yaw π이므로 로봇 정면이다.
        angles = _BEAMS + yaw
        ux, uy = np.cos(angles), np.sin(angles)
        ranges = np.full(720, np.inf)
        for cx, cy, radius in self.world._circles(self):
            dx, dy = cx - x, cy - y
            along = dx * ux + dy * uy
            disc = radius**2 - (dx * uy - dy * ux)**2
            half = np.sqrt(np.maximum(0, disc))
            hit = np.where(along >= half, along - half, along + half)
            ranges = np.minimum(ranges, np.where((disc >= 0) & (hit >= 0), hit, np.inf))
        for cx, cy, w, d, angle in self.world.boxes:
            rot = -math.radians(angle)
            bx, by = _rotate(x - cx, y - cy, rot)
            vx, vy = _rotate(ux, uy, rot)
            nx, fx = _slab(bx, vx, w/2)
            ny, fy = _slab(by, vy, d/2)
            near, far = np.maximum(nx, ny), np.minimum(fx, fy)
            hit = np.where(near >= 0, near, far)
            ranges = np.minimum(ranges, np.where(far >= np.maximum(near, 0), hit, np.inf))
        ranges[(ranges < 0.05) | (ranges > 12.)] = np.inf
        return dict(angle_min=-math.pi, angle_inc=math.tau/720, range_min=0.05,
                    range_max=12., ranges=ranges.tolist())


class Sim:
    def __init__(self, world, robots, dt=0.05):
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.world, self.robots, self.dt = world, list(robots), dt
        self.now = 0.
        self._events = []
        self._seq = count()

    def at(self, t, fn):
        heapq.heappush(self._events, (t, next(self._seq), fn))

    def run_until(self, pred, timeout):
        end = self.now + timeout
        while True:
            while self._events and self._events[0][0] <= self.now + 1e-12:
                heapq.heappop(self._events)[2]()
            if pred():
                return True
            if end - self.now < 1e-12:
                return False
            stop = min(end, self.now + self.dt)
            if self._events:
                stop = min(stop, self._events[0][0])
            for robot in self.robots:
                robot.step(stop - self.now)
            self.now = stop


class SimIO:
    def __init__(self, sim, robot, scan_hz=10):
        self.sim, self.robot = sim, robot
        self.move_log, self.messages = [], []
        self.drop_scans = self._cancelled = False
        self._period = 1 / scan_hz
        self._capture()

    def _capture(self):
        if not self.drop_scans:
            self._last_scan = self.robot.scan()
            self._scan_time = self.now()
        self.sim.at(self.now() + self._period, self._capture)

    def scan(self):
        return self._last_scan, self.now() - self._scan_time

    def odom(self):
        return self.robot.odom()

    def now(self):
        return self.sim.now

    def sleep(self, s):
        self.sim.run_until(lambda: False, s)

    def say(self, text):
        self.messages.append(text)

    def cancel(self):
        self._cancelled = True

    def cancelled(self):
        return self._cancelled

    def _send(self, text):
        self.move_log.append((self.now(), text))
        self.robot.command(text)

    def move(self, text, watch=None, timeout_s=60):
        from ..robot.drive import MoveResult

        self.robot.statuses()
        self._send(text)
        deadline = self.now() + timeout_s
        next_watch = self.now() + 0.1
        while True:
            for status in self.robot.statuses():
                if status.endswith("(모든 명령 완료)") or status.startswith("stopped:"):
                    return MoveResult(True, status.startswith("stopped:"), self.odom())
                if status.startswith(("error:", "blocked:")):
                    return MoveResult(False, False, self.odom())
            if self.now() + 1e-12 >= deadline:
                self._send("stop")
                return MoveResult(False, True, self.odom())
            if self.now() + 1e-12 >= next_watch:
                next_watch += 0.1
                if self.cancelled() or (watch and watch()):
                    self._send("stop")
                    continue
            self.sleep(min(self.sim.dt, next_watch - self.now(), deadline - self.now()))
