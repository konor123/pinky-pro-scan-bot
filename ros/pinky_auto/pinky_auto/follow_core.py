"""초음파로 앞 로봇과 간격을 유지하며 원 위를 따라 도는 계산(ROS 없이 테스트 가능).

- 속도: v = kp × (거리 − gap)를 [0, v_max]로 자르고 가속 한도를 준다. 뒤로는 가지 않는다.
- 방향: 원 반지름 R 위를 돌도록 ω = dir × v / R (dir=+1 왼쪽(반시계), −1 오른쪽).
- 초음파 값은 튀는 값이 섞이므로 최근 n개 중앙값을 쓴다.
- 너무 가까우면(stop_dist 미만) 즉시 정지, 앞 로봇을 놓치면(lost_dist 초과·값 없음) 정지하고 기다린다.
- 한 바퀴: odom yaw 변화량을 누적해 2π × laps를 넘으면 끝.
"""
import math
from collections import deque
from dataclasses import dataclass

FOLLOW = 'follow'
TOO_CLOSE = 'too_close'
LOST = 'lost'
DONE = 'done'


@dataclass(frozen=True)
class FollowParams:
    gap: float = 0.15          # 유지할 간격(m)
    kp: float = 2.5            # 간격 오차 → 속도 이득(1/s)
    v_max: float = 0.12        # 최대 직진 속도(m/s)
    accel: float = 0.3         # 가속 한도(m/s²). 감속은 즉시
    radius: float = 0.62       # 원 반지름(m)
    direction: int = 1         # +1 왼쪽(반시계), −1 오른쪽(시계)
    stop_dist: float = 0.08    # 이보다 가까우면 즉시 정지
    lost_dist: float = 0.60    # 이보다 멀면 앞 로봇을 놓친 것으로 봄
    deadband: float = 0.01     # 간격 오차가 이 안이면 오차 0으로 봄
    window: int = 5            # 중앙값 필터 길이

    def __post_init__(self):
        if self.gap <= self.stop_dist:
            raise ValueError('gap must be larger than stop_dist')
        if self.lost_dist <= self.gap:
            raise ValueError('lost_dist must be larger than gap')
        if self.radius <= 0 or self.v_max <= 0 or self.kp <= 0 or self.accel <= 0:
            raise ValueError('radius, v_max, kp, accel must be positive')
        if self.direction not in (1, -1):
            raise ValueError('direction must be 1 or -1')
        if self.window < 1:
            raise ValueError('window must be >= 1')


class MedianFilter:
    def __init__(self, n):
        self._buf = deque(maxlen=n)

    def push(self, value):
        """유효한 값만 넣는다(None·nan·음수는 버림). 현재 중앙값(없으면 None)을 돌려준다."""
        if value is not None and math.isfinite(value) and value >= 0.0:
            self._buf.append(float(value))
        return self.value()

    def value(self):
        if not self._buf:
            return None
        s = sorted(self._buf)
        m = len(s) // 2
        return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])

    def clear(self):
        self._buf.clear()


class Follower:
    def __init__(self, params: FollowParams):
        self.p = params
        self.v = 0.0

    def step(self, dist, dt):
        """필터된 거리(m 또는 None)와 주기(s) → (v, ω, 상태)."""
        p = self.p
        if dist is None or dist > p.lost_dist:
            self.v = 0.0
            return 0.0, 0.0, LOST
        if dist < p.stop_dist:
            self.v = 0.0
            return 0.0, 0.0, TOO_CLOSE
        err = dist - p.gap
        if abs(err) < p.deadband:
            err = 0.0
        target = min(max(p.kp * err, 0.0), p.v_max)
        if target > self.v:
            self.v = min(target, self.v + p.accel * max(dt, 0.0))
        else:
            self.v = target  # 감속은 바로
        return self.v, p.direction * self.v / p.radius, FOLLOW

    def stop(self):
        self.v = 0.0


class LapCounter:
    """odom yaw를 받아 누적 회전량(rad, 진행 방향 기준)을 센다."""

    def __init__(self, direction=1):
        self.direction = direction
        self._last = None
        self.turned = 0.0

    def update(self, yaw):
        if self._last is not None:
            d = math.atan2(math.sin(yaw - self._last), math.cos(yaw - self._last))
            self.turned += self.direction * d
        self._last = yaw
        return self.turned

    def done(self, laps):
        return laps > 0 and self.turned >= 2.0 * math.pi * laps


def yaw_from_quat(z, w, x=0.0, y=0.0):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
