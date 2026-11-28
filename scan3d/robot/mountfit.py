"""카메라 장착 위치·방향을 주행하면서 표식으로 잰다(docs/specs/mountfit.md). numpy만 쓴다.

두 바퀴 로봇이 원호 문장 `arc r θ`를 달리면 차체는 바퀴 축의 연장선 위, 기준점에서 왼쪽으로 r(오른쪽으로 돌면 −r)
떨어진 점을 중심으로 돈다. 원호 앞뒤의 표식 자세 두 개에서 그 회전 중심을 카메라 기준으로 재면(q, 카메라가 보는
수평 방향이 x, 그 왼쪽이 y) 장착값 (x, y, yaw)와의 관계는

    q = Rot(−yaw) · (−x, r − y)

이다. 반지름이 다른 원호가 쌓이면 yaw까지 정해지고, 반지름이 비슷하면 지난 yaw를 둔 채 x·y만 맞춘다
(그 반지름에서 차체가 어디를 중심으로 도는지는 원호 하나로 정해진다).
"""
import math

import numpy as np

MIN_TURN_DEG = 5.0       # 이보다 덜 돈 원호는 회전 중심이 흔들려 쓰지 않는다
MAX_TURN_DEG = 150.0
TURN_TOL_DEG = 2.5       # 표식으로 잰 회전각과 odom 회전각이 이보다 다르면 그 원호는 쓰지 않는다(미끄러짐, 표식 자세 튐)
KEEP = 24                # 맞추는 데 쓰는 최근 원호 수
PRIOR_M2 = 0.05          # 지난 yaw를 붙드는 세기(반지름 흩어짐의 제곱합 m²과 견준다)
BOX_XY = 0.25            # 장착 위치 한도 m(기준점에서)
BOX_YAW_DEG = 25.0       # 장착 방향 한도(처음 값에서)
OUTLIER_M = 0.04         # 맞춘 뒤 이보다 벗어난 관측은 빼고 다시 맞춘다(4개 이상일 때)

_remembered = {}         # 카메라 일련번호 → 맞춘 장착값(프로세스가 사는 동안. 스캔 뒤 보강 촬영이 이어 쓴다)


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def planar(T_cam_board):
    """표식 자세 → 판 좌표의 카메라 위치 (x, y)와 카메라가 보는 수평 방향(rad)."""
    T = np.asarray(T_cam_board, float)
    R = T[:3, :3]
    c = -R.T @ T[:3, 3]
    return float(c[0]), float(c[1]), math.atan2(R[2, 1], R[2, 0])


def arc_centre(T_before, T_after):
    """원호 앞뒤 표식 자세 → (q, 돈 각 rad) 또는 None(덜 돌았거나 너무 돎).

    q는 원호 앞 카메라 기준(보는 방향 x, 왼쪽 y)의 회전 중심 위치(m).
    """
    x0, y0, h0 = planar(T_before)
    x1, y1, h1 = planar(T_after)
    turn = (h1 - h0 + math.pi) % (2*math.pi) - math.pi
    if not math.radians(MIN_TURN_DEG) <= abs(turn) <= math.radians(MAX_TURN_DEG):
        return None
    R = _rot(turn)
    c0, c1 = np.array([x0, y0]), np.array([x1, y1])
    centre = np.linalg.solve(np.eye(2) - R, c1 - R @ c0)
    return _rot(-h0) @ (centre - c0), turn


def observe(T_before, T_after, odom_yaw_before, odom_yaw_after, radius, angle_deg):
    """원호 문장 `arc radius angle_deg` 앞뒤의 표식 자세와 odom yaw → (q, 부호 있는 반지름) 또는 None.

    명령과 반대로 돌았거나, 표식으로 잰 회전각이 odom과 TURN_TOL_DEG 넘게 다르면 버린다.
    """
    seen = arc_centre(T_before, T_after)
    if seen is None or seen[1] * angle_deg <= 0:
        return None
    odom_turn = (odom_yaw_after - odom_yaw_before + math.pi) % (2*math.pi) - math.pi
    if abs(seen[1] - odom_turn) > math.radians(TURN_TOL_DEG):
        return None
    return seen[0], math.copysign(radius, angle_deg)


def predict(mount, radius):
    """장착값과 부호 있는 반지름(왼쪽으로 돌면 양수) → 카메라 기준 회전 중심 q."""
    return _rot(-math.radians(mount["yaw_deg"])) @ np.array([-mount["x"], radius - mount["y"]])


class MountFit:
    """원호 관측 (q, 부호 있는 반지름)을 모아 장착값 x, y, yaw_deg를 맞춘다.

    prior는 처음 값(config나 지난 맞춤). 그 밖의 장착값(z, pitch_down_deg)은 그대로 둔다.
    """

    def __init__(self, prior):
        self._prior = dict(prior)
        self._mount = dict(prior)
        self._obs = []

    @property
    def n(self):
        return len(self._obs)

    def mount(self):
        return dict(self._mount)

    def _solve(self, obs):
        r = np.array([o[1] for o in obs])
        q = np.array([o[0] for o in obs])
        dr, dq = r - r.mean(), q - q.mean(axis=0)
        yaw0 = math.radians(self._prior["yaw_deg"])
        left0 = np.array([math.sin(yaw0), math.cos(yaw0)])          # 카메라 기준 로봇 왼쪽 방향
        left = (dr @ dq + PRIOR_M2 * left0) / (dr @ dr + PRIOR_M2)
        left = left / np.linalg.norm(left)
        yaw = math.atan2(left[0], left[1])
        base = _rot(yaw) @ (q.mean(axis=0) - r.mean() * left)       # = (−x, −y)
        mount = dict(self._prior, x=float(-base[0]), y=float(-base[1]), yaw_deg=math.degrees(yaw))
        resid = [float(np.linalg.norm(predict(mount, ri) - qi)) for qi, ri in obs]
        return mount, resid

    def add(self, q, radius):
        """관측을 더하고 다시 맞춘다. → 장착값을 바꿨으면 True(한도 밖이면 지난 값을 둔다)."""
        q = np.asarray(q, float)
        if not (np.all(np.isfinite(q)) and math.isfinite(radius)) or np.linalg.norm(q) > 3.0:
            return False
        obs = (self._obs + [(q, float(radius))])[-KEEP:]
        mount, resid = self._solve(obs)
        if len(obs) >= 4:
            keep = [o for o, e in zip(obs, resid) if e <= max(OUTLIER_M, 3 * float(np.median(resid)))]
            if 3 <= len(keep) < len(obs):
                obs = keep
                mount, resid = self._solve(obs)
        dyaw = (mount["yaw_deg"] - self._prior["yaw_deg"] + 180) % 360 - 180
        if abs(mount["x"]) > BOX_XY or abs(mount["y"]) > BOX_XY or abs(dyaw) > BOX_YAW_DEG:
            return False                                             # 이 관측은 버린다
        self._obs, self._mount = obs, mount
        return True


def recall(serial):
    """이 프로세스에서 그 카메라로 맞춘 장착값(x, y, yaw_deg) 또는 None."""
    m = _remembered.get(serial)
    return dict(m) if m else None


def remember(serial, mount):
    if serial:
        _remembered[serial] = {k: float(mount[k]) for k in ("x", "y", "yaw_deg")}


def forget():
    _remembered.clear()
