"""명령 문장 해석과 계산 도구. ROS 의존성 없음(단위 테스트 대상).

명령 문법 (';' 로 여러 개를 이어 쓴다)
  forward <m>      앞으로 m 만큼        예) forward 0.5
  back <m>         뒤로 m 만큼          예) back 0.2
  turn <deg>       제자리 회전(+ 왼쪽/반시계, - 오른쪽/시계)   예) turn 90, turn -45
  square <m>       한 변 m 인 정사각형(forward m; turn 90 x 4)
  speed <m/s>      직진 속도 변경(0.02 ~ 0.25, 실기 상한 고려)
  stop             즉시 정지, 남은 명령 취소
"""
import math
from dataclasses import dataclass
from typing import List, Sequence

MAX_DISTANCE = 5.0
MIN_SPEED, MAX_SPEED = 0.02, 0.25   # 실기 상한: MAX_RPM 100 x 바퀴 반경 0.027 m = 약 0.28 m/s


@dataclass(frozen=True)
class Step:
    kind: str     # 'move' | 'turn' | 'speed' | 'stop'
    value: float  # move: m(부호 = 방향), turn: rad, speed: m/s


def _number(word: str, token: str) -> float:
    try:
        return float(token)
    except ValueError:
        raise ValueError(f"'{word}' 뒤에는 숫자가 와야 합니다: '{token}'") from None


def parse_one(text: str) -> List[Step]:
    words = text.strip().lower().split()
    if not words:
        return []
    word, args = words[0], words[1:]
    if word == 'stop':
        return [Step('stop', 0.0)]
    if len(args) != 1:
        raise ValueError(f"'{word}' 는 숫자 1개가 필요합니다. 예) {word} 0.5")
    v = _number(word, args[0])
    if word in ('forward', 'back'):
        if not 0 < v <= MAX_DISTANCE:
            raise ValueError(f"거리는 0 ~ {MAX_DISTANCE} m: {v}")
        return [Step('move', v if word == 'forward' else -v)]
    if word == 'turn':
        if v == 0 or abs(v) > 720:
            raise ValueError(f"각도는 0이 아닌 -720 ~ 720 도: {v}")
        return [Step('turn', math.radians(v))]
    if word == 'square':
        if not 0 < v <= MAX_DISTANCE:
            raise ValueError(f"변 길이는 0 ~ {MAX_DISTANCE} m: {v}")
        return [Step('move', v), Step('turn', math.pi / 2)] * 4
    if word == 'speed':
        if not MIN_SPEED <= v <= MAX_SPEED:
            raise ValueError(f"속도는 {MIN_SPEED} ~ {MAX_SPEED} m/s: {v}")
        return [Step('speed', v)]
    raise ValueError(f"모르는 명령 '{word}'. forward/back/turn/square/speed/stop")


def parse(text: str) -> List[Step]:
    """'forward 0.5; turn 90' -> [Step(move,0.5), Step(turn,1.5708)]. 하나라도 틀리면 ValueError."""
    steps: List[Step] = []
    for part in text.split(';'):
        steps.extend(parse_one(part))
    if not steps:
        raise ValueError('빈 명령입니다')
    return steps


def wrap_angle(a: float) -> float:
    """-pi ~ pi 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rate_hz(times: Sequence[float]) -> float:
    """시각 목록(초)의 평균 주기(Hz). 2개 미만이거나 시간이 안 흘렀으면 0."""
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def sector_min(ranges: Sequence[float], angle_min: float, angle_increment: float,
               center: float, half_width: float, mount_yaw: float = 0.0,
               range_min: float = 0.0) -> float:
    """로봇 기준 center(rad) 방향 +-half_width 안의 최소 거리. 유효값이 없으면 inf.

    mount_yaw: 라이다가 로봇에 대해 돌아간 각도. Pinky는 rplidar_link가 pi 만큼 돌아가 있어서
    라이다 각도 0이 로봇 뒤쪽이다.
    """
    best = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min:
            continue
        robot_angle = angle_min + i * angle_increment + mount_yaw
        if abs(wrap_angle(robot_angle - center)) <= half_width:
            best = min(best, r)
    return best
