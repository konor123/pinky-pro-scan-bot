import math
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Pose2D:
    name: str
    x: float
    y: float
    yaw: float

    def quaternion_zw(self):
        return math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)


@dataclass(frozen=True)
class PatrolPlan:
    frame_id: str
    initial_pose: Pose2D | None
    waypoints: tuple
    loops: int


def _pose(entry, default_name):
    if not isinstance(entry, dict) or 'x' not in entry or 'y' not in entry:
        raise ValueError(f'{default_name}: x and y are required')
    return Pose2D(
        name=str(entry.get('name', default_name)),
        x=float(entry['x']),
        y=float(entry['y']),
        yaw=math.radians(float(entry.get('yaw_deg', 0.0))),
    )


def parse_plan(data):
    if not isinstance(data, dict):
        raise ValueError('waypoint file must be a mapping')
    entries = data.get('waypoints') or []
    if not entries:
        raise ValueError('at least one waypoint is required')
    loops = int(data.get('loops', 0))
    if loops < 0:
        raise ValueError('loops must be 0 (endless) or positive')
    initial = data.get('initial_pose')
    return PatrolPlan(
        frame_id=str(data.get('frame_id', 'map')),
        initial_pose=_pose(initial, 'initial_pose') if initial is not None else None,
        waypoints=tuple(_pose(e, f'wp{i + 1}') for i, e in enumerate(entries)),
        loops=loops,
    )


def load_plan(path):
    with open(path, encoding='utf-8') as f:
        return parse_plan(yaml.safe_load(f))


class PatrolSequence:
    """Yields waypoints in order and ends when every waypoint fails in a row."""

    def __init__(self, plan):
        self._plan = plan
        self._index = 0
        self._lap = 0
        self._failures = 0

    @property
    def lap(self):
        return self._lap

    def next(self):
        count = len(self._plan.waypoints)
        if self._failures >= count:
            return None
        if self._plan.loops and self._lap >= self._plan.loops:
            return None
        return self._plan.waypoints[self._index]

    def report(self, succeeded):
        self._failures = 0 if succeeded else self._failures + 1
        self._index += 1
        if self._index >= len(self._plan.waypoints):
            self._index = 0
            self._lap += 1
