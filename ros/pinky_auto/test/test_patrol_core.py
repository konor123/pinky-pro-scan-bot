import math
from pathlib import Path

import pytest

from pinky_auto.patrol_core import PatrolSequence, load_plan, parse_plan

WPS = [{'name': 'a', 'x': 1, 'y': 2, 'yaw_deg': 90}, {'x': 3, 'y': 4}]


def test_parse_defaults():
    plan = parse_plan({'waypoints': WPS})
    assert plan.frame_id == 'map'
    assert plan.loops == 0
    assert plan.initial_pose is None
    assert [w.name for w in plan.waypoints] == ['a', 'wp2']
    assert plan.waypoints[0].yaw == pytest.approx(math.pi / 2)


def test_quaternion():
    z, w = parse_plan({'waypoints': [{'x': 0, 'y': 0, 'yaw_deg': 180}]}).waypoints[0].quaternion_zw()
    assert z == pytest.approx(1.0)
    assert w == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('data', [
    None,
    {},
    {'waypoints': []},
    {'waypoints': [{'x': 1}]},
    {'waypoints': WPS, 'loops': -1},
    {'waypoints': WPS, 'initial_pose': {'y': 0}},
])
def test_parse_rejects(data):
    with pytest.raises(ValueError):
        parse_plan(data)


def run(plan, results):
    seq = PatrolSequence(plan)
    visited = []
    for ok in results:
        wp = seq.next()
        if wp is None:
            break
        visited.append(wp.name)
        seq.report(ok)
    return visited, seq


def test_fixed_loops():
    visited, seq = run(parse_plan({'waypoints': WPS, 'loops': 2}), [True] * 10)
    assert visited == ['a', 'wp2', 'a', 'wp2']
    assert seq.next() is None


def test_endless_loops():
    visited, seq = run(parse_plan({'waypoints': WPS}), [True] * 7)
    assert visited == ['a', 'wp2', 'a', 'wp2', 'a', 'wp2', 'a']
    assert seq.next() is not None


def test_single_failure_continues():
    visited, seq = run(parse_plan({'waypoints': WPS}), [False, True, False, True])
    assert visited == ['a', 'wp2', 'a', 'wp2']
    assert seq.next() is not None


def test_all_failing_ends():
    visited, seq = run(parse_plan({'waypoints': WPS}), [False] * 5)
    assert visited == ['a', 'wp2']
    assert seq.next() is None


def test_packaged_config_loads():
    plan = load_plan(Path(__file__).parents[1] / 'config' / 'waypoints.yaml')
    assert plan.initial_pose is not None
    assert len(plan.waypoints) == 3
