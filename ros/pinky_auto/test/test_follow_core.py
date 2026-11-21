import math

import pytest

from pinky_auto.follow_core import (FOLLOW, LOST, TOO_CLOSE, FollowParams, Follower, LapCounter,
                                    MedianFilter, yaw_from_quat)

DT = 0.05


def settled(f, dist, n=100):
    for _ in range(n):
        out = f.step(dist, DT)
    return out


def test_holds_still_at_gap():
    v, w, state = settled(Follower(FollowParams()), 0.15)
    assert state == FOLLOW
    assert v == 0.0 and w == 0.0


def test_speeds_up_when_far_and_clamps():
    p = FollowParams()
    v, w, _ = settled(Follower(p), 0.19)
    assert v == pytest.approx(p.kp * 0.04)
    v, _, _ = settled(Follower(p), 0.50)
    assert v == pytest.approx(p.v_max)


def test_turn_follows_circle():
    p = FollowParams(radius=0.5, direction=-1)
    v, w, _ = settled(Follower(p), 0.30)
    assert w == pytest.approx(-v / 0.5)


def test_never_reverses_when_close():
    v, _, state = settled(Follower(FollowParams()), 0.10)
    assert state == FOLLOW and v == 0.0


def test_accel_limited_decel_immediate():
    p = FollowParams()
    f = Follower(p)
    v1, _, _ = f.step(0.50, DT)
    assert v1 == pytest.approx(p.accel * DT)
    settled(f, 0.50)
    v2, _, _ = f.step(0.15, DT)
    assert v2 == 0.0


@pytest.mark.parametrize('dist,state', [(None, LOST), (0.9, LOST), (0.05, TOO_CLOSE)])
def test_stops(dist, state):
    f = Follower(FollowParams())
    settled(f, 0.40)
    v, w, s = f.step(dist, DT)
    assert (v, w, s) == (0.0, 0.0, state)


def test_median_rejects_spike_and_invalid():
    m = MedianFilter(5)
    for x in (0.15, 0.15, 0.83, 0.16, None, float('nan'), -1.0, 0.15):
        out = m.push(x)
    assert out == pytest.approx(0.15)
    assert MedianFilter(3).value() is None


def test_lap_counter_wraps():
    lap = LapCounter(direction=1)
    for k in range(0, 73):
        yaw = math.atan2(math.sin(math.radians(k * 5)), math.cos(math.radians(k * 5)))
        lap.update(yaw)
    assert lap.turned == pytest.approx(2 * math.pi)
    assert lap.done(1) and not lap.done(2)
    cw = LapCounter(direction=-1)
    cw.update(0.0)
    cw.update(-0.5)
    assert cw.turned == pytest.approx(0.5)


def test_yaw_from_quat():
    assert yaw_from_quat(math.sin(0.3), math.cos(0.3)) == pytest.approx(0.6)


@pytest.mark.parametrize('kw', [
    {'gap': 0.05}, {'lost_dist': 0.1}, {'direction': 0}, {'radius': 0}, {'window': 0},
])
def test_params_rejects(kw):
    with pytest.raises(ValueError):
        FollowParams(**kw)
