"""장착값 맞춤(docs/specs/mountfit.md): 원호 앞뒤 표식 자세 → 회전 중심 → 카메라 장착 위치·방향."""
import math

import numpy as np
import pytest

from scan3d.protocol import parse_arc
from scan3d.robot import mountfit
from scan3d.robot.drive import Driver, base_from_cam, cam_xy, camera_pose, view_pose
from scan3d.sim.sim2d import Robot, Sim, SimIO, World

NOMINAL = {"x": -0.08, "y": 0.0, "z": 0.45, "pitch_down_deg": 37, "yaw_deg": 90}
TRUE_H = {"x": 0.02, "y": 0.07, "z": 0.45, "pitch_down_deg": 37, "yaw_deg": 86}     # 실물 H에서 잰 값
TRUE_L = {"x": -0.08, "y": 0.0, "z": 0.15, "pitch_down_deg": 15, "yaw_deg": 94.5}


def arc_pose(pose, radius, angle_deg):
    """기준점이 왼쪽으로 radius(음수 각이면 오른쪽) 떨어진 점을 중심으로 angle_deg 돈 뒤의 자세."""
    x, y, yaw = pose
    a = math.radians(angle_deg)
    s = math.copysign(1.0, a)
    cx, cy = x - s*radius*math.sin(yaw), y + s*radius*math.cos(yaw)
    dx, dy = x - cx, y - cy
    return (cx + dx*math.cos(a) - dy*math.sin(a), cy + dx*math.sin(a) + dy*math.cos(a), yaw + a)


def seen(pose, radius, angle_deg, mount, noise=None):
    T0 = camera_pose(pose, mount)
    T1 = camera_pose(arc_pose(pose, radius, angle_deg), mount)
    if noise is not None:
        T1 = T1.copy()
        T1[:3, 3] += noise
    return mountfit.arc_centre(T0, T1)


@pytest.mark.parametrize("mount", [NOMINAL, TRUE_H, TRUE_L])
def test_arc_centre(mount):
    """회전 중심은 장착값과 반지름으로 정해지는 카메라 기준의 한 점이다(로봇이 어디 있든)."""
    for pose, radius, angle in [((0.5, 0.1, 0.3), 0.6, 15), ((-0.2, 0.7, 2.0), 0.35, 40), ((0.1, -0.6, -1.0), 0.9, -20)]:
        q, turn = seen(pose, radius, angle, mount)
        assert math.degrees(turn) == pytest.approx(angle, abs=1e-6)
        assert q == pytest.approx(mountfit.predict(mount, math.copysign(radius, angle)), abs=1e-9)
    assert seen((0.5, 0.1, 0.3), 0.6, 2.0, mount) is None            # 덜 돈 원호


def test_fit_recovers_mount():
    """반지름이 다른 원호가 쌓이면 틀린 처음 값에서 실제 위치와 방향이 나온다."""
    rng = np.random.default_rng(1)
    fit = mountfit.MountFit(dict(NOMINAL))
    for radius in (0.56, 0.47, 0.84, 0.42, 0.76, 0.41, 0.87, 0.52):
        q, _ = seen((0.6, 0.0, 1.6), radius, 15, TRUE_H, noise=rng.normal(0, 0.001, 3))
        assert fit.add(q, radius)
    m = fit.mount()
    assert m["x"] == pytest.approx(TRUE_H["x"], abs=0.01) and m["y"] == pytest.approx(TRUE_H["y"], abs=0.01)
    assert m["yaw_deg"] == pytest.approx(TRUE_H["yaw_deg"], abs=1.5)
    assert m["z"] == NOMINAL["z"] and m["pitch_down_deg"] == NOMINAL["pitch_down_deg"]   # 다른 값은 그대로


def test_fit_one_radius_keeps_yaw():
    """반지름이 같은 원호뿐이면 yaw는 처음 값에 두고, 그 반지름의 회전 중심은 맞춘다."""
    fit = mountfit.MountFit(dict(NOMINAL))
    q, _ = seen((0.6, 0.0, 1.6), 0.69, 15, TRUE_H)
    assert fit.add(q, 0.69)
    m = fit.mount()
    assert m["yaw_deg"] == pytest.approx(NOMINAL["yaw_deg"], abs=0.5)
    assert mountfit.predict(m, 0.69) == pytest.approx(q, abs=1e-6)
    assert mountfit.predict(m, 0.65) == pytest.approx(mountfit.predict(TRUE_H, 0.65), abs=0.005)   # 가까운 반지름도


def test_observe_gates():
    T0 = camera_pose((0.6, 0.0, 1.6), TRUE_H)
    T1 = camera_pose(arc_pose((0.6, 0.0, 1.6), 0.6, 15), TRUE_H)
    q, r = mountfit.observe(T0, T1, 0.2, 0.2 + math.radians(15.5), 0.6, 15.0)
    assert r == 0.6 and q == pytest.approx(mountfit.predict(TRUE_H, 0.6), abs=1e-9)
    assert mountfit.observe(T0, T1, 0.2, 0.2 + math.radians(19.0), 0.6, 15.0) is None    # odom은 19° 돌았다고 함
    assert mountfit.observe(T0, T1, 0.2, 0.2 + math.radians(15.0), 0.6, -15.0) is None   # 명령과 반대로 돎
    T2 = camera_pose(arc_pose((0.6, 0.0, 1.6), 0.6, -15), TRUE_H)
    assert mountfit.observe(T0, T2, 0.2, 0.2 - math.radians(15.0), 0.6, -15.0)[1] == -0.6


def test_fit_rejects():
    fit = mountfit.MountFit(dict(NOMINAL))
    assert not fit.add([float("nan"), 0.0], 0.6)
    assert not fit.add([0.9, 0.9], 0.6)                              # 장착 위치가 한도 밖으로 나오는 관측
    assert fit.mount() == NOMINAL
    for radius in (0.5, 0.6, 0.7, 0.8):
        assert fit.add(mountfit.predict(TRUE_H, radius), radius)
    good = fit.mount()
    fit.add(mountfit.predict(TRUE_H, 0.65) + [0.15, 0.0], 0.65)      # 튄 관측 하나는 뺀다
    assert fit.mount()["x"] == pytest.approx(good["x"], abs=0.003) and fit.n == 4


def test_remember():
    mountfit.forget()
    assert mountfit.recall("S1") is None
    mountfit.remember("S1", TRUE_H)
    assert mountfit.recall("S1") == {"x": 0.02, "y": 0.07, "yaw_deg": 86.0}
    mountfit.forget()


@pytest.mark.parametrize("role,true", [("H", TRUE_H), ("L", TRUE_L)])
def test_lap_from_wrong_mount(cfg, role, true):
    """config에는 처음 값만 있고 실제 장착은 다를 때: 원호마다 표식으로 맞춰 가며 한 바퀴.

    표식 자세는 실제 장착으로 만든다(스캔 PC는 그 자세와 보낸 원호 문장만 안다).
    """
    cfg["drive"]["arc"] = True
    R = cfg["scene"]["R"]
    guess = dict(cfg["robots"][role]["mount"], x=-0.08, y=0.0, yaw_deg=90)
    rng = np.random.default_rng(5)
    world = World(circles=[(0, 0, cfg["scene"]["r_obj"])])
    robot = Robot(world, view_pose(40.0, R, true), role, odom_noise={"scale": 0.02, "yaw_drift_deg_per_m": 2})
    io = SimIO(Sim(world, [robot]), robot)
    driver = Driver(io, cfg, dict(guess))
    fit = mountfit.MountFit(dict(guess))
    last = None

    def fix():
        nonlocal last
        true_pose = np.array(io.robot.true_pose()) + rng.uniform(-1, 1, 3) * [0.002, 0.002, math.radians(0.2)]
        T = camera_pose(tuple(true_pose), true)
        sent = driver.take_sent()
        if last is not None and len(sent) == 1 and not sent[0][1] and parse_arc(sent[0][0]):
            _, radius, angle = parse_arc(sent[0][0])
            obs = mountfit.observe(last[0], T, last[1], io.odom()[2], radius, angle)
            if obs is not None and fit.add(*obs):
                driver.mount = fit.mount()
        last = (T, io.odom()[2])
        driver.set_fix(base_from_cam(T, driver.mount), io.odom())

    fix()
    driver.enter()
    off, turns = [], []
    for k in range(1, 25):
        fix()
        mark = len(io.move_log)
        driver.go_to_view(k)
        turns.append(sum("turn" in c for _, c in io.move_log[mark:]))
        off.append(math.hypot(*cam_xy(io.robot.true_pose(), true)) - R)
    assert max(abs(e) for e in off) <= 0.06, off                     # 처음 두세 원호는 처음 값의 오차만큼 벗어난다
    assert max(abs(e) for e in off[6:]) <= 0.012, off                # 그 뒤로는 카메라가 원 위 1.2 cm 안
    assert sum(turns[4:]) == 0, turns                                # 제자리 회전 없이 원호로만
    m = fit.mount()
    q_true, q_fit = mountfit.predict(true, 0.65), mountfit.predict(m, 0.65)
    assert np.linalg.norm(q_true - q_fit) <= 0.015, (m, true)        # 도는 중심을 1.5 cm 안으로 안다


@pytest.mark.slow
def test_scan_with_wrong_mount(cfg, tmp_path):
    """가짜 로봇 H의 카메라가 config와 다른 자리에 달려 있어도 한 바퀴: 처음 몇 시점 뒤로 거리가 목표에 머문다."""
    import copy
    import json

    import yaml

    from scan3d.clock import Clock
    from scan3d.sim.fake_agent import FakeFleet
    from scan3d.station import execute

    cfg["capture"].update({"color": {"w": 1280, "h": 720, "fmt": "yuyv", "fps": 15},
                           "depth": {"w": 424, "h": 240, "fmt": "z16", "fps": 15}})
    true_cfg = copy.deepcopy(cfg)
    true_cfg["robots"]["H"]["mount"].update(x=TRUE_H["x"], y=TRUE_H["y"], yaw_deg=TRUE_H["yaw_deg"])
    cfg_path = tmp_path / "scan.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    clock = Clock(5.0)
    fleet = FakeFleet(true_cfg, clock, roles=("H",))
    try:
        code, scan_dir = execute(cfg, cfg_path, fleet.start(), clock, ("H",), no_window=True,
                                 data_dir=tmp_path / "scans")
    finally:
        fleet.stop()
    assert code == 0
    events = [json.loads(line) for line in (scan_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    fits = [e for e in events if e["type"] == "mount"]
    assert len(fits) >= 15
    last = fits[-1]
    q_fit = mountfit.predict(last, 0.69)
    assert np.linalg.norm(q_fit - mountfit.predict(TRUE_H, 0.69)) <= 0.02, last
    d = {e["k"]: e["d"] for e in events if e["type"] == "view" and e["pose"]}
    target = cfg["drive"]["d_target"]
    assert all(abs(d[k] - target) <= 0.015 for k in range(8, 24)), d
    assert sum(1 for e in events if e["type"] == "correct") <= 1
