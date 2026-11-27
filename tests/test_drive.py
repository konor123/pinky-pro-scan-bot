import math

import numpy as np
import pytest

from scan3d.robot.drive import (
    Blocked, Cancelled, Driver, base_from_cam, cam_xy, camera_pose, front_min,
    arc_yaw, in_mask, plan_move, plan_orbit, view_pose,
)
from scan3d.protocol import parse_arc
from scan3d.sim.sim2d import Robot, Sim, SimIO, World
from scan3d.vendor.pinky_commands import parse, wrap_angle


def rig(cfg, role="L", phi=0, noise=None, radius=None, boxes=(), arc=False):
    cfg["drive"]["arc"] = arc
    mount = cfg["robots"][role]["mount"]
    scene = cfg["scene"]
    pose = view_pose(phi, radius or scene["R"], mount)
    world = World(circles=[(0, 0, scene["r_obj"])], boxes=boxes)
    robot = Robot(world, pose, role, odom_noise=noise)
    io = SimIO(Sim(world, [robot]), robot)
    events = []
    driver = Driver(io, cfg, mount, on_event=events.append)
    driver.set_fix(pose, io.odom())
    return driver, io, mount, events


def check_pose(actual, expected, mount, yaw_deg=5.0):
    assert math.dist(cam_xy(actual, mount), cam_xy(expected, mount)) <= 0.025
    assert abs(wrap_angle(actual[2] - expected[2])) <= math.radians(yaw_deg)


def fix(driver, io, rng):
    error = rng.uniform(-1, 1, 3) * [0.002 / math.sqrt(2), 0.002 / math.sqrt(2),
                                      math.radians(0.2)]
    driver.set_fix(tuple(np.array(io.robot.true_pose()) + error), io.odom())


@pytest.mark.parametrize("role,phi", [("L", 25), ("H", -115)])
def test_base_from_cam(cfg, role, phi):
    mount = cfg["robots"][role]["mount"]
    pose = view_pose(phi, 0.62, mount)
    for delta in (0, 5):
        actual = dict(mount, pitch_down_deg=mount["pitch_down_deg"] + delta)
        T = camera_pose(pose, actual)
        board = np.linalg.inv(T)
        a = math.radians(phi)
        assert board[:3, 3] == pytest.approx(
            [0.62 * math.cos(a), 0.62 * math.sin(a), actual["z"]])
        assert board[:3, 0] == pytest.approx([-math.sin(a), math.cos(a), 0])
        pitch = math.radians(actual["pitch_down_deg"])
        assert board[:3, 2] == pytest.approx([-math.cos(pitch)*math.cos(a),
                                              -math.cos(pitch)*math.sin(a), -math.sin(pitch)])
        recovered = base_from_cam(T, mount)
        assert math.dist(pose[:2], recovered[:2]) <= 0.002
        assert abs(wrap_angle(pose[2] - recovered[2])) <= math.radians(0.5)


@pytest.mark.parametrize("role,phi", [("L", 0), ("H", 180)])
def test_pose_compose(cfg, role, phi):
    driver, io, _, _ = rig(cfg, role, phi)
    io.move("turn 45")
    start = io.robot.true_pose()
    driver.set_fix(start, io.odom())
    io.robot.command("forward 1")
    io.sleep(1)
    io.robot.command("stop")
    x, y, yaw = start
    assert driver.pose() == pytest.approx((x + 0.1*math.cos(yaw), y + 0.1*math.sin(yaw), yaw))


def test_plan_move(cfg):
    _, io, mount, _ = rig(cfg)
    start, target = [view_pose(phi, 0.62, mount) for phi in (0, 15)]
    assert plan_move(start, start, 0.06) == []
    near = start[0]+0.01, start[1], start[2]+0.2
    near_moves = plan_move(start, near, 0.06)
    assert len(near_moves) == 1 and parse(near_moves[0])[0].kind == "turn"
    for cmd in plan_move(start, target, 0.06):
        assert all(step.value for step in parse(cmd))
        assert io.move(cmd).ok
    check_pose(io.robot.true_pose(), target, mount)
    skew = (start[0], start[1], start[2] - math.radians(8))
    open_end = plan_move(skew, target, 0.06, final_turn=False)
    assert parse(open_end[-1])[-1].kind == "move" and sum(c.startswith("turn") for c in open_end) == 1
    h = math.radians(phi_deg := 15) + math.pi / 2 - math.radians(7.5)
    on_circle = cam_xy(view_pose(phi_deg, 0.62, mount, yaw=h), mount)
    assert math.hypot(*on_circle) == pytest.approx(0.62, abs=1e-9)
    assert math.degrees(math.atan2(on_circle[1], on_circle[0])) == pytest.approx(15.0, abs=1e-9)
    moves = plan_move((0, 0, 0), (0.51, 0, 0), 0.06)
    distances = [step.value for cmd in moves for step in parse(cmd) if step.kind == "move"]
    assert sum(distances) == pytest.approx(0.51)
    assert max(distances) <= 0.25


@pytest.mark.parametrize("role,phi,scale", [("L", 0, 0.02), ("H", 180, -0.02)])
def test_full_circle(cfg, rng, role, phi, scale):
    driver, io, mount, _ = rig(cfg, role, phi, {"scale": scale, "yaw_drift_deg_per_m": 2})
    for k in range(24):
        sent = len(io.move_log)
        leg = driver.go_to_view(k) if k else driver.enter()
        assert leg.view == k and leg.skipped == []
        actual = io.robot.true_pose()
        check_pose(actual, view_pose(phi+k*15, 0.62, mount), mount, 5.0 if k == 0 else 12.5)
        if k:
            assert sum(cmd.startswith("turn") for _, cmd in io.move_log[sent:]) <= 1
        d = math.hypot(*cam_xy(actual, mount)) - cfg["scene"]["r_obj"] - cfg["scene"]["place_err"]
        assert cfg["drive"]["d_min"] <= d <= cfg["drive"]["d_max"]
        fix(driver, io, rng)


def test_front_min(cfg):
    scan = Robot(World()).scan()
    scan["ranges"][0], scan["ranges"][30] = 0.15, 0.40
    cfg["lidar"]["mask"] = [[-5, 5]]
    assert front_min(scan, 0, cfg) == 0.40
    assert front_min(None, 0, cfg) == front_min(scan, 0.51, cfg) == 0


def test_mask_wrap():
    assert [in_mask(a, [[170, -170]]) for a in (175, -175, 165, 0)] == [True, True, False, False]


def box_between(cfg, k, size=0.10):
    mount = cfg["robots"]["L"]["mount"]
    a, b = [view_pose(15*i, 0.62, mount) for i in (k, k+1)]
    return ((a[0]+b[0])/2, (a[1]+b[1])/2, size, size,
            math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])))


def test_bypass(cfg, rng):
    driver, io, mount, events = rig(cfg, boxes=[box_between(cfg, 2)])
    driver.enter()
    skipped = []
    k = 1
    while k < 24:
        before = io.now()
        leg = driver.go_to_view(k)
        if leg.skipped:
            assert leg.waited_s >= cfg["drive"]["wait_s"]
            assert all(t >= before + cfg["drive"]["wait_s"] for t, cmd in io.move_log
                       if t >= before and "forward" in cmd)
        skipped.extend(leg.skipped)
        assert abs(math.hypot(*cam_xy(io.robot.true_pose(), mount)) - 0.62) <= 0.025
        fix(driver, io, rng)
        k = leg.view + 1
    assert k == 24 and 1 <= len(skipped) <= 2
    assert {"wait", "bypass", "skip"} <= {e["event"] for e in events}


def on_forward(io, monkeypatch, callback, word="forward"):
    move = io.move
    def wrapped(text, watch=None):
        if word in text:
            monkeypatch.setattr(io, "move", move)
            callback()
        return move(text, watch)
    monkeypatch.setattr(io, "move", wrapped)


def test_stop_resume(cfg, rng, monkeypatch):
    driver, io, mount, events = rig(cfg)
    driver.enter()
    driver.go_to_view(1)
    fix(driver, io, rng)
    def add():
        x, y, yaw = io.robot.true_pose()
        box = io.robot.world.add_box(x + 0.17*math.cos(yaw), y + 0.17*math.sin(yaw),
                                     0.04, 0.04, math.degrees(yaw))
        io.sim.at(io.now()+2, lambda: io.robot.world.remove(box))
    on_forward(io, monkeypatch, lambda: io.sim.at(io.now()+1, add))
    leg = driver.go_to_view(2)
    assert leg.stops == 1 and leg.skipped == []
    assert sum(cmd == "stop" for _, cmd in io.move_log) == 1
    assert any(e["event"] == "stop" for e in events)
    check_pose(io.robot.true_pose(), view_pose(30, 0.62, mount), mount, 12.5)


def test_bypass_tail(cfg):
    mount = cfg["robots"]["L"]["mount"]
    inner = view_pose(345, 0.62, mount)
    outer = view_pose(345, 0.87, mount)
    box = (0.8*inner[0]+0.2*outer[0], 0.8*inner[1]+0.2*outer[1], 0.05, 0.05, 345)
    driver, io, _, _ = rig(cfg)
    driver.enter()
    for k in range(1, 23):
        assert driver.go_to_view(k).view == k
        driver.set_fix(io.robot.true_pose(), io.odom())
    io.robot.world.add_box(*box_between(cfg, 22, 0.02))
    io.robot.world.add_box(*box)
    leg = driver.go_to_view(23)
    assert leg.view == 24 and leg.skipped == [23]
    assert abs(math.hypot(*cam_xy(io.robot.true_pose(), mount)) - 0.87) <= 0.025


def test_correct(cfg):
    driver, io, mount, _ = rig(cfg)
    driver.enter()
    pose = view_pose(45, 0.47, mount)
    driver.set_fix(pose, io.odom())
    assert math.hypot(*cam_xy(driver.pose(), mount)) - 0.17 == pytest.approx(0.30)
    assert driver.correct(3).view == 3
    assert math.hypot(*cam_xy(driver.pose(), mount)) - 0.17 >= cfg["drive"]["d_min"]


@pytest.mark.parametrize("moving", [False, True])
def test_stale_scan(cfg, monkeypatch, moving):
    driver, io, _, events = rig(cfg)
    driver.enter()
    if moving:
        on_forward(io, monkeypatch, lambda: setattr(io, "drop_scans", True))
    else:
        io.drop_scans = True
        io.sleep(0.55)
    with pytest.raises(Blocked):
        driver.go_to_view(1)
    assert io.now() >= cfg["drive"]["block_timeout_s"]
    assert sum(cmd == "stop" for _, cmd in io.move_log) == int(moving)
    assert sum("forward" in cmd for _, cmd in io.move_log) == int(moving)
    assert any(e["event"] == "blocked" for e in events)


def test_cancel(cfg):
    driver, io, _, _ = rig(cfg)
    driver.enter()
    io.drop_scans = True
    io.sleep(1)
    io.sim.at(io.now()+2, io.cancel)
    with pytest.raises(Cancelled):
        driver.go_to_view(1)
    assert all(t < 3 and "forward" not in cmd for t, cmd in io.move_log)


def arc_pose(phi, mount, radius=0.62):
    return view_pose(phi, radius, mount, yaw=arc_yaw(phi, radius, mount))


def test_arc_pose(cfg):
    for role in ("L", "H"):
        mount = cfg["robots"][role]["mount"]
        x, y, yaw = arc_pose(40, mount)
        cx, cy = cam_xy((x, y, yaw), mount)
        assert math.hypot(cx, cy) == pytest.approx(0.62) and math.degrees(math.atan2(cy, cx)) == pytest.approx(40)
        # 바퀴 축의 연장선(로봇 왼쪽)이 판 중심을 지난다 = 로봇 방향이 기준점 원의 접선
        assert x*math.cos(yaw) + y*math.sin(yaw) == pytest.approx(0, abs=1e-9)
        assert -x*math.sin(yaw) + y*math.cos(yaw) < 0
        assert math.degrees(wrap_angle(yaw - view_pose(40, 0.62, mount)[2])) == pytest.approx(
            math.degrees(math.asin(-mount["x"] / 0.62)), abs=1e-6)


def test_plan_orbit(cfg):
    _, io, mount, _ = rig(cfg, arc=True)
    start, target = [arc_pose(phi, mount) for phi in (0, 15)]
    io.robot._pose = start
    base_radius = math.hypot(*start[:2])
    assert base_radius == pytest.approx(math.sqrt(0.62**2 - mount["x"]**2))
    assert plan_orbit(start, start, 0.1) == []
    # 원 위에 제 방향으로 서 있으면 판 중심 둘레의 원호 하나
    (cmd,) = plan_orbit(start, target, 0.1)
    speed, radius, angle = parse_arc(cmd)
    assert speed == 0.1 and radius == pytest.approx(base_radius, abs=0.001) and angle == pytest.approx(15, abs=0.01)
    assert io.move(cmd).ok
    actual = io.robot.true_pose()
    assert math.dist(actual[:2], target[:2]) <= 0.005
    assert abs(wrap_angle(actual[2] - target[2])) <= math.radians(0.5)
    # 벗어난 만큼 휨을 바꾼다: 바깥·오른쪽으로 틀어져 있으면 더 휘고, 안쪽·왼쪽이면 덜 휜다. 회전 문장은 없다.
    def bend(pose):
        (cmd,) = plan_orbit(pose, target, 0.1)
        _, radius, angle = parse_arc(cmd)
        assert radius * math.radians(angle) == pytest.approx(base_radius * math.radians(15), abs=0.003)
        return 1 / radius
    out = (start[0] * 1.016, start[1] * 1.016, start[2])          # 약 1 cm 바깥
    assert bend(out) > 1/base_radius > bend((start[0] / 1.016, start[1] / 1.016, start[2]))
    assert bend((start[0], start[1], start[2] - math.radians(4))) > 1/base_radius
    assert bend((start[0], start[1], start[2] + math.radians(4))) < 1/base_radius
    # 한 번에 다 고치지 않는다: 1 cm 바깥에서 출발하면 조금 줄고 방향이 안쪽으로 든 채 도착한다(다음 원호들이 마저 줄인다)
    io.robot._pose = out
    assert io.move(plan_orbit(out, target, 0.1)[0]).ok
    x, y, yaw = io.robot.true_pose()
    assert 0.006 <= math.hypot(x, y) - base_radius <= 0.0095
    assert wrap_angle(yaw - (math.atan2(y, x) + math.pi/2)) > math.radians(1)
    # 원에서 크게 벗어났거나 목표가 뒤쪽이면 회전 + 직진
    far = (start[0] * 1.1, start[1] * 1.1, start[2])
    assert plan_orbit(far, target, 0.1) == plan_move(far, target, 0.1)
    twisted = (start[0], start[1], start[2] + math.radians(25))
    assert plan_orbit(twisted, target, 0.1) == plan_move(twisted, target, 0.1)
    assert plan_orbit(target, start, 0.1) == plan_move(target, start, 0.1)


@pytest.mark.parametrize("noise_mm,noise_deg", [(2, 0.2), (10, 1.0)])
def test_orbit_fix_noise(cfg, noise_mm, noise_deg):
    """표식 자세가 몇 mm·1°쯤 흔들려도 시점 이동에 회전이 끼지 않고 방향이 고르게 유지된다."""
    rng = np.random.default_rng(1)
    driver, io, mount, _ = rig(cfg, noise={"scale": 0.02, "yaw_drift_deg_per_m": 2}, arc=True)
    driver.enter()
    sent = len(io.move_log)
    for k in range(1, 24):
        error = rng.uniform(-1, 1, 3) * [noise_mm/1000/math.sqrt(2), noise_mm/1000/math.sqrt(2),
                                          math.radians(noise_deg)]
        driver.set_fix(tuple(np.array(io.robot.true_pose()) + error), io.odom())
        assert driver.go_to_view(k).view == k
        check_pose(io.robot.true_pose(), arc_pose(k*15, mount), mount, 5.0)
    texts = [cmd for _, cmd in io.move_log[sent:]]
    assert len(texts) == 23 and all("arc" in cmd for cmd in texts)


@pytest.mark.parametrize("role,phi,scale", [("L", 0, 0.02), ("H", 180, -0.02)])
def test_full_circle_arc(cfg, rng, role, phi, scale):
    driver, io, mount, _ = rig(cfg, role, phi, {"scale": scale, "yaw_drift_deg_per_m": 2}, arc=True)
    errors = []
    for k in range(24):
        sent = len(io.move_log)
        leg = driver.go_to_view(k) if k else driver.enter()
        assert leg.view == k and leg.skipped == []
        actual = io.robot.true_pose()
        expected = arc_pose(phi+k*15, mount)
        check_pose(actual, expected, mount, 5.0)          # 원호 자세(바퀴 축이 판 중심을 향함)로 도착한다
        errors.append(abs(math.degrees(wrap_angle(actual[2] - expected[2]))))
        if k:
            texts = [cmd for _, cmd in io.move_log[sent:]]
            assert sum("arc" in cmd for cmd in texts) == 1 and not any("forward" in cmd for cmd in texts)
        d = math.hypot(*cam_xy(actual, mount)) - cfg["scene"]["r_obj"] - cfg["scene"]["place_err"]
        assert cfg["drive"]["d_min"] <= d <= cfg["drive"]["d_max"]
        fix(driver, io, rng)
    # 방향 오차가 줄어들어 머문다. 회전 문장은 진입 때뿐이다.
    assert max(errors[6:]) <= 2.0
    assert sum(cmd.startswith("turn") for _, cmd in io.move_log) <= 2


def test_bypass_arc(cfg, rng):
    driver, io, mount, events = rig(cfg, boxes=[box_between(cfg, 2)], arc=True)
    driver.enter()
    skipped = []
    k = 1
    while k < 24:
        leg = driver.go_to_view(k)
        if leg.skipped:
            assert leg.waited_s >= cfg["drive"]["wait_s"]
        skipped.extend(leg.skipped)
        assert abs(math.hypot(*cam_xy(io.robot.true_pose(), mount)) - 0.62) <= 0.025
        fix(driver, io, rng)
        k = leg.view + 1
    assert k == 24 and 1 <= len(skipped) <= 2
    assert {"wait", "bypass", "skip"} <= {e["event"] for e in events}


def test_go_to_phi_long_arc(cfg):
    """원 위에 원호 자세로 서 있으면 먼 각도 원호 하나로 간다(보강 촬영 이동)."""
    driver, io, mount, _ = rig(cfg, noise={"scale": 0.02, "yaw_drift_deg_per_m": 2}, arc=True)
    driver.enter()                                     # 원호 자세로
    assert driver.on_orbit()
    start = len(io.move_log)
    driver.go_to_phi(90.0)
    sent = [cmd for _, cmd in io.move_log[start:]]
    assert len(sent) == 1 and parse_arc(sent[0])[2] == pytest.approx(90, abs=3), sent
    actual, expected = io.robot.true_pose(), arc_pose(90, mount)      # 바퀴 오차 2 %·2°/m로 1 m: 도착 뒤 표식으로 다시 잡는다
    assert math.dist(cam_xy(actual, mount), cam_xy(expected, mount)) <= 0.06
    assert abs(wrap_angle(actual[2] - expected[2])) <= math.radians(5.0)
    driver.set_fix(actual, io.odom())
    assert driver.on_orbit()                                           # 그 자리에서 다음 원호로 떠날 수 있다
    x, y, yaw = io.robot.true_pose()
    driver.set_fix((x, y, yaw + math.radians(25)), io.odom())      # 접선에서 25° 틀어짐
    assert not driver.on_orbit()
    plain, _, _, _ = rig(cfg, arc=False)
    assert not plain.on_orbit()


def test_stop_resume_arc(cfg, rng, monkeypatch):
    driver, io, mount, events = rig(cfg, arc=True)
    driver.enter()
    driver.go_to_view(1)
    fix(driver, io, rng)
    def add():
        x, y, yaw = io.robot.true_pose()
        box = io.robot.world.add_box(x + 0.17*math.cos(yaw), y + 0.17*math.sin(yaw),
                                     0.04, 0.04, math.degrees(yaw))
        io.sim.at(io.now()+2, lambda: io.robot.world.remove(box))
    on_forward(io, monkeypatch, lambda: io.sim.at(io.now()+0.5, add), word="arc")
    leg = driver.go_to_view(2)
    assert leg.stops == 1 and leg.skipped == []
    assert sum(cmd == "stop" for _, cmd in io.move_log) == 1
    assert any(e["event"] == "stop" for e in events)
    check_pose(io.robot.true_pose(), arc_pose(30, mount), mount, 5.0)
