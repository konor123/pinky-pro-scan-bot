import math

import pytest

from scan3d.sim.sim2d import Robot, Sim, SimIO, World
from scan3d.vendor.pinky_commands import sector_min


@pytest.fixture
def robot():
    return Robot(World(), (0, 0, 0), "L")


def run(robot, text, seconds):
    robot.command(text)
    for _ in range(round(seconds * 20)):
        robot.step(0.05)
    return robot.statuses()


def test_command_status(robot):
    text = "speed 0.06; turn 7.5; forward 0.17; turn 7.5"
    assert run(robot, text, 10) == [
        f"accepted: {text} (대기 4단계)", "speed: 0.06 m/s", "done: turn +7 deg",
        "done: move +0.17 m", "done: turn +7 deg (모든 명령 완료)",
    ]
    assert robot.statuses() == []


def test_stop_inside_sentence(robot):
    """문장 중간의 stop은 남은 명령을 지우고 멈춘다(맨 앞 stop과 같게). 영원히 busy로 남지 않는다."""
    msgs = run(robot, "forward 0.1; stop; forward 0.1", 5)
    assert msgs[-1] == "stopped: 남은 명령 취소" and not robot.busy()
    assert math.isclose(robot.odom()[0], 0.1, abs_tol=0.015)


def test_done_suffix_after_trailing_speed(robot):
    """뒤에 speed만 남았어도 마지막 동작에 (모든 명령 완료)를 붙인다."""
    msgs = run(robot, "forward 0.1; speed 0.1", 5)
    assert "done: move +0.10 m (모든 명령 완료)" in msgs and not robot.busy()


def test_motion(robot):
    for angle in [7.5, -30]:
        start = robot.true_pose()[2]
        run(robot, f"turn {angle}", 5)
        turned = math.degrees(robot.true_pose()[2] - start)
        assert turned * angle > 0
        assert 0 <= abs(angle) - abs(turned) <= 2
    start = robot.true_pose()[:2]
    run(robot, "speed 0.06; forward 0.17", 5)
    assert 0 <= 0.17 - math.dist(start, robot.true_pose()[:2]) <= 0.01


def test_speed_only(robot):
    assert run(robot, "speed 0.06", 2) == [
        "accepted: speed 0.06 (대기 1단계)", "speed: 0.06 m/s",
    ]


def test_stop(robot):
    run(robot, "forward 1; turn 90", 1)
    stopped = robot.true_pose()
    assert stopped[0] > 0
    assert run(robot, "stop", 0.05) == ["stopped: 남은 명령 취소"]
    assert robot.true_pose() == stopped
    robot.step(3)
    assert robot.true_pose() == stopped


def test_bad_text(robot):
    assert run(robot, "turn 0", 1)[0].startswith("error:")
    assert robot.true_pose() == (0, 0, 0)


def front(robot):
    scan = robot.scan()
    return sector_min(scan["ranges"], scan["angle_min"], scan["angle_inc"],
                      0, math.radians(25), math.pi, scan["range_min"])


def test_scan_front(robot):
    robot.world.add_box(0.55, 0, 0.1, 0.1, 0)
    assert front(robot) == pytest.approx(0.517, abs=0.01)


def test_other_robot(robot):
    other = Robot(robot.world, (1, 0, math.pi), "H")
    for observer in (robot, other):
        assert front(observer) == pytest.approx(1 + 0.017 - 0.07)


def test_collision_and_events(robot):
    box = robot.world.add_box(0.2, 0, 0.1, 0.1, 45)
    sim = Sim(robot.world, [robot])
    robot.command("forward 0.35")
    sim.run_until(lambda: False, 5)
    assert 0 < robot.true_pose()[0] < 0.07
    assert not any(s.startswith("done:") for s in robot.statuses())
    sim.at(sim.now + 0.2, lambda: robot.world.remove(box))
    assert sim.run_until(lambda: robot.true_pose()[0] >= 0.34, 10)


def test_move_timeout(robot):
    io = SimIO(Sim(robot.world, [robot]), robot)
    result = io.move("speed 0.06", timeout_s=0.25)
    assert not result.ok
    stamp, text = io.move_log[-1]
    assert stamp == pytest.approx(0.25) and text == "stop"
    assert robot.true_pose() == (0, 0, 0)


def test_arc(robot):
    # 원 중심 (0, 0.62) 둘레를 반시계로 15°: 로봇은 원 위에 남고 방향이 15° 돈다.
    text = "speed 0.10; arc 0.62 15"
    statuses = run(robot, text, 5)
    assert statuses[0] == f"accepted: {text} (대기 2단계)"
    assert statuses[-1] == "done: arc 0.620 m +15.0 deg (모든 명령 완료)"
    x, y, yaw = robot.true_pose()
    assert math.hypot(x, y - 0.62) == pytest.approx(0.62, abs=0.002)
    moved = math.atan2(x, 0.62 - y)
    assert 0 <= 0.62 * (math.radians(15) - moved) <= 0.01
    assert yaw == pytest.approx(moved, abs=math.radians(0.5))
    assert robot.odom() == pytest.approx(robot.true_pose(), abs=1e-6)


def test_arc_clockwise_and_stop(robot):
    run(robot, "arc 0.5 -30", 5)
    x, y, yaw = robot.true_pose()
    assert yaw < 0 and y < 0
    assert math.hypot(x, y + 0.5) == pytest.approx(0.5, abs=0.002)
    start = robot.true_pose()
    robot.command("arc 0.62 90")
    for _ in range(20):
        robot.step(0.05)
    robot.command("stop")
    robot.statuses()
    here = robot.true_pose()
    assert math.dist(start[:2], here[:2]) > 0.05
    for _ in range(20):
        robot.step(0.05)
    assert robot.true_pose() == here and not robot.busy()


@pytest.mark.parametrize("text", ["arc 0.62", "arc 0.01 15", "arc 0.62 0", "turn 5; arc 0.62 15",
                                  "speed 0.5; arc 0.62 15"])
def test_arc_bad_text(robot, text):
    assert run(robot, text, 1)[0].startswith("error:")
    assert robot.true_pose() == (0, 0, 0)
