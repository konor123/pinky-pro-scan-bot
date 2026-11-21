"""현장 도구(tools/)를 가짜 에이전트에 붙여 돌린다."""
import json

import pytest
import yaml

from scan3d import station
from scan3d.clock import Clock
from scan3d.sim.fake_agent import FakeFleet
from tools import drive_check, led_check, lidar_view, show_pose, snapshot_test

SMALL = {"color": {"w": 1280, "h": 720, "fmt": "yuyv", "fps": 15},
         "depth": {"w": 424, "h": 240, "fmt": "z16", "fps": 15}}


@pytest.fixture
def site(cfg, tmp_path):
    cfg["capture"].update(json.loads(json.dumps(SMALL)))
    cfg["station"]["data_dir"] = str(tmp_path / "scans")
    fleet = FakeFleet(cfg, Clock(1.0))
    addrs = fleet.start()
    for role, (host, port, sport) in addrs.items():
        cfg["robots"][role].update(host=host, port=port, stream_port=sport)
    path = tmp_path / "scan.yaml"
    path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    yield str(path), tmp_path, fleet
    fleet.stop()


def test_hello(site, capsys):
    path, _, _ = site
    assert station.main(["--config", path, "--hello"]) == 0
    out = capsys.readouterr().out
    assert "serial FAKE-L" in out and "serial FAKE-H" in out


def test_snapshot_test(site, capsys):
    path, tmp, _ = site
    assert snapshot_test.main(["--robot", "H", "--count", "2", "--save", "--config", path]) == 0
    assert "PASS 2/2" in capsys.readouterr().out
    raws = list((tmp / "scans").glob("*/H/raw/000001_depth.z16"))
    assert len(raws) == 1


def test_show_pose(site, capsys):
    path, _, _ = site
    assert show_pose.main(["--robot", "L", "--config", path]) == 0
    out = capsys.readouterr().out
    assert "phi" in out and "로봇 기준점" in out


def test_lidar_view(site, capsys):
    path, _, _ = site
    assert lidar_view.main(["--robot", "L", "--mask", "--seconds", "1.5", "--config", path]) == 0
    out = capsys.readouterr().out
    assert "lidar.mask: []" in out and "PASS" in out


@pytest.mark.slow
def test_drive_check_odom(site, capsys):
    path, _, _ = site
    assert drive_check.main(["--robot", "L", "--odom-only", "--config", path]) == 0
    out = capsys.readouterr().out
    assert out.count("PASS") == 3


@pytest.mark.slow
def test_drive_check_tags(site, capsys):
    path, _, _ = site
    assert drive_check.main(["--robot", "H", "--config", path]) == 0
    out = capsys.readouterr().out
    assert out.count("PASS") == 3


def test_led_check(capsys):
    fake = "class _Led:\n    def fill(self, rgb): pass\n    def clear(self): pass\nled = _Led()"
    code = led_check.ROBOT_CODE.format(led=fake)
    spawn = lambda role, host: [__import__("sys").executable, "-u", "-c", code]  # noqa: E731
    assert led_check.main(["--cycles", "1", "--hold", "0"], spawn=spawn) == 0
    out = capsys.readouterr().out
    assert "L(" in out and "H(" in out and "명령 3개" in out and "두 대 응답 차" in out and "PASS" in out
