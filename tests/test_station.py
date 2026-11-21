import json
import math
from pathlib import Path

import pytest
import yaml

from scan3d.robot import drive

from scan3d.sim import synth
from scan3d.sim.fake_agent import FakeFleet
from scan3d.clock import Clock
from scan3d.station import execute
from scan3d.vendor.pinky_commands import wrap_angle

SCALE = 5.0
SMALL = {"color": {"w": 640, "h": 360, "fmt": "yuyv", "fps": 15},
         "depth": {"w": 424, "h": 240, "fmt": "z16", "fps": 15}}


class Run:
    def __init__(self, code, scan_dir, fleet, clock):
        self.code, self.fleet, self.clock = code, fleet, clock
        self.dir = Path(scan_dir)

    def events(self, role=None, typ=None):
        out = []
        for line in (self.dir / "events.jsonl").read_text(encoding="utf-8").splitlines():
            ev = json.loads(line)
            if (role is None or ev.get("role") == role) and (typ is None or ev.get("type") == typ):
                out.append(ev)
        return out

    def report(self):
        return json.loads((self.dir / "report.json").read_text(encoding="utf-8"))

    def raw_heads(self, role):
        heads = []
        for p in sorted((self.dir / role / "raw").glob("[0-9]*.json")):
            heads.append(json.loads(p.read_text(encoding="utf-8")))
        return heads

    def view_ks(self, role):
        return sorted(h["station"]["k"] for h in self.raw_heads(role) if h["station"]["use"] == "view")

    def views(self, role):
        return {v["k"]: v for v in json.loads((self.dir / role / "views.json").read_text(encoding="utf-8"))}


def scan(cfg, tmp_path, roles=("L", "H"), small=True, swap=False, prepare=None, **fleet_kw):
    if small:
        cfg["capture"].update(json.loads(json.dumps(SMALL)))
    cfg_path = tmp_path / "scan.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    clock = Clock(SCALE)
    fleet = FakeFleet(cfg, clock, roles=roles, **fleet_kw)
    addrs = fleet.start()
    if swap:
        addrs = {"L": addrs["H"], "H": addrs["L"]}
    if prepare:
        prepare(fleet)
    try:
        code, scan_dir = execute(cfg, cfg_path, addrs, clock, roles, no_window=True,
                                 data_dir=tmp_path / "scans")
    except BaseException:
        fleet.stop()
        raise
    run = Run(code, scan_dir, fleet, clock)
    return run


@pytest.fixture
def runs():
    made = []
    yield made
    for r in made:
        r.fleet.stop()


def gaps(run):
    vl, vh = run.views("L"), run.views("H")
    return {k: abs(math.degrees(wrap_angle(math.radians(vh[k]["phi"] - vl[k]["phi"]))))
            for k in vl if k in vh}


@pytest.mark.slow
def test_sequence(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path)
    runs.append(run)
    f = run.fleet
    assert run.code == 0
    for role in ("L", "H"):
        t_hello = min(t for r, t in f.hello_log if r == role)
        assert all(t > t_hello for r, _, t in f.move_log if r == role)
        assert all(t >= t_hello for r, _, _, t in f.snapshot_log if r == role)
        assert run.view_ks(role) == list(range(24))
    spans = sorted((a, b) for _, a, b, _ in f.snapshot_log)
    assert all(b0 <= a1 for (_, b0), (a1, _) in zip(spans, spans[1:]))
    g = gaps(run)
    assert len(g) == 24 and all(170 <= v <= 190 for v in g.values()), g


@pytest.mark.slow
def test_gap_kept(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path, starts={"L": 0, "H": 160})
    runs.append(run)
    assert run.code == 0
    start = {e["role"]: e["phi"] for e in run.events(typ="start")}
    g0 = abs(math.degrees(wrap_angle(math.radians(start["H"] - start["L"]))))
    g = gaps(run)
    assert len(g) == 24 and all(abs(v - g0) <= 10 for v in g.values()), (g0, g)


@pytest.mark.slow
def test_bypass_waits(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path, obstacle=(2, 0.10))
    runs.append(run)
    f = run.fleet
    assert run.code == 0
    legs = {r: run.events(r, "leg") for r in ("L", "H")}
    for k in range(23):
        l_done = min(e["pc_t"] for e in legs["L"] if e["view"] >= k)
        h_end = [e["pc_t"] for e in legs["H"] if e["view"] == k]
        if not h_end:
            continue
        later = [t for r, _, t in f.move_log if r == "H" and t > h_end[0]]
        if later:
            assert min(later) > l_done, k
    kinds = {e["event"] for e in run.events("L", "drive")}
    assert {"wait", "bypass"} <= kinds
    rep = run.report()
    assert rep["aborted"] is None
    assert 1 <= len(rep["skipped"]["L"]) <= 2 and set(rep["skipped"]["L"]) <= {2, 3, 4}
    assert 1 <= len(rep["skipped"]["H"]) <= 2 and set(rep["skipped"]["H"]) <= {14, 15, 16}
    for role in ("L", "H"):
        assert run.view_ks(role) == sorted(set(range(24)) - set(rep["skipped"][role]))
        skips = sorted(e["k"] for e in run.events(role, "skip"))
        assert skips == sorted(rep["skipped"][role])
        assert rep["stops"][role] == sum(e["stops"] for e in legs[role])
        assert rep["corrections"][role] == len(run.events(role, "correct"))


def _no_start(run):
    assert run.code == 1
    assert run.fleet.move_log == []


def test_pair_gap(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path, starts={"L": 0, "H": 120})
    runs.append(run)
    _no_start(run)


def test_no_tags_start(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path, starts={"L": (0, 180), "H": 180})
    runs.append(run)
    _no_start(run)


@pytest.mark.parametrize("case", ["role", "serial"])
def test_role_serial(cfg, tmp_path, runs, case):
    if case == "role":
        run = scan(cfg, tmp_path, swap=True)
    else:
        run = scan(cfg, tmp_path, serials={"L": "SAME", "H": "SAME"})
    runs.append(run)
    _no_start(run)


@pytest.mark.parametrize("after,nth", [("L_moving", 10), ("L_waiting", 11)])
def test_failure_stops(cfg, tmp_path, runs, after, nth):
    run = scan(cfg, tmp_path, fail={"role": "H", "cmd": "move", "nth": nth, "after": after})
    runs.append(run)
    f = run.fleet
    assert run.code == 1
    (role, t_fail, l_busy), = f.fail_log
    assert l_busy == (after == "L_moving")
    aborts = {r: t for r, what, t in f.stop_log if what == "abort"}
    assert set(aborts) == {"L", "H"} and all(t - t_fail < 2.0 for t in aborts.values())
    assert all(t <= t_fail + 2.0 for _, _, t in f.move_log)
    before = {r: f.true_pose(r) for r in ("L", "H")}
    run.clock.sleep(2.0)
    for r in ("L", "H"):
        assert math.dist(before[r][:2], f.true_pose(r)[:2]) < 1e-3
    assert (run.dir / "L" / "raw" / "calib.json").exists()
    assert run.report()["aborted"]


@pytest.mark.slow
def test_only(cfg, tmp_path, runs):
    run = scan(cfg, tmp_path, roles=("L",))
    runs.append(run)
    assert run.code == 0
    assert run.view_ks("L") == list(range(24))
    assert not (run.dir / "H").exists() or not list((run.dir / "H").glob("raw/*.json"))


@pytest.mark.slow
@pytest.mark.parametrize("retake_tags", [True, False])
def test_retake_next_leg(cfg, tmp_path, runs, retake_tags):
    mount = cfg["robots"]["L"]["mount"]
    r_eff = cfg["scene"]["r_obj"] + cfg["scene"]["place_err"]
    state = {"moved": False, "retake": False, "at6": None}

    def prepare(fleet):
        orig = fleet._snapshot
        blank = synth.Scene({**fleet.board, "tags": []}, fleet.mesh)

        def angle_k(pose):
            cx, cy = drive.cam_xy(pose, mount)
            rel = (math.degrees(math.atan2(cy, cx)) - fleet.bots["L"].phi0) % 360
            return rel / 15.0

        def hooked(bot, req):
            pose = fleet.true_pose("L")
            k = angle_k(pose)
            if not state["moved"] and abs(k - 5) < 0.3:
                cx, cy = drive.cam_xy(pose, mount)
                phi = math.degrees(math.atan2(cy, cx))
                inner = drive.view_pose(phi, r_eff + 0.30, mount)
                with fleet._sim_lock:
                    fleet.bots["L"].robot._pose = inner
                state["moved"] = True
                return orig(bot, req)
            if state["moved"] and not state["retake"]:
                state["retake"] = True
                if not retake_tags:
                    saved, fleet.scene = fleet.scene, blank
                    try:
                        return orig(bot, req)
                    finally:
                        fleet.scene = saved
            if state["retake"] and state["at6"] is None and abs(k - 6) < 0.3:
                state["at6"] = pose
            return orig(bot, req)

        fleet._snapshot = hooked

    run = scan(cfg, tmp_path, roles=("L",), prepare=prepare)
    runs.append(run)
    assert run.code == 0 and state["moved"] and state["retake"]
    corr = run.events("L", "correct")
    assert len(corr) == 1 and corr[0]["k"] == 5 and corr[0]["d_before"] == pytest.approx(0.30, abs=0.01)
    retake = [e for e in run.events("L", "view") if e["use"] == "retake"]
    assert len(retake) == 1 and retake[0]["pose"] is retake_tags
    assert retake[0]["d_source"] == ("tag" if retake_tags else "odom")
    target = run.fleet.view_target("L", 6)
    got = state["at6"]
    assert got is not None
    assert math.dist(drive.cam_xy(got, mount), drive.cam_xy(target, mount)) <= 0.025
    assert abs(math.degrees(wrap_angle(got[2] - target[2]))) <= 5.0
