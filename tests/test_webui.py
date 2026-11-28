"""웹 화면(docs/specs/webui.md): 맞추기 값, 스캔 일시정지·재개·취소, 보강 촬영, 구멍 후보, HTTP."""
import json
import math
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest
import yaml

from scan3d import dataset
from scan3d.clock import Clock
from scan3d.config import ROOT
from scan3d.robot import drive
from scan3d.sim.fake_agent import BOX, FakeFleet
from scan3d.web import alarms, closeup, coverage, focus, holes, preview, recipe, server
from scan3d.robot.unit import StartError
from scan3d.web.session import Busy, Job, NotReady, Session

SCALE = 5.0
SMALL = {"color": {"w": 1280, "h": 720, "fmt": "yuyv", "fps": 15},
         "depth": {"w": 424, "h": 240, "fmt": "z16", "fps": 15}}


def wait(pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return
        time.sleep(0.1)
    raise AssertionError(f"기다린 일이 일어나지 않음: {what}")


def load_cfg():
    return yaml.safe_load((ROOT / "config" / "scan.yaml").read_text(encoding="utf-8"))


# ---------- 순수 함수 ----------

def test_camera_angles():
    """카메라가 판 좌표 (0.6, 0, 0.3)에서 판 중심을 보며 30° 숙임 → 높이 0.3, 숙임 30°, 조준 차 0."""
    c = np.array([0.6, 0.0, 0.3])
    pitch = math.radians(30)
    f = np.array([-math.cos(pitch), 0.0, -math.sin(pitch)])          # 광축
    right = np.array([0.0, 1.0, 0.0])
    down = np.cross(f, right)
    R = np.array([right, down, f])                                   # 카메라 x, y, z(판 좌표에서)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -R @ c
    z, p, aim = preview.camera_angles(T)
    assert math.isclose(z, 0.3, abs_tol=1e-9) and math.isclose(p, 30.0, abs_tol=1e-6)
    assert math.isclose(aim, 0.0, abs_tol=1e-6)


def test_pair_check():
    assert preview.pair_check({"L": 0.0, "H": 175.0}, 30)[1]
    gap, ok, hint = preview.pair_check({"L": 0.0, "H": 120.0}, 30)
    assert not ok and math.isclose(gap, 120.0) and "반시계 방향 60°" in hint
    assert preview.pair_check({"L": 10.0, "H": None}, 30)[0] is None
    assert preview.pair_check({"L": 10.0}, 30) == (None, True, "")


def test_focus_plan():
    cfg = load_cfg()
    side_low = {"point": [0.09, 0.0, 0.05], "normal": [1, 0, 0]}
    side_high = {"point": [0.0, 0.07, 0.17], "normal": [0, 1, 0]}
    top = {"point": [0.02, 0.01, 0.12], "normal": [0, 0, 1]}
    center_top = {"point": [0.0, 0.0, 0.16], "normal": None}
    p = focus.plan([side_low, side_high, top, center_top], cfg, ["L", "H"])
    assert [x["role"] for x in p] == ["L", "H", "H", "H"]
    assert math.isclose(p[0]["phi"], 0.0, abs_tol=1e-9) and math.isclose(p[1]["phi"], 90.0)
    assert p[0]["phis"] == [-10.0, 0.0, 10.0]
    assert math.isclose(p[2]["phi"], math.degrees(math.atan2(0.01, 0.02)))   # 윗면: 중심 → 점 방향
    assert p[3]["phi"] is None and p[3]["phis"] is None                     # 중심 위: 그 자리에서
    assert focus.plan([side_low], cfg, ["H"])[0]["role"] == "H"             # 맡을 로봇이 없으면 다른 로봇
    with pytest.raises(focus.PlanError):
        focus.plan([{"point": [1, 2]}], cfg, ["L"])
    with pytest.raises(focus.PlanError):
        focus.plan([side_low], cfg, [])


def test_project_crop():
    K = np.array([[600.0, 0, 640], [0, 600.0, 360], [0, 0, 1]])
    T = np.eye(4)
    T[2, 3] = 0.5                                                    # 점이 카메라 앞 0.5 m
    uv = focus.project([0.05, 0.0, 0.0], T, K)
    assert np.allclose(uv, (700.0, 360.0))
    assert focus.project([0, 0, -1.0], T, K) is None                 # 카메라 뒤
    img = np.zeros((720, 1280, 3), np.uint8)
    jpg, local = focus.crop_ring(img, (1270.0, 10.0), (480, 360))    # 모서리 근처 → 창을 안으로 민다
    assert jpg[:2] == b"\xff\xd8" and local == (470, 10)


def _grid_box(n=6, w=0.18, d=0.14, h=0.16, hole=None):
    """바닥이 열린 상자(면마다 n×n 격자). hole=(면, i0, j0)이면 그 면의 2×2 칸을 뺀다."""
    verts, tris = [], []
    faces = {"+x": (np.array([w / 2, -d / 2, 0]), np.array([0, d, 0]), np.array([0, 0, h])),
             "-x": (np.array([-w / 2, d / 2, 0]), np.array([0, -d, 0]), np.array([0, 0, h])),
             "+y": (np.array([w / 2, d / 2, 0]), np.array([-w, 0, 0]), np.array([0, 0, h])),
             "-y": (np.array([-w / 2, -d / 2, 0]), np.array([w, 0, 0]), np.array([0, 0, h])),
             "+z": (np.array([-w / 2, -d / 2, h]), np.array([w, 0, 0]), np.array([0, d, 0]))}
    index = {}

    def vid(p):
        key = tuple(np.round(p, 6))
        if key not in index:
            index[key] = len(verts)
            verts.append(p)
        return index[key]

    for name, (o, u, v) in faces.items():
        for i in range(n):
            for j in range(n):
                if hole and hole[0] == name and hole[1] <= i < hole[1] + 2 and hole[2] <= j < hole[2] + 2:
                    continue
                a, b = vid(o + u * i / n + v * j / n), vid(o + u * (i + 1) / n + v * j / n)
                c, e = vid(o + u * (i + 1) / n + v * (j + 1) / n), vid(o + u * i / n + v * (j + 1) / n)
                tris += [(a, b, c), (a, c, e)]
    return np.array(verts), np.array(tris)


def test_holes():
    v, t = _grid_box()
    assert holes.find_holes(v, t, floor_cut=0.005) == []            # 바닥 테두리만 열림 → 후보 없음
    v, t = _grid_box(hole=("+x", 2, 2))
    found = holes.find_holes(v, t, floor_cut=0.005)
    assert len(found) == 1
    h = found[0]
    assert h["n_edges"] == 8 and math.isclose(h["center"][0], 0.09, abs_tol=1e-6)
    assert 0.04 < h["center"][2] < 0.12 and np.allclose(h["normal"][:2], [1, 0], atol=0.2)
    assert math.isclose(h["perimeter_m"], 2 * (0.14 / 3) + 2 * (0.16 / 3), rel_tol=1e-6)


def test_raw_writer_continues_numbering(tmp_path):
    rw = dataset.RawWriter(tmp_path / "L")
    hdr = {"ok": True, "t": 1.0, "odom": [0, 0, 0], "color": {"w": 2, "h": 1, "fmt": "yuyv", "bytes": 4}}
    assert rw.snapshot(hdr, [b"\0" * 4], 0, "view", 0.0) == "000000"
    rw.close()
    rw2 = dataset.RawWriter(tmp_path / "L")                          # 보강 촬영: 같은 폴더에 이어 쓴다
    assert rw2.snapshot(hdr, [b"\0" * 4], 100, "view", 0.0) == "000001"
    rw2.close()


def test_outcome_failed_vs_cancelled(tmp_path):
    """스캔 실패 때 station.run의 abort_all도 cancel을 켜지만, 사용자가 누른 취소만 cancelled다."""
    cfg = load_cfg()
    s = Session(cfg, ROOT / "config" / "scan.yaml", {"L": ("127.0.0.1", 1, 2), "H": ("127.0.0.1", 3, 4)},
                Clock(), data_dir=tmp_path)
    job = Job("scan")
    s.cancel.set()
    assert s._outcome(job, StartError("H: 표식이 보이지 않음")) == ("failed", "StartError: H: 표식이 보이지 않음")
    assert s._outcome(job, drive.Cancelled("취소됨"))[0] == "failed"
    job.user_cancel = True
    assert s._outcome(job, drive.Cancelled("취소됨")) == ("cancelled", "취소됨")
    with pytest.raises(NotReady):
        s.cancel_job()                                               # 작업이 없음


def test_coverage_score():
    """카메라(원점, +z) 앞 1 m 평면: 정면이면 시점마다 1, 가리면 0, 비껴 보면 덜."""
    K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
    xs, ys = np.meshgrid(np.linspace(-0.2, 0.2, 5), np.linspace(-0.15, 0.15, 4))
    v = np.c_[xs.ravel(), ys.ravel(), np.ones(xs.size)]
    n = np.tile([0.0, 0.0, -1.0], (len(v), 1))
    T = np.eye(4)
    flat = np.full((480, 640), 1000, np.uint16)
    s = coverage.score(v, n, [(T, K, flat), (T, K, flat)])
    assert s.shape == (len(v),) and np.all(s > 1.9)                 # 두 시점에서 정면(cos≈1)
    blocked = np.full((480, 640), 500, np.uint16)                    # 0.5 m 앞에 다른 것
    assert np.all(coverage.score(v, n, [(T, K, blocked)]) == 0)
    side = np.tile([1.0, 0.0, -0.2], (len(v), 1))                    # 거의 옆을 향한 면
    assert np.all(coverage.score(v, side, [(T, K, flat)]) < 0.5)


def test_plan_region():
    cfg = load_cfg()
    zs = np.linspace(0.02, 0.12, 6)
    ys = np.linspace(-0.05, 0.05, 6)
    low_x = {"points": [[0.09, y, z] for y in ys for z in zs], "normals": [[1, 0, 0]] * 36}
    p = focus.plan_region(low_x, cfg, ["L", "H"])
    assert len(p) == 1 and p[0]["role"] == "L" and p[0]["region"]
    assert min(p[0]["phis"]) <= -4.9 and max(p[0]["phis"]) >= 4.9 and len(p[0]["phis"]) >= 2
    tall = {"points": [[0.09, 0.0, z] for z in np.linspace(0.04, 0.2, 20)], "normals": [[1, 0, 0]] * 20}
    assert sorted(x["role"] for x in focus.plan_region(tall, cfg, ["L", "H"])) == ["H", "L"]   # 높이 경계를 걸침
    corner = {"points": [[0.09, 0.0, 0.05]] * 10 + [[0.0, 0.07, 0.05]] * 10,
              "normals": [[1, 0, 0]] * 10 + [[0, 1, 0]] * 10}
    q = focus.plan_region(corner, cfg, ["L", "H"])[0]
    assert q["phis"][0] <= -4.9 and q["phis"][-1] >= 94.9 and len(q["phis"]) <= focus.REGION_MAX_SHOTS
    assert all(b - a <= focus.REGION_STEP_DEG + 1e-6 for a, b in zip(q["phis"], q["phis"][1:]))
    top = {"points": [[0.02, 0.0, 0.12]] * 8, "normals": [[0, 0, 1]] * 8}
    assert focus.plan_region(top, cfg, ["L", "H"])[0]["role"] == "H"
    assert focus.plan_region(low_x, cfg, ["H"])[0]["role"] == "H"                   # L이 없으면 H
    with pytest.raises(focus.PlanError):
        focus.plan_region({"points": [[0, 0]]}, cfg, ["L"])
    assert focus._arc([350.0, 10.0]) == (350.0, 20.0)                             # 0°를 넘는 범위


def test_recipes_file_and_judge(tmp_path):
    recs = recipe.load(ROOT / "config" / "recipes.yaml")
    assert "default" in recs and "sim_box" in recs
    cfg = load_cfg()
    pig = recipe.apply(cfg, recs["pig_doll"])
    assert pig["scene"]["obj_h"] == 0.23 and cfg["scene"]["obj_h"] == 0.2
    board = json.loads((ROOT / "config" / "board.json").read_text(encoding="utf-8"))
    assert abs(recipe.free_radius(board) - 0.1875) < 1e-9
    for name, r in recs.items():                                      # 들어 있는 레시피는 모두 이 판에서 된다
        recipe.check(name, recipe.apply(cfg, r), board)
    with pytest.raises(recipe.RecipeError):                           # 표식을 가리는 큰 물체
        recipe.check("big", recipe.apply(cfg, {"scene": {"r_obj": 0.30}}), board)
    with pytest.raises(recipe.RecipeError):                           # 원이 너무 작아 늘 d_min보다 가깝다
        recipe.check("tight", recipe.apply(cfg, {"scene": {"R": 0.5}}), board)
    bad = tmp_path / "r.yaml"
    for text in ("x: {capture: {n_depth: 1}}", "x: {scene: {r_obj: -1}}", "x: {spec_mm: [1, 2]}", "x: {spec_mm: [1, 2, 3]}",
                 "x: {scene: {R: .inf}}", "x: {scene: {n_views: 24.5}}", "x: {scene: {n_views: 2}}",
                 "x: {spec_mm: [128, 138, 201], tol_mm: 5}", "x: {spec_mm: [3, 2, 1], tol_mm: .inf}",
                 "x: {spec_mm: [3, 2, 1], tol_mm: true}"):
        bad.write_text(text, encoding="utf-8")
        with pytest.raises(recipe.RecipeError):
            recipe.load(bad)
    j = recipe.judge([183.6, 143.6, 160.4], {"spec_mm": [180, 140, 160], "tol_mm": 5})
    assert j["result"] == "OK" and j["dev"] == [3.6, 3.6, 0.4] and j["axes"] == ["OK"] * 3
    j = recipe.judge([190.0, 140, 160], {"spec_mm": [180, 140, 160], "tol_mm": [5, 5, 5]})
    assert j["result"] == "NG" and j["axes"] == ["NG", "OK", "OK"]
    assert recipe.judge([1, 2, 3], {}) is None
    assert recipe.judge(None, {"spec_mm": [1, 1, 1], "tol_mm": 1})["result"] == "NG"
    assert recipe.judge([1, 1], {"spec_mm": [1, 1, 1], "tol_mm": 1})["result"] == "NG"      # 높이 없음


def test_alarms():
    from scan3d.robot.link import RemoteError
    assert alarms.classify(StartError("H: 표식이 보이지 않음")) == "E01"
    assert alarms.classify(StartError("두 로봇 각도 차 120°")) == "E04"
    assert alarms.classify(drive.Blocked("x")) == "E02"
    assert alarms.classify(RemoteError("x")) == "E03" and alarms.classify(TimeoutError()) == "E03"
    assert alarms.classify(RuntimeError("융합한 시점 없음")) == "E05"
    assert alarms.classify(ValueError("x")) == "E09"
    assert alarms.classify(FileNotFoundError("depth.png")) == "E09"   # 디스크 오류는 통신 끊김이 아니다
    a = alarms.Alarms()
    a.raise_("W06", "L 막힘", "L", key="obs:L")
    a.raise_("W06", "L 막힘 계속", "L", key="obs:L")                  # 같은 key는 하나
    snap = a.snapshot()
    assert len(snap["items"]) == 1 and snap["active_warn"] and not snap["active_alarm"]
    a.clear("obs:L")
    assert not a.snapshot()["active_warn"] and a.snapshot()["items"][0]["cleared_t"]
    i = a.raise_("E02", "scan: Blocked")
    assert a.snapshot()["active_alarm"] and a.snapshot()["unacked"] == 1   # 해제된 경고는 확인을 기다리지 않는다
    a.ack(i)                                                         # key 없는 알람은 확인하면 해제
    assert not a.snapshot()["active_alarm"]
    a.ack()
    assert a.snapshot()["unacked"] == 0
    a.raise_("W06", "H: 대기", "H", key="obs:H")
    a.ack()
    a.raise_("W06", "H: 정지", "H", key="obs:H")                    # 같은 알람이 다른 내용으로 → 다시 확인
    assert a.snapshot()["unacked"] == 1


def test_obstacle_events(tmp_path):
    s = Session(load_cfg(), ROOT / "config" / "scan.yaml", {"L": ("127.0.0.1", 1, 2), "H": ("127.0.0.1", 3, 4)},
                Clock(), data_dir=tmp_path)
    s._on_event("L", "drive", {"event": "wait", "view": 3})          # 작업이 없으면(늦게 온 이벤트) 버린다
    assert s.slots["L"].obstacle is None
    s.job = Job("scan")
    s._on_event("L", "drive", {"event": "wait", "view": 3})
    assert s.slots["L"].obstacle["event"] == "wait" and s.alarms.active("obs:L")
    s._on_event("L", "drive", {"event": "stop", "view": 3})
    assert len(s.alarms.snapshot()["items"]) == 1                    # 같은 막힘은 알람 하나
    s._on_event("L", "drive", {"event": "go", "view": 3})            # 길이 비어 다시 간다
    assert s.slots["L"].obstacle is None and not s.alarms.active("obs:L")
    s.job = None
    with pytest.raises(NotReady):
        s.set_recipe("nope")
    s.set_recipe("pig_doll")
    assert s.cfg["scene"]["obj_h"] == 0.23 and s.base_cfg["scene"]["obj_h"] == 0.2


def _stub_robot_sh(tmp_path, start_out="started robot_scan", start_rc=0):
    log = tmp_path / "robot_sh.log"
    sh = tmp_path / "robot.sh"
    sh.write_text(f"""#!/bin/sh
echo "$@" >> {log}
case "$3" in
  start) echo "{start_out}"; exit {start_rc};;
  stop) echo "stopped robot_scan";;
  status) echo "robot_scan: running"; echo "Mem: 1 2 3";;
esac
""", encoding="utf-8")
    sh.chmod(0o755)
    return sh, log


def test_robot_stack_buttons(tmp_path):
    """웹 화면 버튼 → scripts/robot.sh -H <host> start|stop|status robot_scan."""
    sh, log = _stub_robot_sh(tmp_path)
    cfg = load_cfg()
    addrs = {"L": ("127.0.0.1", 1, 2), "H": ("127.0.0.1", 3, 4)}
    s = Session(cfg, ROOT / "config" / "scan.yaml", addrs, Clock(), data_dir=tmp_path, robot_sh=sh)
    s.stack_action("L", "start")
    wait(lambda: not s.slots["L"].stack_busy, 10, "켜기")
    assert s.slots["L"].stack == "running"
    assert log.read_text().splitlines()[-1] == f"-H {cfg['robots']['L']['host']} start robot_scan"
    s.stack_action("L", "stop")
    wait(lambda: not s.slots["L"].stack_busy, 10, "끄기")
    assert s.slots["L"].stack == "stopped" and not s.alarms.active("conn:L")     # 일부러 끈 것은 알람 없음
    s.connect()                                                      # 다시 연결 주기: 끈 로봇은 건너뛴다
    assert not s.alarms.active("conn:L") and s.alarms.active("conn:H")
    s.stack_action("H", "status")
    wait(lambda: not s.slots["H"].stack_busy, 10, "상태")
    assert s.slots["H"].stack == "running"
    st = s.state()
    assert st["robots"]["L"]["stack"] == "stopped" and st["robots"]["H"]["stack"] == "running"
    s.job = Job("scan")                                              # 작업 중에는 끄지 않는다
    with pytest.raises(Busy):
        s.stack_action("H", "stop")
    s.job = None
    with pytest.raises(NotReady):
        s.stack_action("L", "jump")
    s.slots["H"].stack = "running"
    s.start_stacks()                                                 # 꺼진 L만 켠다
    wait(lambda: not s.slots["L"].stack_busy, 10, "모두 켜기")
    assert log.read_text().splitlines()[-1].endswith("start robot_scan") and "-H " + cfg["robots"]["L"]["host"] in log.read_text().splitlines()[-1]

    other = tmp_path / "other"
    other.mkdir()
    sh2, _ = _stub_robot_sh(other, start_out="이미 구동 스택이 돌고 있다(LiDAR·모터는 한 프로세스만 쓴다)", start_rc=1)
    s2 = Session(cfg, ROOT / "config" / "scan.yaml", addrs, Clock(), data_dir=tmp_path, robot_sh=sh2)
    s2.stack_action("L", "start")
    wait(lambda: not s2.slots["L"].stack_busy, 10, "막힌 켜기")
    assert s2.slots["L"].stack == "blocked" and s2.alarms.snapshot()["active_alarm"]
    fake = Session(cfg, ROOT / "config" / "scan.yaml", addrs, Clock(), data_dir=tmp_path, fake=True, robot_sh=sh)
    with pytest.raises(NotReady):
        fake.stack_action("L", "start")
    assert fake.state()["robots"]["L"]["stack"] == "sim"


def test_focus_wrap():
    assert focus._wrap(350.0) == -10.0 and focus._wrap(-190.0) == 170.0 and focus._wrap(10.0) == 10.0


def test_closeup_geometry():
    """근접 촬영 자리: 카메라가 지점에서 standoff 떨어져 정면으로 보고, 표식 자세를 로봇이 움직인 만큼 옮긴다."""
    cfg = load_cfg()
    for role in ("L", "H"):
        mount = dict(cfg["robots"][role]["mount"], x=0.02, y=0.07, yaw_deg=86) if role == "H" else cfg["robots"][role]["mount"]
        point = [0.07, 0.02, 0.08]
        pose, cam = closeup.shot_pose(point, 20.0, 0.25, mount)
        assert math.hypot(cam[0] - point[0], cam[1] - point[1]) == pytest.approx(0.25)
        assert drive.cam_xy(pose, mount) == pytest.approx(cam)
        T = drive.camera_pose(pose, mount)                                   # 그 자세의 실제 카메라
        u, v = focus.project(point, T, np.array([[640.0, 0, 640], [0, 640.0, 360], [0, 0, 1]]))
        assert u == pytest.approx(640, abs=1e-6)                             # 지점이 사진 가로 가운데
        # 원 위 자세 E의 표식 자세를 로봇 이동만큼 옮기면 그 자리의 카메라 자세와 같다
        pose_e = drive.view_pose(25.0, 0.62, mount, yaw=drive.arc_yaw(25.0, 0.62, mount))
        assert closeup.moved(drive.camera_pose(pose_e, mount), pose_e, pose) == pytest.approx(T, abs=1e-9)
    st = closeup.settings(cfg)
    assert st["standoff"] == 0.25 and closeup.settings({"closeup": {"standoff": 0.1}})["standoff"] == closeup.STANDOFF_MIN
    mount = cfg["robots"]["L"]["mount"]
    pose_e = drive.view_pose(0.0, 0.62, mount, yaw=drive.arc_yaw(0.0, 0.62, mount))
    K = np.array([[646.0, 0, 640], [0, 646.0, 360], [0, 0, 1]])
    T_e = drive.camera_pose(pose_e, mount)
    s, pose, _ = closeup.choose_standoff([0.09, 0.0, 0.08], 0.0, mount, T_e, pose_e, K, (1280, 720), st, 0.615)
    assert s == pytest.approx(0.25) and math.hypot(pose[0] - 0.09, pose[1]) >= st["clear_obj"]
    s, why, _ = closeup.choose_standoff([0.40, 0.0, 0.08], 0.0, mount, T_e, pose_e, K, (1280, 720), st, 0.615)
    assert s is None and "자리가 없음" in why                                # 지점이 원에 너무 가깝다
    p = closeup.plan([{"point": [0.09, 0.0, 0.05], "normal": [1, 0, 0]}, {"point": [0.0, 0.05, 0.19]}], cfg, ["L", "H"])
    assert [x["role"] for x in p] == ["L", "H"] and p[0]["phi"] == pytest.approx(0.0)


class _StubDriver:
    """원 위 각만 따라가는 주행(보강 촬영의 이동 나누기 확인용)."""

    def __init__(self, phi, arc=True, off_orbit=False):
        self.phi, self.arc, self.off_orbit, self.hops = phi, arc, off_orbit, []

    def angle_deg(self):
        return self.phi

    def on_orbit(self):
        return self.arc and not self.off_orbit

    def go_to_phi(self, phi):
        self.hops.append(round(phi - self.phi, 6))
        self.phi, self.off_orbit = phi, False


class _StubUnit:
    def __init__(self, driver):
        self.driver, self.refixes = driver, 0

    def refix(self):
        self.refixes += 1


def _hops(delta, tmp_path, transit_max=None, **l_driver):
    """L(각 0°)·H(각 180°)를 함께 delta° 옮길 때 로봇마다 한 번씩 간 각 목록."""
    cfg = load_cfg()
    if transit_max is not None:
        cfg["webui"]["transit_max_deg"] = transit_max
    units = {"L": _StubUnit(_StubDriver(0.0, **l_driver)), "H": _StubUnit(_StubDriver(180.0))}
    run = focus.FocusRun(cfg, units, tmp_path, [], hold=lambda role=None: None, cancelled=lambda: False,
                         progress={}, log=lambda text: None)
    run._move_all(delta, "L")
    hops = {r: u.driver.hops for r, u in units.items()}
    assert hops["L"] == hops["H"]                                    # 두 로봇이 늘 같은 각만큼
    long = [h for h in hops["L"][:-1] if abs(h) > focus.STEP_DEG]    # 긴 원호 뒤 갈 길이 남으면 자세를 다시 잡는다
    assert units["L"].refixes == units["H"].refixes == len(long)
    assert not hops["L"] or math.isclose(units["L"].driver.phi % 360, delta % 360, abs_tol=1e-6)
    return hops["L"]


def test_focus_transit(tmp_path):
    assert _hops(170.0, tmp_path) == [85.0, 85.0]                    # 반시계: 90° 이하의 원호 둘
    assert _hops(10.0, tmp_path) == [10.0]
    assert _hops(-120.0, tmp_path) == [80.0, 80.0, 80.0]             # 시계로 멀면 반시계로 돌아서
    assert _hops(-30.0, tmp_path) == [-15.0, -15.0]                  # 시계로 가까우면 15° 이하씩
    assert _hops(60.0, tmp_path, arc=False) == [15.0] * 4            # 원호를 모르는 로봇이 있으면 15° 이하씩
    assert _hops(-120.0, tmp_path, arc=False) == [-15.0] * 8
    assert _hops(90.0, tmp_path, off_orbit=True) == [15.0, 75.0]     # 원에서 벗어나 있으면 15°로 자리를 잡은 뒤
    assert _hops(-120.0, tmp_path, transit_max=15) == [-15.0] * 8    # 설정으로 끄면 가까운 방향 15° 이하씩
    assert _hops(0.3, tmp_path) == []


# ---------- HTTP(세션 대역) ----------

class StubSession:
    def __init__(self):
        self.calls = []
        self.alarms = alarms.Alarms()
        self.slots = {"L": type("S", (), {"jpeg": b"\xff\xd8jpeg"})(), "H": type("S", (), {"jpeg": b""})()}

    def state(self):
        return {"mode": "control", "calls": list(self.calls)}

    def connect(self):
        self.calls.append("connect")

    def set_preview(self, on):
        self.calls.append(("preview", on))

    def start_scan(self):
        raise NotReady("각도 차 120°")

    def pause(self):
        raise Busy("작업 중")

    def resume(self):
        self.calls.append("resume")

    def cancel_job(self):
        self.calls.append("cancel")

    def set_recipe(self, name):
        self.calls.append(("recipe", name))

    def stack_action(self, role, action):
        if action not in ("start", "stop", "status"):
            raise NotReady("모르는 동작")
        self.calls.append(("stack", role, action))

    def start_stacks(self):
        self.calls.append("start_all")

    def start_focus(self, markers, regions=None):
        if not markers and not regions:
            raise focus.PlanError("markers")
        self.calls.append(("focus", markers))

    def focus_image(self, i):
        return b"\xff\xd8crop" if i == 0 else None

    def set_obstacle(self, on, view=None):
        self.calls.append(("obstacle", on, view))


@pytest.fixture
def http(tmp_path):
    stub = StubSession()
    httpd = server.serve(server.App([tmp_path / "scans"], session=stub), "127.0.0.1", 0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}", stub
    httpd.shutdown()
    httpd.server_close()


def _req(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def test_http(http):
    base, stub = http
    assert _req(base + "/api/connect", {})[0] == 200
    code, body = _req(base + "/api/preview", {"on": True})
    assert code == 200 and json.loads(body)["calls"] == ["connect", ["preview", True]]
    code, body = _req(base + "/api/scan/start", {})
    assert code == 409 and json.loads(body) == {"error": "not_ready", "message": "각도 차 120°"}
    assert _req(base + "/api/job/pause", {})[0] == 409
    assert _req(base + "/api/job/resume", {})[0] == 200 and _req(base + "/api/job/cancel", {})[0] == 200
    assert _req(base + "/api/focus/start", {"markers": []})[0] == 400
    assert _req(base + "/api/focus/start", {"markers": [{"point": [0, 0, 0.1]}]})[0] == 200
    assert stub.calls[-1] == ("focus", [{"point": [0, 0, 0.1]}])
    assert _req(base + "/api/nope", {})[0] == 404
    assert _req(base + "/api/recipe", {"name": "sim_box"})[0] == 200 and stub.calls[-1] == ("recipe", "sim_box")
    assert _req(base + "/api/robot/stack", {"role": "L", "action": "start"})[0] == 200
    assert stub.calls[-1] == ("stack", "L", "start")
    assert _req(base + "/api/robot/stack", {"role": "L", "action": "jump"})[0] == 409
    assert _req(base + "/api/robot/stack/start_all", {})[0] == 200 and stub.calls[-1] == "start_all"
    aid = stub.alarms.raise_("E02", "x")
    assert _req(base + "/api/alarm/ack", {"id": "abc"})[0] == 400
    assert _req(base + "/api/alarm/ack", {"id": aid})[0] == 200 and stub.alarms.snapshot()["unacked"] == 0
    def raw(body, headers):
        req = urllib.request.Request(base + "/api/scan/start", data=body, method="POST", headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code
    n_calls = len(stub.calls)
    assert raw(b"{bad", {"Content-Type": "application/json"}) == 400
    assert raw(b"{}", {"Content-Type": "text/plain"}) == 403                      # 다른 사이트의 단순 요청
    assert raw(b"{}", {"Content-Type": "application/json", "Origin": "http://evil.example"}) == 403
    assert len(stub.calls) == n_calls                                                # 아무것도 하지 않았다
    host = base.split("//")[1]
    assert raw(b"{}", {"Content-Type": "application/json", "Origin": f"http://{host}"}) == 409   # 같은 출처는 통과
    rebound = "evil.example:" + host.split(":")[1]                   # DNS 재바인딩: Origin == Host지만 이름이 IP가 아님
    assert raw(b"{}", {"Content-Type": "application/json", "Host": rebound, "Origin": f"http://{rebound}"}) == 403
    assert raw(b"[]", {"Content-Type": "application/json"}) == 400   # 객체가 아닌 JSON
    assert len(stub.calls) == n_calls
    assert _req(base + "/api/preview/L.jpg") == (200, b"\xff\xd8jpeg")
    assert _req(base + "/api/preview/H.jpg")[0] == 404                # 아직 사진 없음
    assert _req(base + "/api/preview/X.jpg")[0] == 404
    assert _req(base + "/api/focus/0.jpg") == (200, b"\xff\xd8crop")
    assert _req(base + "/api/focus/1.jpg")[0] == 404
    assert _req(base + "/api/holes?id=nope")[0] == 404
    assert _req(base + "/stageview.js")[0] == 200
    assert _req(base + "/api/sim/obstacle", {"on": True, "view": 2})[0] == 200
    assert stub.calls[-1] == ("obstacle", True, 2)
    assert _req(base + "/api/sim/obstacle", {"on": True, "view": "bad"})[0] == 400


def test_stage_state_and_live_capture(tmp_path):
    """실시간 3D 스테이지 상태: 판·표식, fix+odom 자세, LiDAR 판 좌표, 촬영 콜백 갱신."""
    cfg = load_cfg()
    s = Session(cfg, ROOT / "config" / "scan.yaml", {"L": ("127.0.0.1", 1, 2), "H": ("127.0.0.1", 3, 4)},
                Clock(), data_dir=tmp_path)

    class FakeLink:
        def alive(self):
            return True

        def ages(self):
            return (0.05, 0.08)

        def odom(self):
            return (0.10, 0.0, 0.0)

        def scan(self):
            # 정면(로봇 기준 0 rad → mount_yaw=180°이므로 raw 각도는 π) 0.5 m 지점에 장애물
            return ({"angle_min": math.pi, "angle_inc": 0.1, "range_min": 0.05, "range_max": 5.0,
                     "ranges": [0.5]}, 0.05)

    s.slots["L"].link = FakeLink()
    s.slots["L"].fix = ((0.62, 0.0, math.pi / 2), (0.0, 0.0, 0.0))   # 출발 자세 + odom +0.10 m → (0.62, 0.10)
    st = s.state()
    stage = st["stage"]
    assert math.isclose(stage["board_m"], 0.594, abs_tol=1e-4)
    assert len(stage["tags"]) == 16
    rl = stage["robots"]["L"]
    assert rl["pose"] is not None
    assert math.isclose(rl["pose"][0], 0.62, abs_tol=1e-3)
    assert math.isclose(rl["pose"][1], 0.10, abs_tol=1e-3)
    assert math.isclose(rl["pose"][2], math.pi / 2, abs_tol=1e-3)
    # 정면(+Y) 0.5 m 지점의 LiDAR 반사점이 (0.62, 0.60) 근처
    assert rl["lidar"] and math.isclose(rl["lidar"][0][0], 0.62, abs_tol=1e-2)
    assert math.isclose(rl["lidar"][0][1], 0.60, abs_tol=1e-2)

    # 촬영 콜백: 스캔 중에도 preview JPEG, seq, shots, fix가 갱신된다
    frame = type("F", (), {"color": np.full((120, 160, 3), 180, np.uint8),
                           "depth": np.full((120, 160), 600, np.uint16)})()
    T = drive.camera_pose((0.62, 0.0, math.pi / 2), cfg["robots"]["L"]["mount"])
    pose = type("P", (), {"T_cam_board": T, "n_tags": 6, "reproj_px": 0.4})()
    s._on_capture("L", "view", 3, frame, pose, {"odom": [0.1, 0.0, 0.0]}, cfg)
    st2 = s.state()
    assert st2["robots"]["L"]["preview_seq"] == 1
    assert s.slots["L"].jpeg[:2] == b"\xff\xd8"
    assert st2["stage"]["robots"]["L"]["shot_k"] == 3
    assert len(st2["stage"]["robots"]["L"]["shots"]) == 1
    assert st2["stage"]["robots"]["L"]["shots"][0]["k"] == 3
    with pytest.raises(NotReady):
        s.set_obstacle(True)

    # fake 세션에서 장애물 동적 추가·제거
    class StubFleet:
        def __init__(self):
            self.boxes = []

        def true_pose(self, role):
            return (0.62, 0.0, math.pi / 2) if role == "L" else (-0.62, 0.0, -math.pi / 2)

        def world_boxes(self):
            return list(self.boxes)

        def add_obstacle(self, k=None, size=0.10, role="L"):
            b = (0.4, 0.45, size, size, 135.0)
            self.boxes.append(b)
            return b

        def clear_obstacles(self):
            self.boxes.clear()

    sf = Session(cfg, ROOT / "config" / "scan.yaml", {"L": ("127.0.0.1", 1, 2), "H": ("127.0.0.1", 3, 4)},
                 Clock(), data_dir=tmp_path, fake=True, fleet=StubFleet())
    assert sf.state()["stage"]["obstacles"] == []
    sf.set_obstacle(True)
    assert len(sf.state()["stage"]["obstacles"]) == 1
    sf.set_obstacle(False)
    assert sf.state()["stage"]["obstacles"] == []


def test_http_view_only(tmp_path):
    httpd = server.serve(server.App([tmp_path]), "127.0.0.1", 0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        assert json.loads(_req(base + "/api/state")[1]) == {"mode": "view"}
        assert _req(base + "/api/scan/start", {})[0] == 404
    finally:
        httpd.shutdown()
        httpd.server_close()


# ---------- 가짜 로봇 2대로 처음부터 끝까지 ----------

@pytest.fixture(scope="module")
def site(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("webui")
    cfg = load_cfg()
    cfg["capture"].update(json.loads(json.dumps(SMALL)))
    cfg_path = tmp / "scan.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    clock = Clock(SCALE)
    fleet = FakeFleet(cfg, clock)
    addrs = fleet.start()
    recipes = tmp / "recipes.yaml"
    recipes.write_text("default: {label: 기본}\nsim_box: {label: 시험 상자, scene: {obj_h: 0.21}, "
                       "spec_mm: [180, 140, 160], tol_mm: 15}\n", encoding="utf-8")
    session = Session(cfg, cfg_path, addrs, clock, data_dir=tmp / "scans", recipes_path=recipes)
    session.start()
    yield cfg, fleet, session
    session.close()
    fleet.stop()


def _cam_true(cfg, fleet, role):
    mount = cfg["robots"][role]["mount"]
    x, y = drive.cam_xy(fleet.true_pose(role), mount)
    return x, y, mount


@pytest.mark.slow
def test_preview_values(site):
    cfg, fleet, session = site
    assert all(s.connected() for s in session.slots.values())
    session.set_preview(True)
    wait(lambda: all(s.preview for s in session.slots.values()), 60, "두 로봇 미리보기")
    session.set_preview(False)
    r_eff = cfg["scene"]["r_obj"] + cfg["scene"]["place_err"]
    for role, s in session.slots.items():
        p = s.preview
        x, y, mount = _cam_true(cfg, fleet, role)
        assert p["pose"], p
        assert abs((p["phi_deg"] - math.degrees(math.atan2(y, x)) + 180) % 360 - 180) < 1.0
        assert abs(p["d_m"] - (math.hypot(x, y) - r_eff)) < 0.01
        assert abs(p["pitch_deg"] - mount["pitch_down_deg"]) < 1.0
        assert p["depth_center_m"] and 0.2 < p["depth_center_m"] < 1.5
        assert s.jpeg[:2] == b"\xff\xd8"
    st = session.state()
    assert st["pair"]["gap_ok"] and st["pair"]["can_start"], st["pair"]


@pytest.mark.slow
def test_scan_pause_resume(site):
    cfg, fleet, session = site
    session.set_recipe("sim_box")
    job = session.start_scan()
    with pytest.raises(Busy):
        session.start_scan()
    with pytest.raises(Busy):
        session.set_recipe("default")
    views = lambda: min(u.cur for u in job.pair.units.values()) if job.pair else -1   # noqa: E731
    wait(lambda: views() >= 3, 300, "시점 3")
    session.pause()
    wait(lambda: len(session.state()["job"]["progress"]["holding"]) >= 2, 60, "두 로봇 일시정지")
    held = {r: u.cur for r, u in job.pair.units.items()}
    time.sleep(3.0)                                                  # clock으로 15 s
    assert {r: u.cur for r, u in job.pair.units.items()} == held
    assert session.state()["job"]["paused"]
    session.resume()
    wait(lambda: job.state != "running", 900, "스캔 끝")
    assert job.state == "done", job.message
    w, d, h = job.result["dims_mm"]
    assert abs(w - BOX[0] * 1000) <= 12 and abs(d - BOX[1] * 1000) <= 12 and abs(h - BOX[2] * 1000) <= 12
    assert (session.last_scan_dir / "out" / "mesh.ply").is_file()
    assert session.state()["last_scan"]["mesh_id"] == f"scans/{job.scan_id}/out"
    assert job.result["judge"]["result"] == "OK" and session.state()["judge"]["recipe"] == "sim_box"
    saved = yaml.safe_load((session.last_scan_dir / "config.yaml").read_text(encoding="utf-8"))
    assert saved["scene"]["obj_h"] == 0.21                           # 다시 만들기가 레시피 값을 쓴다
    assert json.loads((session.last_scan_dir / "judge.json").read_text(encoding="utf-8"))["result"] == "OK"
    st = session.state()
    assert all(st["robots"][r]["front_m"] is not None for r in ("L", "H"))


@pytest.mark.slow
def test_focus_run(site):
    cfg, fleet, session = site
    assert session.last_scan_dir is not None
    before = {r: math.degrees(math.atan2(*_cam_true(cfg, fleet, r)[1::-1])) for r in ("L", "H")}
    marker = {"point": [BOX[0] / 2, 0.0, 0.05], "normal": [1.0, 0.0, 0.0]}    # +x 면 낮은 곳 → L, φ 0
    job = session.start_focus([marker])
    wait(lambda: job.state != "running", 900, "보강 촬영 끝")
    assert job.state == "done", job.message
    crops = job.result["crops"]
    assert [c["role"] for c in crops] == ["L", "L", "L"]
    assert all(c["url"] and c["posed"] for c in crops), crops
    assert sorted(round(c["phi"]) for c in crops) == [-10, 0, 10]
    for c in crops:                                                  # 점이 자른 사진 안(전체 사진 픽셀)
        assert 0 <= c["pixel"][0] < 1280 and 0 <= c["pixel"][1] < 720
        assert session.focus_image(crops.index(c))[:2] == b"\xff\xd8"
    views = json.loads((session.last_scan_dir / "L" / "views.json").read_text(encoding="utf-8"))
    assert len([v for v in views if v["k"] >= focus.FOCUS_K0]) == 3
    assert (session.last_scan_dir / "out_focus" / "mesh.ply").is_file()
    assert (session.last_scan_dir / "out" / "mesh.ply").is_file()               # 원래 결과는 그대로
    after = {r: math.degrees(math.atan2(*_cam_true(cfg, fleet, r)[1::-1])) for r in ("L", "H")}
    gap_before = (before["H"] - before["L"]) % 360
    gap = (after["H"] - after["L"]) % 360
    assert abs(gap - gap_before) < 5, (before, after)                # 두 로봇이 같은 각만큼 함께 움직였다
    assert abs(((after["L"] - 10.0) + 180) % 360 - 180) < 5          # L은 마지막 각(+10°)

    # 지금 L 각보다 조금 뒤(−25°)의 점 → 가까운 방향(시계)으로 가고 한 바퀴를 돌지 않는다
    phi = after["L"] - 25.0
    n = math.radians(phi)
    marker = {"point": [0.07 * math.cos(n), 0.07 * math.sin(n), 0.04], "normal": [math.cos(n), math.sin(n), 0.0]}
    moves_before = sum(1 for m in fleet.move_log if m[0] == "L")
    job = session.start_focus([marker])
    wait(lambda: job.state != "running", 900, "두 번째 보강 촬영")
    assert job.state == "done", job.message
    assert sum(1 for m in fleet.move_log if m[0] == "L") - moves_before <= 15      # 한 바퀴면 24 이상

    # 반대편(시계로 150°)의 점 → 두 로봇이 반시계로 긴 원호 몇 번에 돌아서 간다(제자리 회전 없음)
    here = {r: math.degrees(math.atan2(*_cam_true(cfg, fleet, r)[1::-1])) for r in ("L", "H")}
    n = math.radians(here["L"] - 140.0)
    marker = {"point": [0.07 * math.cos(n), 0.07 * math.sin(n), 0.04], "normal": [math.cos(n), math.sin(n), 0.0]}
    mark = len(fleet.move_log)
    job = session.start_focus([marker])
    wait(lambda: job.state != "running", 900, "세 번째 보강 촬영")
    assert job.state == "done", job.message
    texts = [m[1] for m in fleet.move_log[mark:] if m[0] == "L"]
    assert all("arc" in t for t in texts), texts
    assert len(texts) <= 6, texts                                    # 200° ÷ 90° → 원호 3 + 촬영 각 사이 2
    assert all(c["posed"] for c in job.result["crops"])
    after = {r: math.degrees(math.atan2(*_cam_true(cfg, fleet, r)[1::-1])) for r in ("L", "H")}
    assert abs(((after["L"] - (here["L"] - 130.0)) + 180) % 360 - 180) < 5      # L은 마지막 각(점 +10°)
    assert abs((after["H"] - after["L"]) % 360 - (here["H"] - here["L"]) % 360) < 5


@pytest.mark.slow
def test_closeup_run(site):
    """스캔 뒤 옆면 낮은 점 하나: L이 원 위에서 다가가 찍고 원 위로 돌아온다. 메시는 그대로."""
    cfg, fleet, session = site
    assert session.last_scan_dir is not None
    R = cfg["scene"]["R"]
    before = {r: _cam_true(cfg, fleet, r) for r in ("L", "H")}
    mesh_time = (session.last_scan_dir / "out" / "mesh.ply").stat().st_mtime
    phi = math.degrees(math.atan2(before["L"][1], before["L"][0])) + 30.0
    n = math.radians(phi)
    point = [BOX[0] / 2 * 0.9 * math.cos(n), BOX[1] / 2 * 0.9 * math.sin(n), 0.06]
    marker = {"point": point, "normal": [math.cos(n), math.sin(n), 0.0]}
    mark = len(fleet.move_log)
    job = session.start_closeup([marker])
    with pytest.raises(Busy):
        session.start_closeup([marker])
    closest = [9.0]

    def watch():
        x, y, _ = _cam_true(cfg, fleet, "L")
        closest[0] = min(closest[0], math.hypot(x - point[0], y - point[1]))
        return job.state != "running"
    wait(watch, 900, "근접 촬영 끝")
    assert job.state == "done", job.message
    (c,) = job.result["crops"]
    assert c["role"] == "L" and c["url"] and c["reason"] is None, c
    assert 0.20 <= c["distance_m"] <= 0.40 and c["source"] in ("lidar", "odom")
    assert 0 <= c["pixel"][0] < 1280 and 0 <= c["pixel"][1] < 720
    assert session.focus_image(0)[:2] == b"\xff\xd8"
    assert (session.last_scan_dir / "closeup").is_dir()
    assert closest[0] <= 0.33, closest                                # 실제로 다가갔다(원 위에서는 0.5 m 안팎)
    texts = [m[1] for m in fleet.move_log[mark:] if m[0] == "L"]
    assert any(t.startswith("turn") for t in texts) and any("forward" in t for t in texts)
    after = {r: _cam_true(cfg, fleet, r) for r in ("L", "H")}
    for r in ("L", "H"):                                              # 두 로봇 모두 원 위
        assert abs(math.hypot(after[r][0], after[r][1]) - R) <= 0.04, after
    gap = lambda a: (math.degrees(math.atan2(a["H"][1], a["H"][0]) - math.atan2(a["L"][1], a["L"][0]))) % 360   # noqa: E731
    assert abs(gap(after) - gap(before)) < 6
    assert c["return_err"][0] <= 0.05, c                              # 돌아와서 잰 자세와 추정의 차
    assert (session.last_scan_dir / "out" / "mesh.ply").stat().st_mtime == mesh_time
    views = json.loads((session.last_scan_dir / "L" / "views.json").read_text(encoding="utf-8"))
    assert all(v["k"] != c["k"] for v in views)                       # 가까운 사진은 메시 데이터에 넣지 않는다
    assert session.state()["job"]["kind"] == "closeup"


@pytest.mark.slow
def test_scan_cancel_reconnect(site):
    cfg, fleet, session = site
    n_abort = sum(1 for e in fleet.stop_log if e[1] == "abort")
    job = session.start_scan()
    wait(lambda: job.pair is not None and job.pair.phase in ("enter", "views"), 300, "진입")
    session.cancel_job()
    wait(lambda: job.state != "running", 120, "취소 끝")
    assert job.state == "cancelled"
    assert sum(1 for e in fleet.stop_log if e[1] == "abort") >= n_abort + 2
    with pytest.raises(NotReady):
        session.pause()
    wait(lambda: all(s.connected() for s in session.slots.values()), 60, "다시 연결")
    assert session.state()["pair"]["can_start"] or session.state()["pair"]["gap_deg"] is None


@pytest.mark.slow
def test_obstacle_alarm(tmp_path):
    """원 경로에 상자: 주행이 막혀 기다리는 동안 W06 경고, 다시 움직이면 해제, 스캔은 끝난다(L 한 대)."""
    cfg = load_cfg()
    cfg["capture"].update(json.loads(json.dumps(SMALL)))
    cfg_path = tmp_path / "scan.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    clock = Clock(SCALE)
    fleet = FakeFleet(cfg, clock, roles=("L",), obstacle=2)
    session = Session(cfg, cfg_path, fleet.start(), clock, data_dir=tmp_path / "scans", roles=("L",))
    session.start()
    try:
        seen = []
        job = session.start_scan()

        def watch():
            if session.slots["L"].obstacle is not None:
                seen.append(session.state()["robots"]["L"]["obstacle"])
            return job.state != "running"
        wait(watch, 900, "장애물 스캔 끝")
        assert job.state == "done", job.message
        assert seen and seen[0]["event"] in ("wait", "stop")
        items = session.alarms.snapshot()["items"]
        w06 = [a for a in items if a["code"] == "W06"]
        assert w06 and not any(a["active"] for a in w06)            # 끝나면 해제
    finally:
        session.close()
        fleet.stop()
