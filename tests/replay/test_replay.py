"""현장 원본 재생 테스트. tests/data/*/ 또는 scans/*/ 아래 로봇에서 받은 raw/로 돈다(없으면 건너뜀).

다른 폴더는 SCAN3D_REPLAY=<scans/<scan_id> 경로>로 준다.
"""
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pytest

from scan3d import dataset


from scan3d.robot import align, tagpose

ROOT = Path(__file__).resolve().parents[2]


def _scan_dirs():
    if os.environ.get("SCAN3D_REPLAY"):
        return [Path(os.environ["SCAN3D_REPLAY"])]
    found = []
    for base in (ROOT / "tests" / "data", ROOT / "scans"):
        if base.is_dir():
            found += sorted(p for p in base.iterdir() if p.is_dir() and list(p.glob("*/raw/calib.json")))
    return found


def _snapshots():
    out = []
    for d in _scan_dirs():
        for calib in sorted(d.glob("*/raw/calib.json")):
            for head in sorted(calib.parent.glob("[0-9]*.json")):
                out.append(pytest.param(d, head, id=f"{d.name}-{calib.parent.parent.name}-{head.stem}"))
    return out


SNAPS = _snapshots()
pytestmark = pytest.mark.skipif(not SNAPS, reason="재생할 현장 원본 없음")


def _load(head_path):
    raw = head_path.parent
    head = json.loads(head_path.read_text(encoding="utf-8"))
    calib = json.loads((raw / "calib.json").read_text(encoding="utf-8"))["calib"]
    payloads = []
    for key in ("color", "depth"):
        if key in head:
            payloads.append((raw / f"{head_path.stem}_{key}.{head[key]['fmt']}").read_bytes())
    return head, payloads, calib


@pytest.mark.parametrize("scan_dir,head_path", SNAPS or [(None, None)])
def test_snapshot_decodes(scan_dir, head_path):
    head, payloads, calib = _load(head_path)
    assert [len(p) for p in payloads] == [head["color"]["bytes"], head["depth"]["bytes"]]
    frame = align.prepare(head, payloads, calib)
    assert frame.color.shape == (calib["color"]["h"], calib["color"]["w"], 3)
    assert frame.depth.shape == frame.color.shape[:2]
    assert np.count_nonzero(frame.depth) / frame.depth.size > 0.2
    board_json = json.loads((scan_dir / "board.json").read_text(encoding="utf-8"))
    import yaml
    cfg = yaml.safe_load((scan_dir / "config.yaml").read_text(encoding="utf-8"))
    pose = tagpose.estimate(frame.color, frame.K, tagpose.load_board(board_json, cfg["board"]["tag_mm"]))
    if pose is not None:
        assert pose.reproj_px < 2.0


@pytest.mark.parametrize("scan_dir", _scan_dirs() or [None])
def test_build(scan_dir, tmp_path):
    copy = tmp_path / scan_dir.name
    shutil.copytree(scan_dir, copy, ignore=shutil.ignore_patterns("color", "depth", "out"))
    for role_dir in sorted(p.parent.parent for p in copy.glob("*/raw/calib.json")):
        views = dataset.build(copy, role_dir.name)
        assert (role_dir / "outcomes.json").exists()
        assert isinstance(views, list)
