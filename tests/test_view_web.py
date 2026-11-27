"""tools/view_web.py: 스캔 결과 목록과 파일 내보내기."""
import json
import os
import threading
import urllib.error
import urllib.request

import pytest

from tools import view_web

REPORT = "dims_mm: w 181.2  d 139.8  h 160.4\n\n[L] fused 24 views\n\nelapsed_s: 312.5\n"


def _scan(root, scan_id, out="out", files=("mesh.ply",), report=REPORT, mtime=None):
    d = root / scan_id / out
    d.mkdir(parents=True)
    for name in files:
        (d / name).write_bytes(f"ply {scan_id} {name}".encode())
        if mtime is not None:
            os.utime(d / name, (mtime, mtime))
    if report is not None:
        (d / "report.txt").write_text(report, encoding="utf-8")
    return d


def test_list_entries(tmp_path):
    root = tmp_path / "scans"
    _scan(root, "20261120-100000", mtime=1000)
    _scan(root, "20261121-100000", out="out_recon", report=None, mtime=3000)
    _scan(root, "20261122-100000", files=("cloud.ply",), mtime=2000)
    (root / "20261123-100000" / "out").mkdir(parents=True)                # 결과 없음
    (root / "20261123-100000" / "L").mkdir()                              # out 폴더가 아님
    _scan(root, "20261123-100000", out="outside", mtime=4000)             # out으로 시작할 뿐인 폴더
    entries = view_web.list_entries(root)
    assert [e["id"] for e in entries] == ["scans/20261121-100000/out_recon", "scans/20261122-100000/out",
                                          "scans/20261120-100000/out"]
    first, _, last = entries
    assert first["dims_mm"] is None and sorted(first["files"]) == ["mesh.ply"]
    assert last["dims_mm"] == [181.2, 139.8, 160.4] and last["elapsed_s"] == 312.5
    assert sorted(view_web.public(last)["files"]) == ["mesh.ply", "report.txt"]
    assert "20261123" not in json.dumps([view_web.public(e) for e in entries])


def test_list_entries_several_roots(tmp_path):
    _scan(tmp_path / "scans", "20261120-100000", mtime=1000)
    kept = _scan(tmp_path / "results", "20261120-100000", mtime=2000)
    (kept.parent / "summary.json").write_text('{"note": "돼지 인형"}', encoding="utf-8")
    entries = view_web.list_entries([tmp_path / "scans", tmp_path / "results"])
    assert [e["id"] for e in entries] == ["results/20261120-100000/out", "scans/20261120-100000/out"]
    assert entries[0]["label"].endswith("— 돼지 인형")


def test_list_entries_missing_root(tmp_path):
    assert view_web.list_entries(tmp_path / "none") == []


def test_extra_file(tmp_path):
    d = tmp_path / "elsewhere"
    d.mkdir()
    (d / "model.ply").write_bytes(b"ply x")
    (d / "report.txt").write_text(REPORT, encoding="utf-8")
    entries = view_web.list_entries(tmp_path / "scans", [d / "model.ply", d / "missing.ply"])
    assert len(entries) == 1
    assert entries[0]["id"] == "file0" and entries[0]["dims_mm"][2] == 160.4


@pytest.fixture
def server(tmp_path):
    _scan(tmp_path / "scans", "20261120-100000", files=("mesh.ply", "cloud.ply"))
    secret = tmp_path / "secret.txt"
    secret.write_text("x", encoding="utf-8")
    httpd = view_web.serve([tmp_path / "scans"], "127.0.0.1", 0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()
    httpd.server_close()


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


def test_http(server):
    status, ctype, body = _get(server + "/")
    assert status == 200 and ctype.startswith("text/html") and b"app.js" in body
    for path in ("/app.js", "/meshview.js", "/ply.js", "/style.css"):
        assert _get(server + path)[0] == 200
    assert json.loads(_get(server + "/api/state")[2]) == {"mode": "view"}
    _, _, body = _get(server + "/api/scans")
    entries = json.loads(body)
    assert entries[0]["id"] == "scans/20261120-100000/out"
    assert entries[0]["files"] == ["cloud.ply", "mesh.ply", "report.txt"]
    _, _, body = _get(server + "/files?id=scans/20261120-100000/out&name=mesh.ply")
    assert body == b"ply 20261120-100000 mesh.ply"
    _, ctype, body = _get(server + "/files?id=scans/20261120-100000/out&name=report.txt")
    assert ctype.startswith("text/plain") and body.decode() == REPORT


@pytest.mark.parametrize("path", [
    "/files?id=scans/20261120-100000/out&name=../../secret.txt",
    "/files?id=../secret.txt&name=mesh.ply",
    "/files?id=scans/20261120-100000/..&name=mesh.ply",
    "/files?id=nope/out&name=mesh.ply",
    "/../secret.txt",
    "/web/../../secret.txt",
])
def test_http_rejects(server, path):
    with pytest.raises(urllib.error.HTTPError) as e:
        _get(server + path)
    assert e.value.code == 404
