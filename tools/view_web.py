#!/usr/bin/env python3
"""스캔 결과 메시를 브라우저(localhost)에서 3D로 본다. 표준 라이브러리만 쓴다.

`scans/<id>/out*/mesh.ply`(없으면 `cloud.ply`)와 `results/`(tools/archive_scan.py 보관본)를 찾아 목록으로 내보내고,
뷰어(tools/web/)가 WebGL2로 그린다.
뷰어는 외부 라이브러리를 받지 않으므로 인터넷이 없어도 된다. 새 스캔이 끝나면 뷰어가 최신 결과로 바꿔 보여 준다.

  tools/view_web.py                       http://127.0.0.1:8000 (이 컴퓨터에서만)
  tools/view_web.py --host 0.0.0.0        같은 Wi-Fi의 다른 기기에서도 http://<이 PC 주소>:8000
  tools/view_web.py --file some/mesh.ply  scans/ 밖의 PLY도 목록에 넣는다(여러 번 줄 수 있음)
  tools/view_web.py --open                브라우저를 연다
"""
import argparse
import json
import re
import sys
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
WEB = Path(__file__).resolve().parent / "web"
STATIC = {"/": ("index.html", "text/html; charset=utf-8"),
          "/index.html": ("index.html", "text/html; charset=utf-8"),
          "/viewer.js": ("viewer.js", "text/javascript; charset=utf-8"),
          "/ply.js": ("ply.js", "text/javascript; charset=utf-8")}
NAMES = {"mesh.ply": "application/octet-stream", "cloud.ply": "application/octet-stream",
         "report.txt": "text/plain; charset=utf-8"}
OUT_DIR = re.compile(r"out(_[\w-]+)?")
DIMS = re.compile(r"dims_mm:\s*w\s*([\d.]+)\s+d\s*([\d.]+)\s+h\s*([\d.]+)")
ELAPSED = re.compile(r"elapsed_s:\s*([\d.]+)")


def default_roots():
    """[config/scan.yaml의 station.data_dir(못 읽으면 scans), results(tools/archive_scan.py 보관본)]."""
    try:
        import yaml
        cfg = yaml.safe_load((ROOT / "config" / "scan.yaml").read_text(encoding="utf-8"))
        data = ROOT / cfg["station"]["data_dir"]
    except Exception:  # noqa: BLE001  yaml이 없어도 기본 폴더로 뜬다
        data = ROOT / "scans"
    return [data, ROOT / "results"]


def read_report(path):
    """report.txt → (dims_mm [w, d, h] 또는 None, elapsed_s 또는 None)."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError:
        return None, None
    m, e = DIMS.search(text), ELAPSED.search(text)
    return ([float(v) for v in m.groups()] if m else None), (float(e.group(1)) if e else None)


def _entry(eid, label, files, report):
    ply = files.get("mesh.ply") or files.get("cloud.ply")
    dims, elapsed = read_report(report) if report else (None, None)
    if report:
        files = {**files, "report.txt": report}
    return {"id": eid, "label": label, "files": files, "mtime": ply.stat().st_mtime,
            "dims_mm": dims, "elapsed_s": elapsed}


def _note(scan):
    """tools/archive_scan.py가 남긴 summary.json의 메모."""
    try:
        return json.loads((scan / "summary.json").read_text(encoding="utf-8")).get("note")
    except (OSError, ValueError):
        return None


def list_entries(roots, extra_files=()):
    """보여 줄 결과 목록(최신 먼저). roots는 스캔 폴더 하나 또는 여러 개(scans/, results/).

    항목 id는 `<폴더 이름>/<스캔>/<out 폴더>`이고, files는 {이름: 경로}이며 이름은 NAMES 안에서만이다.
    """
    entries = []
    roots = [roots] if isinstance(roots, (str, Path)) else roots
    for root in map(Path, roots):
        if not root.is_dir():
            continue
        for scan in root.iterdir():
            if not scan.is_dir() or scan.name.startswith("."):
                continue
            note = _note(scan)
            for out in scan.iterdir():
                if not (out.is_dir() and OUT_DIR.fullmatch(out.name)):
                    continue
                files = {n: out / n for n in ("mesh.ply", "cloud.ply") if (out / n).is_file()}
                if files:
                    report = out / "report.txt"
                    label = f"{root.name}/{scan.name} · {out.name}" + (f" — {note}" if note else "")
                    entries.append(_entry(f"{root.name}/{scan.name}/{out.name}", label, files,
                                          report if report.is_file() else None))
    for i, f in enumerate(extra_files):
        f = Path(f).resolve()
        if f.is_file():
            report = f.parent / "report.txt"
            entries.append(_entry(f"file{i}", str(f), {"mesh.ply": f}, report if report.is_file() else None))
    entries.sort(key=lambda e: e["mtime"], reverse=True)
    return entries


def public(entry):
    """브라우저에 보내는 항목(경로는 빼고 파일 이름만)."""
    out = {k: v for k, v in entry.items() if k != "files"}
    out["files"] = sorted(entry["files"])
    return out


def make_handler(roots, extra_files=()):
    class Handler(BaseHTTPRequestHandler):
        server_version = "scan3d-view"

        def log_message(self, fmt, *args):  # 요청마다 찍지 않는다
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _error(self, code, msg):
            self._send(code, msg.encode("utf-8"), "text/plain; charset=utf-8")

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            url = urlparse(self.path)
            try:
                if url.path in STATIC:
                    name, ctype = STATIC[url.path]
                    return self._send(HTTPStatus.OK, (WEB / name).read_bytes(), ctype)
                if url.path == "/api/scans":
                    body = json.dumps([public(e) for e in list_entries(roots, extra_files)], ensure_ascii=False)
                    return self._send(HTTPStatus.OK, body.encode("utf-8"), "application/json; charset=utf-8")
                if url.path == "/files":
                    q = parse_qs(url.query)
                    eid, name = q.get("id", [""])[0], q.get("name", [""])[0]
                    if name not in NAMES:
                        return self._error(HTTPStatus.NOT_FOUND, "unknown file name")
                    # id는 지금 목록에 있는 것만 받는다(경로를 조립하지 않으므로 폴더 밖으로 나갈 수 없다)
                    entry = next((e for e in list_entries(roots, extra_files) if e["id"] == eid), None)
                    if entry is None or name not in entry["files"]:
                        return self._error(HTTPStatus.NOT_FOUND, "not found")
                    return self._send(HTTPStatus.OK, Path(entry["files"][name]).read_bytes(), NAMES[name])
                return self._error(HTTPStatus.NOT_FOUND, "not found")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def serve(roots, host="127.0.0.1", port=8000, extra_files=()):
    """서버를 만들어 돌려준다(serve_forever는 부르는 쪽이). port 0이면 빈 포트."""
    return ThreadingHTTPServer((host, port), make_handler(roots, extra_files))


def main(argv=None):
    ap = argparse.ArgumentParser(description="스캔 결과 메시를 브라우저에서 3D로 본다")
    ap.add_argument("--root", action="append", default=None,
                    help="스캔 폴더(여러 번 줄 수 있음. 기본: config station.data_dir와 results/)")
    ap.add_argument("--file", action="append", default=[], help="목록에 더할 PLY 파일")
    ap.add_argument("--host", default="127.0.0.1", help="다른 기기에서도 보려면 0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--open", action="store_true", help="브라우저를 연다")
    args = ap.parse_args(argv)
    roots = [Path(r) for r in args.root] if args.root else default_roots()
    httpd = serve(roots, args.host, args.port, args.file)
    port = httpd.server_address[1]
    url = f"http://{'127.0.0.1' if args.host in ('0.0.0.0', '') else args.host}:{port}"
    n = len(list_entries(roots, args.file))
    print(f"뷰어: {url}  (폴더 {', '.join(map(str, roots))}, 결과 {n}개, 끝내려면 Ctrl-C)", flush=True)
    if args.host in ("0.0.0.0", ""):
        print(f"다른 기기에서는 http://<이 PC 주소>:{port}", flush=True)
    if args.open:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
