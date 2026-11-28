"""웹 화면 HTTP 서버(docs/specs/webui.md). 이 파일은 표준 라이브러리만 import한다.

session이 없으면 보기 전용(`mode: "view"`, tools/view_web.py): 결과 목록과 메시만.
session이 있으면 로봇 연결·미리보기·스캔·보강 촬영 요청도 받는다(python -m scan3d.web).
"""
import ipaddress
import json
import re
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from . import results

STATIC_DIR = Path(__file__).resolve().parent / "static"
STATIC = {"/": "index.html", "/index.html": "index.html", "/app.js": "app.js", "/meshview.js": "meshview.js",
          "/stageview.js": "stageview.js", "/ply.js": "ply.js", "/style.css": "style.css"}
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8"}
MAX_BODY = 1 << 20
PREVIEW = re.compile(r"/api/preview/([A-Za-z]+)\.jpg")
FOCUS_IMG = re.compile(r"/api/focus/(\d+)\.jpg")


class App:
    """서버가 쓰는 것: 결과 폴더, 더 넣을 PLY, 세션(없으면 보기 전용), 바닥 자르기 높이."""

    def __init__(self, roots, extra_files=(), session=None, floor_cut=0.005):
        self.roots, self.extra_files, self.session, self.floor_cut = list(roots), list(extra_files), session, floor_cut


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        server_version = "scan3d-web"

        def log_message(self, fmt, *args):  # 요청마다 찍지 않는다
            pass

        # ---- 응답 ----
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, obj, code=HTTPStatus.OK):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

        def _error(self, code, error, message=""):
            self._json({"error": error, "message": message}, code)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_BODY:
                raise ValueError("body too large")
            raw = self.rfile.read(n) if n else b""
            return json.loads(raw.decode("utf-8")) if raw.strip() else {}

        # ---- GET ----
        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            s = app.session
            try:
                if url.path in STATIC:
                    f = STATIC_DIR / STATIC[url.path]
                    return self._send(HTTPStatus.OK, f.read_bytes(), TYPES[f.suffix])
                if url.path == "/api/state":
                    return self._json(s.state() if s else {"mode": "view"})
                if url.path == "/api/scans":
                    return self._json([results.public(e) for e in results.list_entries(app.roots, app.extra_files)])
                if url.path == "/files":
                    name = q.get("name", [""])[0]
                    if name not in results.NAMES:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found", "unknown file name")
                    entry = results.find(app.roots, q.get("id", [""])[0], app.extra_files)
                    if entry is None or name not in entry["files"]:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found")
                    return self._send(HTTPStatus.OK, Path(entry["files"][name]).read_bytes(), results.NAMES[name])
                if url.path == "/api/holes":
                    entry = results.find(app.roots, q.get("id", [""])[0], app.extra_files)
                    if entry is None or "mesh.ply" not in entry["files"]:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found")
                    try:
                        from . import holes
                    except ImportError as e:
                        return self._error(HTTPStatus.NOT_IMPLEMENTED, "unavailable", f"open3d·numpy 필요: {e}")
                    try:
                        found = holes.holes_from_ply(entry["files"]["mesh.ply"], app.floor_cut)
                    except Exception as e:  # noqa: BLE001  연결을 끊지 않고 이유를 보인다
                        return self._error(HTTPStatus.INTERNAL_SERVER_ERROR, "holes_failed", f"{type(e).__name__}: {e}")
                    return self._json({"holes": found})
                if url.path == "/api/coverage":
                    entry = results.find(app.roots, q.get("id", [""])[0], app.extra_files)
                    if entry is None or "mesh.ply" not in entry["files"]:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found")
                    try:
                        from . import coverage
                    except ImportError as e:
                        return self._error(HTTPStatus.NOT_IMPLEMENTED, "unavailable", f"open3d·numpy 필요: {e}")
                    cov = coverage.coverage_for(entry["files"]["mesh.ply"])
                    if cov is None:
                        return self._error(HTTPStatus.NOT_FOUND, "no_dataset", "깊이 사진이 없는 결과(보관본)")
                    return self._json(cov)
                m = PREVIEW.fullmatch(url.path)
                if m and s:
                    slot = s.slots.get(m.group(1))
                    if slot is None or not slot.jpeg:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found")
                    return self._send(HTTPStatus.OK, slot.jpeg, "image/jpeg")
                m = FOCUS_IMG.fullmatch(url.path)
                if m and s:
                    data = s.focus_image(int(m.group(1)))
                    if data is None:
                        return self._error(HTTPStatus.NOT_FOUND, "not_found")
                    return self._send(HTTPStatus.OK, data, "image/jpeg")
                return self._error(HTTPStatus.NOT_FOUND, "not_found")
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:  # noqa: BLE001  다시 합치는 중 바뀐 파일 등
                try:
                    self._error(HTTPStatus.INTERNAL_SERVER_ERROR, "error", f"{type(e).__name__}: {e}")
                except OSError:
                    pass

        # ---- POST ----
        def _same_origin(self):
            """로봇을 움직이는 요청은 이 화면에서 온 JSON만 받는다.

            application/json이면 다른 사이트 요청은 사전 요청(OPTIONS)이 필요하고 이 서버는 허락하지 않는다.
            Origin이 있으면 Host와 같아야 한다.
            """
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            if ctype != "application/json":
                return False, "Content-Type: application/json만 받는다"
            origin = self.headers.get("Origin")
            host = self.headers.get("Host") or ""
            if origin and urlparse(origin).netloc != host:
                return False, "다른 출처의 요청"
            # DNS 재바인딩은 Origin == Host라 이름으로 막는다(화면은 IP나 localhost로만 연다)
            name = urlparse(f"//{host}").hostname or ""
            if name != "localhost":
                try:
                    ipaddress.ip_address(name)
                except ValueError:
                    return False, f"주소 {name!r}로는 로봇을 움직이지 않는다(IP나 localhost로 연다)"
            return True, ""

        def do_POST(self):
            s = app.session
            if s is None:
                return self._error(HTTPStatus.NOT_FOUND, "view_only", "보기 전용 서버")
            ok, why = self._same_origin()
            if not ok:
                return self._error(HTTPStatus.FORBIDDEN, "forbidden", why)
            from .focus import PlanError
            from .session import Busy, NotReady
            path = urlparse(self.path).path
            try:
                body = self._body()
            except (ValueError, UnicodeDecodeError):
                return self._error(HTTPStatus.BAD_REQUEST, "bad_request", "JSON 본문")
            if not isinstance(body, dict):
                return self._error(HTTPStatus.BAD_REQUEST, "bad_request", "JSON 객체 본문")
            try:
                if path == "/api/connect":
                    s.connect()
                elif path == "/api/preview":
                    s.set_preview(bool(body.get("on")))
                elif path == "/api/scan/start":
                    s.start_scan()
                elif path == "/api/job/pause":
                    s.pause()
                elif path == "/api/job/resume":
                    s.resume()
                elif path == "/api/job/cancel":
                    s.cancel_job()
                elif path == "/api/robot/stack":
                    s.stack_action(str(body.get("role", "")), str(body.get("action", "")))
                elif path == "/api/robot/stack/start_all":
                    s.start_stacks()
                elif path == "/api/recipe":
                    s.set_recipe(str(body.get("name", "")))
                elif path == "/api/alarm/ack":
                    aid = body.get("id")
                    if aid is not None and (not isinstance(aid, int) or isinstance(aid, bool)):
                        return self._error(HTTPStatus.BAD_REQUEST, "bad_request", "id는 정수")
                    s.alarms.ack(aid)
                elif path == "/api/focus/start":
                    s.start_focus(body.get("markers"), body.get("regions"))
                elif path == "/api/closeup/start":
                    s.start_closeup(body.get("markers"))
                elif path == "/api/sim/obstacle":
                    view = body.get("view")
                    if view is not None and (not isinstance(view, int) or isinstance(view, bool)):
                        return self._error(HTTPStatus.BAD_REQUEST, "bad_request", "view는 정수")
                    s.set_obstacle(bool(body.get("on", True)), view)
                else:
                    return self._error(HTTPStatus.NOT_FOUND, "not_found")
            except Busy as e:
                return self._error(HTTPStatus.CONFLICT, "busy", str(e))
            except NotReady as e:
                return self._error(HTTPStatus.CONFLICT, "not_ready", str(e))
            except PlanError as e:
                return self._error(HTTPStatus.BAD_REQUEST, "bad_request", str(e))
            except (BrokenPipeError, ConnectionResetError):
                return None
            except Exception as e:  # noqa: BLE001  연결을 끊지 않고 이유를 보인다
                return self._error(HTTPStatus.INTERNAL_SERVER_ERROR, "error", f"{type(e).__name__}: {e}")
            return self._json(s.state())

    return Handler


def serve(app, host="127.0.0.1", port=8080):
    """서버를 만들어 돌려준다(serve_forever는 부르는 쪽이). port 0이면 빈 포트."""
    return ThreadingHTTPServer((host, port), make_handler(app))
