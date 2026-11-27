#!/usr/bin/env python3
"""스캔 결과 메시를 브라우저에서 3D로 본다(보기 전용). 표준 라이브러리만(호스트 python3).

웹 화면(scan3d/web, docs/specs/webui.md)에서 로봇 조작을 뺀 것. 로봇까지는 `tools/webui.sh`.

  tools/view_web.py                       http://127.0.0.1:8000 (이 컴퓨터에서만)
  tools/view_web.py --host 0.0.0.0        같은 Wi-Fi의 다른 기기에서도 http://<이 PC 주소>:8000
  tools/view_web.py --file some/mesh.ply  scans/ 밖의 PLY도 목록에 넣는다(여러 번 줄 수 있음)
  tools/view_web.py --open                브라우저를 연다
"""
import argparse
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d.web import results, server  # noqa: E402  표준 라이브러리만 쓰는 모듈

# tests와 tools/archive_scan.py가 쓰는 이름
OUT_DIR, NAMES = results.OUT_DIR, results.NAMES
list_entries, public, read_report, default_roots = (results.list_entries, results.public,
                                                    results.read_report, results.default_roots)


def serve(roots, host="127.0.0.1", port=8000, extra_files=()):
    """보기 전용 서버(serve_forever는 부르는 쪽이). port 0이면 빈 포트."""
    return server.serve(server.App(roots, extra_files), host, port)


def main(argv=None):
    ap = argparse.ArgumentParser(description="스캔 결과 메시를 브라우저에서 3D로 본다(보기 전용)")
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
