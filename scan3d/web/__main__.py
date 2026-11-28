"""웹 화면 서버 실행(docs/specs/webui.md).

  python -m scan3d.web                      config 로봇에 연결, http://127.0.0.1:8080
  python -m scan3d.web --host 0.0.0.0       같은 Wi-Fi의 다른 기기에서도
  python -m scan3d.web --fake               가짜 로봇 2대(로봇 없이 화면 전체 시험)
"""
import argparse
import sys
import threading
import webbrowser

from ..clock import Clock
from ..config import ORDER, load_config, robot_addr
from . import results, server
from .session import Session


def main(argv=None):
    ap = argparse.ArgumentParser(description="웹 화면 하나로 스캔")
    ap.add_argument("--config", default=None)
    ap.add_argument("--host", default="127.0.0.1", help="다른 기기에서도 보려면 0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--only", choices=ORDER, help="한 대만")
    ap.add_argument("--fake", action="store_true", help="가짜 로봇 2대(로봇 없이 시험)")
    ap.add_argument("--obstacle", type=int, default=None, help="--fake에서 L 시점 k·k+1 사이에 상자")
    ap.add_argument("--time-scale", type=float, default=None, help="--fake 시계 배속(기본 5)")
    ap.add_argument("--data-dir", default=None, help="스캔 저장 폴더(기본 config station.data_dir)")
    ap.add_argument("--open", action="store_true", help="브라우저를 연다")
    args = ap.parse_args(argv)
    cfg_path, cfg = load_config(args.config)
    roles = (args.only,) if args.only else ORDER
    fleet = None
    if args.fake:
        from ..sim.fake_agent import FakeFleet
        clock = Clock(args.time_scale or 5.0)
        fleet = FakeFleet(cfg, clock, roles=roles, obstacle=args.obstacle)
        addrs = fleet.start()
    else:
        clock = Clock(args.time_scale or 1.0)
        addrs = {r: robot_addr(cfg, r) for r in roles}
    session = Session(cfg, cfg_path, addrs, clock, data_dir=args.data_dir, roles=roles, fake=fleet is not None,
                      fleet=fleet)
    roots = [session.data_dir] + [r for r in results.default_roots(cfg)[1:]]
    app = server.App(roots, session=session, floor_cut=float(cfg["scene"]["floor_cut"]))
    httpd = server.serve(app, args.host, args.port)
    url = f"http://{'127.0.0.1' if args.host in ('0.0.0.0', '') else args.host}:{httpd.server_address[1]}"
    print(f"웹 화면: {url}{'  (가짜 로봇)' if fleet else ''}  끝내려면 Ctrl-C", flush=True)
    if args.host in ("0.0.0.0", ""):
        print(f"다른 기기에서는 http://<이 PC 주소>:{httpd.server_address[1]}", flush=True)
    threading.Thread(target=session.start, daemon=True).start()
    if args.open:
        threading.Timer(0.8, webbrowser.open, args=(url,)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        session.close()
        if fleet is not None:
            fleet.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
