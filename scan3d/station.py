"""스캔 PC 주 프로그램(docs/specs/station.md).

  연결(robot.link) → 두 대 맞춤 진행(pair.coordinator) → 로봇마다 데이터셋(dataset) → 두 대 융합 재구성(pair.recon) → 표시

  python -m scan3d.station [--only L|H] [--hello] [--fake] [--no-window]
"""
import argparse
import sys
import threading
import time
from pathlib import Path

from . import dataset
from .clock import Clock
from .config import ORDER, ROOT, load_board, load_config, robot_addr
from .pair import recon
from .pair.coordinator import PairScan
from .robot import link as robot_link
from .robot.unit import StartError


def run(cfg, links, writer, board, cancel=None, t0=None, pair=None):
    """links로 스캔 한 번. 성공하면 (scan_dir, recon 결과, 보고서).

    pair를 주면 그 PairScan으로 진행(웹 화면이 진행 표시·일시정지용으로 미리 만든다).
    """
    cancel = cancel or threading.Event()
    t0 = time.monotonic() if t0 is None else t0
    pair = pair or PairScan(cfg, links, writer, board, cancel)
    try:
        pair.run()
    except BaseException as e:
        reason = f"{type(e).__name__}: {e}"
        pair.abort_all(reason)
        writer.report(pair.report(aborted=reason, elapsed_s=time.monotonic() - t0))
        raise
    scan_dir = Path(writer.scan_dir)
    for r in pair.roles:                       # 로봇마다
        dataset.build(scan_dir, r)
    result = recon.reconstruct([scan_dir / r for r in pair.roles], cfg)   # 두 대 합침
    report = pair.report(elapsed_s=time.monotonic() - t0)
    missing = [r for r in pair.roles if not result.used.get(r)]
    if missing:
        report["aborted"] = f"융합한 시점 없음: {missing}"
        writer.report(report)
        raise RuntimeError(report["aborted"])
    report["dims_mm"] = [round(float(x), 1) for x in result.dims_mm]
    report["elapsed_s"] = round(time.monotonic() - t0, 1)
    recon.write_outputs(scan_dir, result, report)
    writer.report(report)
    return writer.scan_dir, result, report


def show_mesh(mesh):
    import open3d as o3d
    o3d.visualization.draw_geometries([mesh], window_name="scan3d")


def execute(cfg, cfg_path, addrs, clock, roles=ORDER, no_window=True, data_dir=None):
    """연결부터 끝까지. addrs는 {role: (host, port, stream_port)}. 반환 (종료 코드, scan_dir)."""
    t0 = time.monotonic()
    board_path, board = load_board(cfg)
    writer = dataset.ScanWriter(Path(data_dir or ROOT / cfg["station"]["data_dir"]), cfg_path, board_path)
    cancel = threading.Event()
    links = {}
    try:
        for r in roles:
            links[r] = robot_link.connect(cfg, r, writer.role(r), clock, cancel, addr=addrs[r])
        scan_dir, result, report = run(cfg, links, writer, board, cancel, t0)
    except StartError as e:
        print(f"시작하지 않음: {e}", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    except KeyboardInterrupt:
        print("중단: Ctrl-C", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    except Exception as e:  # noqa: BLE001  원인을 화면과 report.json에 남기고 끝낸다
        print(f"중단: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
        return 1, writer.scan_dir
    finally:
        for lk in links.values():
            lk.close()
        writer.close()
    dims = result.dims_mm
    print(f"치수(mm): {dims[0]:.0f} × {dims[1]:.0f} × {dims[2]:.0f}, 걸린 시간 {report['elapsed_s']:.0f} s",
          flush=True)
    if not no_window and cfg["station"].get("show_mesh", True):
        show_mesh(result.mesh)
    return 0, scan_dir


def hello_only(cfg, roles):
    """로봇마다 연결과 hello만 하고 role·일련번호·보정값을 출력한다."""
    code = 0
    for r in roles:
        try:
            lk = robot_link.connect(cfg, r)
        except OSError as e:
            print(f"{r}: 연결 실패 {e}", flush=True)
            code = 1
            continue
        try:
            reply = lk.hello()
            c, d = reply["calib"]["color"], reply["calib"]["depth"]
            print(f"{r}: role {reply['role']}, serial {reply['serial']}, "
                  f"color {c['w']}x{c['h']} fx {c['fx']:.1f} fy {c['fy']:.1f} {c['model']}, "
                  f"depth {d['w']}x{d['h']} fx {d['fx']:.1f}, depth_scale {reply['calib']['depth_scale']}",
                  flush=True)
            if reply["role"] != r:
                print(f"{r}: role이 config와 다름", flush=True)
                code = 1
        except Exception as e:  # noqa: BLE001
            print(f"{r}: hello 실패 {e}", flush=True)
            code = 1
        finally:
            lk.close()
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(description="두 로봇 3D 스캔")
    ap.add_argument("--config", default=str(ROOT / "config" / "scan.yaml"))
    ap.add_argument("--only", choices=ORDER)
    ap.add_argument("--fake", action="store_true", help="가짜 에이전트 2대로 실행")
    ap.add_argument("--obstacle", type=int, default=None, help="--fake에서 L 시점 k·k+1 사이에 상자")
    ap.add_argument("--time-scale", type=float, default=None)
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--hello", action="store_true", help="연결과 hello만 하고 role·일련번호·보정값을 출력")
    args = ap.parse_args(argv)
    cfg_path, cfg = load_config(args.config)
    roles = (args.only,) if args.only else ORDER
    if args.hello:
        return hello_only(cfg, roles)
    fleet = None
    if args.fake:
        from .sim.fake_agent import FakeFleet
        clock = Clock(args.time_scale or 5.0)
        fleet = FakeFleet(cfg, clock, roles=roles, obstacle=args.obstacle)
        addrs = fleet.start()
    else:
        clock = Clock(args.time_scale or 1.0)
        addrs = {r: robot_addr(cfg, r) for r in roles}
    try:
        code, scan_dir = execute(cfg, cfg_path, addrs, clock, roles, no_window=args.no_window)
    finally:
        if fleet is not None:
            fleet.stop()
    print(f"데이터: {scan_dir}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
