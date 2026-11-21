#!/usr/bin/env python3
"""두 로봇 뒤 LED를 같은 순간에 바꿔, 스캔 PC에서 두 대에 동시에 명령이 닿는지 본다(주행 없음). 스캔 PC의 저장소 폴더에서 실행한다.

  tools/led_check.py               L 초록·H 파랑 → 둘 다 흰색으로 3번 깜빡임 → 끔
  tools/led_check.py --robot H     한 대만

로봇마다 SSH로 파이썬(pinkylib.LED)을 하나씩 띄워 두고, 두 대에 같은 순간 명령을 보내 응답 시간을 잰다.
"""
import argparse
import queue
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d import config  # noqa: E402

COLOR = {"L": (0, 120, 0), "H": (0, 0, 120)}
WHITE = (90, 90, 90)
ROBOT_CODE = """
import sys
{led}
print("ready", flush=True)
for line in sys.stdin:
    p = line.split()
    if p and p[0] == "fill":
        led.fill(tuple(int(v) for v in p[1:4]))
    elif p and p[0] == "clear":
        led.clear()
    print("ok", flush=True)
led.clear()
"""
PINKY_LED = "from pinkylib import LED\nled = LED()"


def ssh_argv(host):
    code = ROBOT_CODE.format(led=PINKY_LED)
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, "python3 -u -c " + shlex.quote(code)]


class Robot:
    def __init__(self, role, argv):
        self.role = role
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  text=True, bufsize=1)
        self.lines = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            self.lines.put((time.monotonic(), line.strip()))
        self.lines.put((time.monotonic(), None))

    def wait(self, want, timeout):
        t, line = self.lines.get(timeout=timeout)
        if line is None:
            raise ConnectionError(f"{self.role}: 연결 끊김 {self.p.stderr.read().strip()[-300:]}")
        if line != want:
            raise RuntimeError(f"{self.role}: 예상 {want!r}, 받음 {line!r}")
        return t

    def send(self, text):
        self.p.stdin.write(text + "\n")
        self.p.stdin.flush()

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self.p.kill()


def step(robots, cmds, rtts):
    """모든 로봇에 같은 순간 명령을 보내고 응답까지 걸린 시간을 rtts[role]에 붙인다."""
    t0 = time.monotonic()
    for r in robots:
        r.send(cmds[r.role])
    for r in robots:
        rtts[r.role].append(r.wait("ok", 5.0) - t0)


def fill(rgb):
    return "fill {} {} {}".format(*rgb)


def main(argv=None, spawn=None):
    ap = argparse.ArgumentParser(description="두 로봇 LED 동시 명령 확인")
    ap.add_argument("--config", default=str(ROOT / "config" / "scan.yaml"))
    ap.add_argument("--robot", choices=config.ORDER)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--hold", type=float, default=2.0, help="역할 색을 켜 두는 시간 s")
    args = ap.parse_args(argv)
    _, cfg = config.load_config(args.config)
    roles = [args.robot] if args.robot else list(config.ORDER)
    spawn = spawn or (lambda role, host: ssh_argv(host))

    robots = [Robot(r, spawn(r, cfg["robots"][r]["host"])) for r in roles]
    rtts = {r: [] for r in roles}
    try:
        for r in robots:
            r.wait("ready", 20.0)
        step(robots, {r: fill(COLOR[r]) for r in roles}, rtts)
        time.sleep(args.hold)
        for _ in range(args.cycles):
            step(robots, {r: fill(WHITE) for r in roles}, rtts)
            time.sleep(0.4)
            step(robots, {r: "clear" for r in roles}, rtts)
            time.sleep(0.4)
    except (queue.Empty, ConnectionError, RuntimeError, OSError) as e:
        print(f"FAIL {e or '응답 없음'}", flush=True)
        return 1
    finally:
        for r in robots:
            r.close()

    for r in roles:
        v = rtts[r]
        print(f"{r}({cfg['robots'][r]['host']}): 명령 {len(v)}개, 응답 평균 {1000 * sum(v) / len(v):.0f} ms, "
              f"최대 {1000 * max(v):.0f} ms", flush=True)
    if len(roles) == 2:
        gap = max(abs(a - b) for a, b in zip(*rtts.values()))
        print(f"두 대 응답 차 최대 {1000 * gap:.0f} ms", flush=True)
    worst = max(max(v) for v in rtts.values())
    print(("PASS" if worst < 0.5 else "SLOW") + " — 뒤 LED가 역할 색으로 켜졌다가 함께 흰색으로 깜빡였는지 눈으로 본다", flush=True)
    return 0 if worst < 0.5 else 1


if __name__ == "__main__":
    sys.exit(main())
