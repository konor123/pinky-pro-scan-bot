"""Headless MuJoCo probe: full IR-bounded boustrophedon on a raised 2.4 x 1.2 m panel."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe as P  # noqa: E402  (reuses make, ir_surface, fallen)

V, V_SLOW, W_MAX = float(sys.argv[2]), 0.03, 0.4
LATENCY, BACK, SPACING = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
TOOL = 0.10
CELL = 0.005


class Run:
    def __init__(self, start):
        self.p = P.make(start, P.HERE / 'out')
        self.t = 0.0
        self.track = []
        self.fell = False

    def step(self, v, w):
        self.p.step(v, w)
        self.t += P.DT
        x, y, th = self.p.pose()
        self.track.append((x, y, th))
        if P.fallen(self.p):
            self.fell = True
            raise RuntimeError('fell')

    def yaw(self):
        return self.p.pose()[2]

    def turn_to(self, target):
        for _ in range(4000):
            err = math.atan2(math.sin(target - self.yaw()), math.cos(target - self.yaw()))
            if abs(err) < math.radians(0.4) and abs(self.p.twist[1]) < 0.02:
                break
            self.step(0.0, max(-W_MAX, min(W_MAX, 3.0 * err)))
        for _ in range(20):
            self.step(0.0, 0.0)

    def forward(self, heading, v, max_dist=None):
        """Heading-hold drive. Returns 'EDGE' when IR lost the surface, or 'DIST'."""
        start = np.array(self.p.odom[:2])
        lost_at = None
        while True:
            if lost_at is None and not all(P.ir_surface(self.p)):
                lost_at = self.t
            stop = lost_at is not None and self.t - lost_at >= LATENCY
            err = math.atan2(math.sin(heading - self.yaw()), math.cos(heading - self.yaw()))
            if stop:
                self.step(0.0, 0.0)
                if abs(self.p.twist[0]) < 1e-4:
                    return 'EDGE'
            else:
                self.step(v, 2.0 * err)
                if max_dist is not None and lost_at is None and np.linalg.norm(np.array(self.p.odom[:2]) - start) >= max_dist:
                    for _ in range(20):
                        self.step(0.0, 0.0)
                    return 'DIST'

    def reverse(self, heading, dist):
        start = np.array(self.p.odom[:2])
        while np.linalg.norm(np.array(self.p.odom[:2]) - start) < dist:
            err = math.atan2(math.sin(heading - self.yaw()), math.cos(heading - self.yaw()))
            self.step(-V_SLOW, 2.0 * err)
        for _ in range(20):
            self.step(0.0, 0.0)


def coverage(track):
    nx, ny = int(2.4 / CELL), int(1.2 / CELL)
    grid = np.zeros((nx, ny), dtype=bool)
    xs = (np.arange(nx) + 0.5) * CELL - 1.2
    ys = (np.arange(ny) + 0.5) * CELL - 0.6
    last = None
    for x, y, th in track:
        if last is not None and math.hypot(x - last[0], y - last[1]) < 0.004 and abs(th - last[2]) < 0.02:
            continue
        last = (x, y, th)
        i0, i1 = max(0, int((x - 0.06 + 1.2) / CELL)), min(nx, int((x + 0.06 + 1.2) / CELL) + 1)
        j0, j1 = max(0, int((y - 0.06 + 0.6) / CELL)), min(ny, int((y + 0.06 + 0.6) / CELL) + 1)
        gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
        dx, dy = gx - x, gy - y
        along = dx * math.cos(th) + dy * math.sin(th)
        across = -dx * math.sin(th) + dy * math.cos(th)
        grid[i0:i1, j0:j1] |= (np.abs(along) <= 0.006) & (np.abs(across) <= TOOL / 2)
    return grid.mean()


def main():
    run = Run((-0.9, -0.3))
    lanes, note = 0, 'DONE'
    try:
        run.turn_to(math.pi)                       # face end edge A (x = -1.2)
        run.forward(math.pi, V_SLOW); run.reverse(math.pi, BACK)
        run.turn_to(-math.pi / 2)                  # face side edge (y = -0.6)
        run.forward(-math.pi / 2, V_SLOW); run.reverse(-math.pi / 2, BACK)
        heading, last = 0.0, False
        run.turn_to(heading)
        while True:
            run.forward(heading, V)                # lane until the end edge
            lanes += 1
            if last:
                run.reverse(heading, BACK)
                break
            run.reverse(heading, BACK)
            run.turn_to(math.pi / 2)               # face the far side edge
            if run.forward(math.pi / 2, V_SLOW, max_dist=SPACING) == 'EDGE':
                run.reverse(math.pi / 2, BACK)
                last = True
            heading = math.pi if heading == 0.0 else 0.0
            run.turn_to(heading)
            if lanes > 40:
                note = 'TOO MANY LANES'
                break
    except RuntimeError:
        note = 'FELL'
    x, y, th = run.p.pose()
    ys = sorted({round(t[1], 2) for t in run.track[::200]})
    print(f'v={V} latency={LATENCY} back={BACK} spacing={SPACING}: {note}, lanes={lanes}, time={run.t / 60:.1f} min, '
          f'end=({x:.3f},{y:.3f},{math.degrees(th):.0f}deg), coverage={100 * coverage(run.track):.1f}% of panel, '
          f'y-range=({min(t[1] for t in run.track):.3f},{max(t[1] for t in run.track):.3f}), x-range=({min(t[0] for t in run.track):.3f},{max(t[0] for t in run.track):.3f})',
          flush=True)


if __name__ == '__main__':
    main()
