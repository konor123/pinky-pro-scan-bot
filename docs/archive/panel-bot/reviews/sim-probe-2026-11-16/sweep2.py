"""Headless MuJoCo probe: IR-bounded sweep with a heading offset, optional IR squaring and outer-lane offset."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe as P  # noqa: E402
from sweep import coverage  # noqa: E402

EPS = math.radians(float(sys.argv[2]))      # heading reference error (robot's idea of "panel x" is rotated by EPS)
ALIGN = sys.argv[3] == '1'                  # square to the edges with the left/right IR channels
BACK_END, BACK_SIDE = float(sys.argv[4]), float(sys.argv[5])
IR_PERIOD = int(sys.argv[6]) if len(sys.argv) > 6 else 2   # IR sampled every N x 10 ms
V, V_SLOW, V_ALIGN, W_MAX, LATENCY, SPACING = 0.08, 0.03, 0.015, 0.4, 0.08, 0.08
BASE = 0.04                                  # distance between left and right IR channels


class Run:
    def __init__(self, start):
        self.p = P.make(start, P.HERE / 'out')
        self.t, self.n, self.track, self.ir, self.deltas = 0.0, 0, [], [True, True, True], []

    def step(self, v, w):
        self.p.step(v, w)
        self.t += P.DT
        self.n += 1
        if self.n % IR_PERIOD == 0:
            self.ir = P.ir_surface(self.p)
        self.track.append(tuple(self.p.pose()))
        if P.fallen(self.p):
            raise RuntimeError('FELL')

    def yaw(self):
        return self.p.pose()[2]

    def err(self, target):
        return math.atan2(math.sin(target - self.yaw()), math.cos(target - self.yaw()))

    def settle(self, n=20):
        for _ in range(n):
            self.step(0.0, 0.0)

    def turn_to(self, target):
        for _ in range(4000):
            e = self.err(target)
            if abs(e) < math.radians(0.3) and abs(self.p.twist[1]) < 0.02:
                break
            self.step(0.0, max(-W_MAX, min(W_MAX, 3.0 * e)))
        self.settle()

    def forward(self, heading, v, max_dist=None, extra=0.0):
        """Returns ('EDGE', delta) or ('DIST', None); delta = heading minus edge normal from L/R IR timing.

        extra: keep moving up to this distance after the first channel triggers, to time the other side.
        """
        start = np.array(self.p.odom[:2])
        lost = [None, None, None]
        first = None
        for _ in range(60000):
            for i, ok in enumerate(self.ir):
                if not ok and lost[i] is None:
                    lost[i] = self.t
            if first is None and any(x is not None for x in lost):
                first = self.t
            both = lost[0] is not None and lost[2] is not None
            stop = first is not None and self.t - first >= (LATENCY if extra == 0.0 else (0.0 if both else extra / v))
            if stop:
                self.step(0.0, 0.0)
                if abs(self.p.twist[0]) < 1e-4:
                    delta = None
                    if lost[0] is not None and lost[2] is not None:
                        delta = math.atan2(v * (lost[0] - lost[2]), BASE)
                    elif extra > 0.0:
                        delta = math.radians(8.0) * (1 if lost[2] is not None else -1)   # only one side reached the edge
                    return 'EDGE', delta
            else:
                self.step(v, 2.0 * self.err(heading))
                if max_dist is not None and first is None and np.linalg.norm(np.array(self.p.odom[:2]) - start) >= max_dist:
                    self.settle()
                    return 'DIST', None
        raise RuntimeError('STUCK')

    def reverse(self, heading, dist):
        start = np.array(self.p.odom[:2])
        for _ in range(60000):
            if np.linalg.norm(np.array(self.p.odom[:2]) - start) >= dist:
                break
            self.step(-V_SLOW, 2.0 * self.err(heading))
        self.settle()

    def find_edge(self, heading, back):
        """Approach an edge frontally; with ALIGN, re-approach slowly to square on it. Returns the edge normal."""
        self.forward(heading, V_SLOW)
        normal = heading
        if ALIGN:
            for _ in range(6):
                self.reverse(normal, 0.03)
                self.turn_to(normal)
                _, delta = self.forward(normal, V_ALIGN, extra=0.006)
                self.deltas.append(round(math.degrees(delta), 2))
                normal -= delta
                if abs(delta) < math.radians(0.5):
                    break
        self.reverse(normal, back)
        return normal


def main():
    run = Run((-0.9, -0.3))
    lanes, note = 0, 'DONE'
    try:
        run.turn_to(math.pi + EPS)
        n_a = run.find_edge(math.pi + EPS, BACK_END)
        run.turn_to(n_a + math.pi / 2 if ALIGN else -math.pi / 2 + EPS)
        n_s = run.find_edge(run.yaw() if ALIGN else -math.pi / 2 + EPS, BACK_SIDE)
        x_axis = n_s + math.pi / 2 if ALIGN else EPS          # lane direction: parallel to the side edge
        heading, last = x_axis, False
        run.turn_to(heading)
        while True:
            run.forward(heading, V)
            lanes += 1
            run.reverse(heading, BACK_END)
            if last:
                break
            run.turn_to(x_axis + math.pi / 2)
            result, _ = run.forward(x_axis + math.pi / 2, V_SLOW, max_dist=SPACING)
            if result == 'EDGE':
                if ALIGN:
                    far = run.find_edge(x_axis + math.pi / 2, BACK_SIDE)
                    x_axis = far - math.pi / 2
                else:
                    run.reverse(x_axis + math.pi / 2, BACK_SIDE)
                last = True
            heading = x_axis + math.pi if abs(run.err(x_axis)) > math.pi / 2 or lanes % 2 == 1 else x_axis
            run.turn_to(heading)
            if lanes > 40:
                note = 'TOO MANY LANES'
                break
    except RuntimeError as error:
        note = str(error)
    ys = [t[1] for t in run.track]
    true_err = math.degrees(math.atan2(math.sin(x_axis), math.cos(x_axis))) if 'x_axis' in dir() else float('nan')
    print(f'offset={math.degrees(EPS):+.1f}deg align={int(ALIGN)} back_end={BACK_END} back_side={BACK_SIDE} ir_period={IR_PERIOD * 10}ms: '
          f'{note}, lanes={lanes}, time={run.t / 60:.1f} min, coverage={100 * coverage(run.track):.1f}%, '
          f'min centre-to-side-edge={1000 * (0.6 - max(abs(min(ys)), abs(max(ys)))):.0f} mm, lane-direction error={true_err:+.2f}deg, squaring steps={run.deltas}', flush=True)


if __name__ == '__main__':
    main()
