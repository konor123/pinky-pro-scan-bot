"""Headless MuJoCo probe: IR-based edge stop and lane change on a raised 2.4 x 1.2 m panel."""
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
UPSTREAM = Path(sys.argv[1]).resolve()
os.environ['PINKY_UPSTREAM'] = str(UPSTREAM)
sys.path.insert(0, str(HERE / 'shim'))
sys.path.insert(0, str(UPSTREAM / 'pinky_mujoco'))

import mujoco  # noqa: E402
from pinky_mujoco.model import Physics, build_model  # noqa: E402

PANEL_TOP = 0.30
EDGE_X, EDGE_Y = 1.2, 0.6
IR_POINTS = np.array([[0.0295, 0.02, -0.015], [0.0295, 0.0, -0.015], [0.0295, -0.02, -0.015]])
DT = 0.01


_SCENES = {}


def make(spawn_xy, out):
    cfg = yaml.safe_load((UPSTREAM / 'pinky_mujoco/config/mujoco.yaml').read_text())
    cfg['spawn']['position'] = [spawn_xy[0], spawn_xy[1], PANEL_TOP + 0.030]
    cfg['camera']['enabled'] = False
    cfg_path = out / 'cfg.yaml'
    cfg_path.write_text(yaml.safe_dump(cfg))
    scene = build_model(str(UPSTREAM / 'pinky_mujoco'), out, config_file=str(cfg_path), world=str(HERE / 'panel.yaml'))
    p = Physics(scene, str(cfg_path))
    for _ in range(100):
        p.step()
    return p


def ir_surface(p):
    """True per channel when the downward ray hits a surface within 30 mm."""
    body = p.model.body('base_link').id
    pos, rot = p.data.xpos[body], p.data.xmat[body].reshape(3, 3)
    down = -rot[:, 2]
    geom = np.zeros(1, dtype=np.int32)
    result = []
    for local in IR_POINTS:
        d = mujoco.mj_ray(p.model, p.data, pos + rot @ local, down, p.ray_group, 1, -1, geom)
        result.append(0 <= d <= 0.03)
    return result


def fallen(p):
    body = p.model.body('base_link').id
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, p.data.xmat[body].reshape(3, 3)[2, 2]))))
    return p.data.xpos[body][2] < PANEL_TOP or tilt > 15


def yaw(p):
    return p.pose()[2]


def drive_until_ir(p, v, latency, limit_s=30):
    """Drive +x at v; `latency` seconds after any IR channel loses the surface, command zero."""
    t, lost_at = 0.0, None
    while t < limit_s:
        if lost_at is None and not all(ir_surface(p)):
            lost_at = t
        cmd = 0.0 if (lost_at is not None and t - lost_at >= latency) else v
        p.step(cmd, 0.0)
        t += DT
        if fallen(p):
            return 'FELL', lost_at
        if cmd == 0.0 and abs(p.twist[0]) < 1e-4 and t - lost_at > latency + 0.3:
            return 'STOPPED', lost_at
    return 'TIMEOUT', lost_at


def move(p, dist, v):
    start = p.pose()[:2].copy()
    steps = 0
    while np.linalg.norm(p.pose()[:2] - start) < abs(dist):
        p.step(math.copysign(v, dist), 0.0)
        steps += 1
        if fallen(p) or steps > 6000:
            return False
    for _ in range(30):
        p.step()
    return not fallen(p)


def turn(p, angle, w=0.4, track=None):
    start, done = yaw(p), 0.0
    prev = start
    steps = 0
    while abs(done) < abs(angle) - math.radians(0.5):
        p.step(0.0, math.copysign(w, angle))
        steps += 1
        if steps > 6000:
            return False
        now = yaw(p)
        done += math.atan2(math.sin(now - prev), math.cos(now - prev))
        prev = now
        if track is not None:
            track.append(p.data.xpos[p.model.body('caster_wheel').id].copy())
        if fallen(p):
            return False
    for _ in range(30):
        p.step()
    return not fallen(p)


def main():
    out = HERE / 'out'
    p = make((0.9, 0.0), out)
    print('bodies:', [p.model.body(i).name for i in range(p.model.nbody)])
    print('on-panel IR:', ir_surface(p), 'z', round(float(p.data.xpos[p.model.body("base_link").id][2]), 4))

    print('\n[A] no stop, 0.08 m/s')
    p = make((0.9, 0.0), out)
    for _ in range(1500):
        p.step(0.08, 0.0)
        if fallen(p):
            break
    print('   fell:', fallen(p), 'x at fall', round(float(p.pose()[0]), 3))

    print('\n[B] IR stop: speed, latency -> result, axle-to-edge (mm), front IR overhang (mm)')
    for v in (0.05, 0.08, 0.12, 0.20):
        for latency in (0.03, 0.08, 0.15):
            p = make((0.9, 0.0), out)
            result, _ = drive_until_ir(p, v, latency)
            x = float(p.pose()[0])
            print(f'   v={v:.2f} latency={latency:.2f}: {result:8s} axle-to-edge={1000 * (EDGE_X - x):6.1f}  ir-over-edge={1000 * (x + 0.0295 - EDGE_X):6.1f}')

    print('\n[C] lane change after IR stop (v=0.08, latency=0.08): reverse, turn 90, shift 0.08, turn 90, drive back')
    for back in (0.0, 0.03, 0.06):
        p = make((0.9, 0.0), out)
        result, _ = drive_until_ir(p, 0.08, 0.08)
        ok = result == 'STOPPED'
        track = []
        if ok and back > 0:
            ok = move(p, -back, 0.03)
        centre = float(EDGE_X - p.pose()[0])
        ok = ok and turn(p, math.pi / 2, track=track)
        ok = ok and move(p, 0.08, 0.05)
        ok = ok and turn(p, math.pi / 2, track=track)
        ok = ok and move(p, 0.5, 0.08)
        caster_min = min((EDGE_X - c[0]) for c in track) if track else float('nan')
        print(f'   reverse={back * 1000:3.0f} mm: centre-to-edge before turn={centre * 1000:5.1f} mm, '
              f'min caster-centre-to-edge={caster_min * 1000:6.1f} mm, completed={ok}, fell={fallen(p)}, '
              f'end pose=({p.pose()[0]:.3f}, {p.pose()[1]:.3f}, {math.degrees(p.pose()[2]):.1f} deg)')

    print('\n[D] oblique approach, v=0.08 latency=0.08: heading -> result')
    for deg in (15, 30, 45, 60):
        p = make((0.9, 0.0), out)
        turn(p, math.radians(deg))
        t, lost = 0.0, None
        res = 'TIMEOUT'
        while t < 30:
            if lost is None and not all(ir_surface(p)):
                lost = t
            cmd = 0.0 if (lost is not None and t - lost >= 0.08) else 0.08
            p.step(cmd, 0.0)
            t += DT
            if fallen(p):
                res = 'FELL'
                break
            if cmd == 0.0 and abs(p.twist[0]) < 1e-4 and t - lost > 0.4:
                res = 'STOPPED'
                break
        wl = p.data.xpos[p.model.body('l_wheel').id]
        wr = p.data.xpos[p.model.body('r_wheel').id]
        print(f'   {deg:2d} deg: {res:8s} wheel-axle-to-edge L={1000 * (EDGE_X - wl[0]):6.1f} R={1000 * (EDGE_X - wr[0]):6.1f} mm')


if __name__ == '__main__':
    main()
