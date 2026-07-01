#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline DWA benchmark: omnidirectional DWA (dog) vs differential-drive
DWA (wheeled baseline) on identical scenarios.

The wheeled DWA is imported from the existing py_hybrid_planner package
and integrated using the same world map. We feed its (v, w) commands to
a unicycle integrator; the OmniDWA's (vx, vy, w) commands feed a
holonomic integrator. Both controllers chase the same A* key points.

Output:
  experiments/results/dog_dwa_benchmark.json
  experiments/results/dog_dwa_benchmark.png
"""
import os, sys, json, math, time, types
import yaml, numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WHEELED_SCRIPTS = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "py_hybrid_planner", "scripts"))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, WHEELED_SCRIPTS)

stub = types.ModuleType("rospy")
stub.loginfo = lambda *a, **k: None
stub.logwarn = lambda *a, **k: None
stub.logerr  = lambda *a, **k: None
sys.modules["rospy"] = stub

from improved_astar import ImprovedAStar
from omni_dwa import OmniDWA
try:
    from improved_dwa import ImprovedDWA as WheeledDWA
    HAS_WHEELED = True
except Exception as e:
    print(f"Warning: improved_dwa import failed: {e}")
    HAS_WHEELED = False

MAP_YAML = os.path.join(SCRIPT_DIR, "..", "maps", "map.yaml")
OUT_DIR  = os.path.join(SCRIPT_DIR, "..", "experiments", "results")

# Same dog-friendly scenarios as benchmark_dog_astar.py.
SCENARIOS = [
    {"name": "short_open",     "start": (0.5, -0.7, 0.0),    "goal": (2.0, -0.7), "timeout": 30.0},
    {"name": "long_corridor",  "start": (0.5, -0.7, 0.0),    "goal": (6.0, -0.7), "timeout": 80.0},
    {"name": "diag_via_top",   "start": (0.5, -0.7, 0.7854), "goal": (6.0,  3.5), "timeout": 120.0},
]

DOG_INFLATE = 0.22
DOG_RADIUS = 0.23


def load_map(path):
    with open(path) as f:
        m = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path), m['image'])))
    occ = np.zeros_like(img, dtype=np.int8)
    occ[img < 50] = 100
    occ[img > 200] = 0
    occ = np.flipud(occ)
    h, w = occ.shape
    return occ, m['resolution'], (m['origin'][0], m['origin'][1]), w, h


def occ_points(occ, res, origin):
    ys, xs = np.where(occ >= 80)
    wx = origin[0] + xs * res + res * 0.5
    wy = origin[1] + ys * res + res * 0.5
    return np.stack([wx, wy], axis=1)


def laser_fan(robot_x, robot_y, yaw, obstacle_cells, max_range=4.0, n_beams=180):
    d = np.hypot(obstacle_cells[:, 0] - robot_x, obstacle_cells[:, 1] - robot_y)
    mask = d < max_range
    pts = obstacle_cells[mask]
    if pts.size == 0:
        return []
    bear = np.arctan2(pts[:, 1] - robot_y, pts[:, 0] - robot_x) - yaw
    bear = (bear + np.pi) % (2 * np.pi) - np.pi
    dist = d[mask]
    bucket = np.floor((bear + np.pi) / (2 * np.pi) * n_beams).astype(int)
    bucket = np.clip(bucket, 0, n_beams - 1)
    best_d = np.full(n_beams, np.inf)
    best_i = np.full(n_beams, -1, dtype=int)
    for i, b in enumerate(bucket):
        if dist[i] < best_d[b]:
            best_d[b] = dist[i]
            best_i[b] = i
    out = []
    for k in range(n_beams):
        if best_i[k] >= 0:
            out.append((float(pts[best_i[k], 0]), float(pts[best_i[k], 1])))
    return out


def simulate_omni(scenario, occ, res, origin, w, h):
    astar = ImprovedAStar(occ, res, origin, w, h, inflate_radius=DOG_INFLATE)
    smooth, keys = astar.plan((scenario['start'][0], scenario['start'][1]),
                              scenario['goal'])
    if not smooth:
        return {"ok": False, "reason": "global planning failed",
                "path": [], "cmd": []}

    dwa = OmniDWA()
    obstacle_cells = occ_points(occ, res, origin)

    dt = 0.1
    x, y, yaw = scenario['start']
    vx = vy = w_ = 0.0
    prev_theta = yaw

    path_hist = [(x, y, yaw)]
    cmd_hist = []
    min_obs_dist = float('inf')
    stuck_steps = 0
    collision = False
    key_idx = 0

    t = 0.0
    while t < scenario['timeout']:
        if math.hypot(x - scenario['goal'][0], y - scenario['goal'][1]) < 0.30:
            return _pack_result(True, t, path_hist, cmd_hist,
                                min_obs_dist, stuck_steps, collision)
        while key_idx < len(keys) - 1:
            kx, ky = keys[key_idx]
            if math.hypot(kx - x, ky - y) < 0.5:
                key_idx += 1
            else:
                break
        target = keys[key_idx]
        obs = laser_fan(x, y, yaw, obstacle_cells)
        d_now = float(np.min(np.hypot(obstacle_cells[:, 0] - x,
                                      obstacle_cells[:, 1] - y)))
        if d_now < min_obs_dist:
            min_obs_dist = d_now
        if d_now < DOG_RADIUS - 0.04:
            collision = True
        nvx, nvy, nw, info = dwa.plan(x, y, yaw, vx, vy, w_,
                                       target[0], target[1], obs, prev_theta)
        if abs(nvx) < 1e-3 and abs(nvy) < 1e-3 and abs(nw) < 1e-3:
            stuck_steps += 1
        else:
            stuck_steps = max(0, stuck_steps - 1)
        ct, st = math.cos(yaw), math.sin(yaw)
        x += (nvx * ct - nvy * st) * dt
        y += (nvx * st + nvy * ct) * dt
        yaw += nw * dt
        vx, vy, w_ = nvx, nvy, nw
        prev_theta = yaw
        path_hist.append((x, y, yaw))
        cmd_hist.append((t, nvx, nvy, nw))
        t += dt
    return _pack_result(False, t, path_hist, cmd_hist,
                        min_obs_dist, stuck_steps, collision, reason="timeout")


def simulate_wheeled(scenario, occ, res, origin, w, h):
    if not HAS_WHEELED:
        return {"ok": False, "reason": "wheeled DWA not available",
                "path": [], "cmd": []}
    astar = ImprovedAStar(occ, res, origin, w, h, inflate_radius=DOG_INFLATE)
    smooth, keys = astar.plan((scenario['start'][0], scenario['start'][1]),
                              scenario['goal'])
    if not smooth:
        return {"ok": False, "reason": "global planning failed",
                "path": [], "cmd": []}
    dwa = WheeledDWA()
    obstacle_cells = occ_points(occ, res, origin)
    dt = 0.1
    x, y, yaw = scenario['start']
    v = w_ = 0.0
    prev_theta = yaw
    path_hist = [(x, y, yaw)]
    cmd_hist = []
    min_obs_dist = float('inf')
    stuck_steps = 0
    collision = False
    key_idx = 0
    t = 0.0
    while t < scenario['timeout']:
        if math.hypot(x - scenario['goal'][0], y - scenario['goal'][1]) < 0.30:
            return _pack_result(True, t, path_hist, cmd_hist,
                                min_obs_dist, stuck_steps, collision)
        while key_idx < len(keys) - 1:
            kx, ky = keys[key_idx]
            if math.hypot(kx - x, ky - y) < 0.5:
                key_idx += 1
            else:
                break
        target = keys[key_idx]
        obs = laser_fan(x, y, yaw, obstacle_cells)
        d_now = float(np.min(np.hypot(obstacle_cells[:, 0] - x,
                                      obstacle_cells[:, 1] - y)))
        if d_now < min_obs_dist:
            min_obs_dist = d_now
        if d_now < DOG_RADIUS - 0.04:
            collision = True
        out = dwa.plan(x, y, yaw, v, w_, target[0], target[1], obs, prev_theta)
        if len(out) == 3:
            nv, nw, info = out
        else:
            nv, nw = out
        if abs(nv) < 1e-3 and abs(nw) < 1e-3:
            stuck_steps += 1
        else:
            stuck_steps = max(0, stuck_steps - 1)
        x += nv * math.cos(yaw) * dt
        y += nv * math.sin(yaw) * dt
        yaw += nw * dt
        v, w_ = nv, nw
        prev_theta = yaw
        path_hist.append((x, y, yaw))
        cmd_hist.append((t, nv, 0.0, nw))
        t += dt
    return _pack_result(False, t, path_hist, cmd_hist,
                        min_obs_dist, stuck_steps, collision, reason="timeout")


def _pack_result(ok, t, path_hist, cmd_hist, min_obs, stuck, collision,
                 reason=None):
    res = {
        "ok": ok,
        "arrival_time_s": t,
        "path_length_m": sum(
            math.hypot(path_hist[i+1][0]-path_hist[i][0],
                       path_hist[i+1][1]-path_hist[i][1])
            for i in range(len(path_hist)-1)),
        "min_obs_dist_m": float(min_obs) if min_obs != float('inf') else None,
        "stuck_steps": stuck,
        "collision": collision,
        "path": path_hist,
        "cmd": cmd_hist,
    }
    if reason:
        res["reason"] = reason
    return res


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    occ, res, origin, w, h = load_map(MAP_YAML)
    print(f"Map: {w}x{h} res={res} origin={origin}")

    results = []
    for sc in SCENARIOS:
        print(f"\n=== {sc['name']}: {sc['start']} -> {sc['goal']} ===")
        wd = simulate_wheeled(sc, occ, res, origin, w, h)
        od = simulate_omni   (sc, occ, res, origin, w, h)
        for tag, r in (("wheeled", wd), ("omni", od)):
            print(f"  {tag:8s}: ok={r.get('ok')} t={r.get('arrival_time_s', 0):.1f}s "
                  f"len={r.get('path_length_m', 0):.2f}m "
                  f"min_obs={r.get('min_obs_dist_m')} stuck={r.get('stuck_steps')}")
        results.append({
            "name": sc['name'],
            "start": sc['start'],
            "goal": sc['goal'],
            "wheeled": wd,
            "omni": od,
        })

    out_json = os.path.join(OUT_DIR, "dog_dwa_benchmark.json")
    with open(out_json, "w") as f:
        json.dump([{
            "name": r['name'],
            "start": r['start'],
            "goal": r['goal'],
            "wheeled": {k: v for k, v in r['wheeled'].items() if k not in ('path', 'cmd')},
            "omni":    {k: v for k, v in r['omni'].items()    if k not in ('path', 'cmd')},
        } for r in results], f, indent=2)
    print(f"\nWrote {out_json}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        fig, axes = plt.subplots(1, len(SCENARIOS),
                                 figsize=(4 * len(SCENARIOS), 4))
        if len(SCENARIOS) == 1:
            axes = [axes]
        disp = (occ < 50).astype(np.uint8)
        for ax, sc, r in zip(axes, SCENARIOS, results):
            ax.imshow(disp, cmap=ListedColormap(["#222222", "#dddddd"]),
                      extent=[origin[0],
                              origin[0] + w * res,
                              origin[1],
                              origin[1] + h * res], origin='lower')
            if 'path' in r['wheeled']:
                xs = [p[0] for p in r['wheeled']['path']]
                ys = [p[1] for p in r['wheeled']['path']]
                if xs:
                    ax.plot(xs, ys, color='tab:orange', lw=1.6, label='Wheeled DWA')
            if 'path' in r['omni']:
                xs = [p[0] for p in r['omni']['path']]
                ys = [p[1] for p in r['omni']['path']]
                if xs:
                    ax.plot(xs, ys, color='tab:blue', lw=1.6, label='OmniDWA (dog)')
            ax.plot(sc['start'][0], sc['start'][1], 'g*', ms=12, label='start')
            ax.plot(*sc['goal'],      'rX', ms=10, label='goal')
            ax.set_title(sc['name'])
            ax.set_aspect('equal')
            ax.legend(fontsize=7, loc='upper right')
        plt.tight_layout()
        png = os.path.join(OUT_DIR, "dog_dwa_benchmark.png")
        plt.savefig(png, dpi=140)
        print(f"Wrote {png}")
    except Exception as e:
        print(f"Plot skipped: {e}")


if __name__ == "__main__":
    main()
