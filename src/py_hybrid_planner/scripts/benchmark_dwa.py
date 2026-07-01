#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline DWA comparison benchmark.

Simulates the robot executing DWA commands against the map (no Gazebo) so
we can compare Traditional vs Improved DWA on identical scenarios. Laser
"rays" are approximated by taking occupied cells within a 3.5 m radius
from the robot pose and within a +-180 deg fan.

For each scenario we record: arrival time, path length, min distance to
obstacles during run, number of 'stuck' steps, success flag.

Output:
  experiments/results/dwa_benchmark.json
  experiments/results/dwa_benchmark.png
"""
import os, sys, json, math, time, types
import yaml, numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

# stub rospy for improved_astar import (planners use rospy.loginfo only)
stub = types.ModuleType("rospy")
stub.loginfo = lambda *a, **k: None
stub.logwarn = lambda *a, **k: None
stub.logerr  = lambda *a, **k: None
sys.modules["rospy"] = stub

from improved_astar import ImprovedAStar
from improved_dwa   import ImprovedDWA
from traditional_dwa import TraditionalDWA

MAP_YAML = os.path.join(SCRIPT_DIR, "..", "maps", "map.yaml")
OUT_DIR  = os.path.join(SCRIPT_DIR, "..", "experiments", "results")

SCENARIOS = [
    {"name": "short_open",        "start": (0.8, 0.8, 0.7854), "goal": (2.0, 1.5), "timeout": 30.0},
    {"name": "diagonal_long",     "start": (0.8, 0.8, 0.7854), "goal": (5.5, 2.7), "timeout": 90.0},
    {"name": "around_obstacles",  "start": (0.8, 0.8, 0.0),    "goal": (4.0, 1.0), "timeout": 80.0},
]


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


def laser_fan(robot_x, robot_y, yaw, obstacle_cells, max_range=3.5, n_beams=180):
    """Return a list of (px, py) in world frame for each 'hit' found by a
    simulated laser fan. The naive approach: project cells within radius
    and keep those closer than farther ones along each bearing."""
    d = np.hypot(obstacle_cells[:, 0] - robot_x, obstacle_cells[:, 1] - robot_y)
    mask = d < max_range
    pts = obstacle_cells[mask]
    if pts.size == 0:
        return []
    # bearing in robot frame
    bear = np.arctan2(pts[:, 1] - robot_y, pts[:, 0] - robot_x) - yaw
    bear = (bear + np.pi) % (2 * np.pi) - np.pi
    dist = d[mask]
    # bucket bearings; keep closest in each bucket
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


def simulate(planner_cls, scenario, occ, res, origin, w, h, astar_cls=ImprovedAStar):
    astar = astar_cls(occ, res, origin, w, h)
    start_pose = scenario['start']
    goal = scenario['goal']
    smooth, keys = astar.plan((start_pose[0], start_pose[1]), goal)
    if not smooth:
        return {"ok": False, "reason": "global planning failed"}

    dwa = planner_cls()
    obstacle_cells = occ_points(occ, res, origin)

    dt = 0.1  # 10 Hz control
    robot_x, robot_y, yaw = start_pose
    v, w_ = 0.0, 0.0
    prev_theta = yaw

    path_hist = [(robot_x, robot_y, yaw)]
    cmd_hist = []
    min_obs_dist = float('inf')
    stuck_steps = 0
    collision = False
    key_idx = 0

    t = 0.0
    while t < scenario['timeout']:
        # goal reached?
        if math.hypot(robot_x - goal[0], robot_y - goal[1]) < 0.20:
            return {
                "ok": True,
                "arrival_time_s": t,
                "path_length_m": sum(
                    math.hypot(path_hist[i+1][0]-path_hist[i][0],
                               path_hist[i+1][1]-path_hist[i][1])
                    for i in range(len(path_hist)-1)),
                "min_obs_dist_m": float(min_obs_dist),
                "stuck_steps": stuck_steps,
                "collision": collision,
                "path": path_hist,
                "cmd": cmd_hist,
            }

        # advance key index
        while key_idx < len(keys) - 1:
            kx, ky = keys[key_idx]
            if math.hypot(kx - robot_x, ky - robot_y) < 0.5:
                key_idx += 1
            else:
                break
        target = keys[key_idx]

        # get simulated laser obstacles
        obs = laser_fan(robot_x, robot_y, yaw, obstacle_cells)

        # current distance to closest cell
        d_now = float(np.min(np.hypot(obstacle_cells[:, 0] - robot_x,
                                      obstacle_cells[:, 1] - robot_y)))
        if d_now < min_obs_dist:
            min_obs_dist = d_now
        if d_now < 0.11:   # robot radius -> collision
            collision = True

        out = dwa.plan(robot_x, robot_y, yaw, v, w_, target[0], target[1],
                       obs, prev_theta)
        if len(out) == 3:
            nv, nw, info = out
        else:
            nv, nw = out; info = {}

        if abs(nv) < 1e-3 and abs(nw) < 1e-3:
            stuck_steps += 1
        else:
            stuck_steps = max(0, stuck_steps - 1)

        # integrate
        robot_x += nv * math.cos(yaw) * dt
        robot_y += nv * math.sin(yaw) * dt
        yaw += nw * dt
        v, w_ = nv, nw
        prev_theta = yaw

        path_hist.append((robot_x, robot_y, yaw))
        cmd_hist.append((t, nv, nw))
        t += dt

    return {
        "ok": False,
        "reason": "timeout",
        "arrival_time_s": t,
        "path_length_m": sum(
            math.hypot(path_hist[i+1][0]-path_hist[i][0],
                       path_hist[i+1][1]-path_hist[i][1])
            for i in range(len(path_hist)-1)),
        "min_obs_dist_m": float(min_obs_dist),
        "stuck_steps": stuck_steps,
        "collision": collision,
        "path": path_hist,
        "cmd": cmd_hist,
    }


def summarise(r):
    return {k: v for k, v in r.items() if k not in ("path", "cmd")}


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    occ, res, origin, w, h = load_map(MAP_YAML)

    results = []
    for sc in SCENARIOS:
        print(f"\n=== {sc['name']} {sc['start']} -> {sc['goal']} ===")
        t0 = time.perf_counter()
        tr = simulate(TraditionalDWA, sc, occ, res, origin, w, h)
        tr['sim_time_s'] = time.perf_counter() - t0
        t0 = time.perf_counter()
        ir = simulate(ImprovedDWA,    sc, occ, res, origin, w, h)
        ir['sim_time_s'] = time.perf_counter() - t0
        print("  traditional:", summarise(tr))
        print("  improved   :", summarise(ir))
        results.append({"scenario": sc, "traditional": tr, "improved": ir})

    # save json (without path lists for size)
    strip = lambda d: {k: v for k, v in d.items() if k not in ("path", "cmd")}
    def to_json(o):
        if isinstance(o, (np.floating, np.integer)):
            return float(o)
        raise TypeError
    with open(os.path.join(OUT_DIR, "dwa_benchmark.json"), "w") as f:
        json.dump([{
            "scenario": r['scenario'],
            "traditional": strip(r['traditional']),
            "improved": strip(r['improved']),
        } for r in results], f, indent=2, default=to_json)

    # plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(SCENARIOS)
        fig, axes = plt.subplots(1, n, figsize=(5 * n, 5))
        if n == 1:
            axes = [axes]
        extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
        for ax, r in zip(axes, results):
            ax.imshow(occ, cmap='gray_r', origin='lower', extent=extent, alpha=0.4)
            tp = np.array(r['traditional']['path'])
            ip = np.array(r['improved']['path'])
            if tp.size:
                ax.plot(tp[:, 0], tp[:, 1], 'r-', lw=2, label='Traditional DWA')
            if ip.size:
                ax.plot(ip[:, 0], ip[:, 1], 'b-', lw=2, label='Improved DWA')
            sx, sy = r['scenario']['start'][:2]
            gx, gy = r['scenario']['goal']
            ax.plot(sx, sy, 'go', ms=10, label='start')
            ax.plot(gx, gy, 'r*', ms=14, label='goal')
            ax.set_title(r['scenario']['name'])
            ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, "dwa_benchmark.png"), dpi=120)
        print(f"\nSaved plot to {OUT_DIR}/dwa_benchmark.png")
    except Exception as e:
        print("Plot skipped:", e)

    print("\n{:<20}{:>12}{:>14}{:>16}{:>14}{:>10}".format(
        "case", "alg", "arrival(s)", "min_obs(m)", "length(m)", "stuck"))
    print("-" * 96)
    for r in results:
        for alg in ('traditional', 'improved'):
            x = r[alg]
            ok = "OK" if x.get('ok') else f"FAIL-{x.get('reason', '?')[:6]}"
            print("{:<20}{:>12}{:>14.1f}{:>16.3f}{:>14.3f}{:>10d}  [{}]".format(
                r['scenario']['name'], alg,
                x.get('arrival_time_s', 0),
                x.get('min_obs_dist_m', 0),
                x.get('path_length_m', 0),
                x.get('stuck_steps', 0),
                ok))


if __name__ == "__main__":
    main()
