#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline A* comparison benchmark.

Loads map.pgm + map.yaml directly, then runs TraditionalAStar and
ImprovedAStar on the same start/goal pairs and records metrics.

No ROS master required. A stub rospy module is injected so that
improved_astar's rospy.loginfo / rospy.logerr calls become no-ops,
while we still grab the "A* meet at ..., expanded N+M nodes" message
for the reported expansion counts.

Output:
  experiments/results/astar_benchmark.json
  experiments/results/astar_benchmark.png
"""
import os, sys, json, math, re, time, types
import yaml
import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

# --- inject a stub rospy BEFORE importing improved_astar ---
_rospy_log = []
def _capture(msg, *args):
    try:
        _rospy_log.append(msg % args if args else msg)
    except Exception:
        _rospy_log.append(str(msg))

stub = types.ModuleType("rospy")
stub.loginfo = _capture
stub.logwarn = _capture
stub.logerr  = _capture
stub.logdebug = _capture
sys.modules["rospy"] = stub

from improved_astar import ImprovedAStar
from traditional_astar import TraditionalAStar

MAP_YAML = os.path.join(SCRIPT_DIR, "..", "maps", "map.yaml")
OUT_DIR  = os.path.join(SCRIPT_DIR, "..", "experiments", "results")

TEST_CASES = [
    {"name": "short_open",       "start": (0.8, 0.8), "goal": (2.0, 1.5)},
    {"name": "diagonal_long",    "start": (0.8, 0.8), "goal": (5.5, 2.7)},
    {"name": "around_obstacles", "start": (0.8, 0.8), "goal": (4.0, 1.0)},
]


def load_occupancy(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    img = np.array(Image.open(img_path))
    occ = np.zeros_like(img, dtype=np.int8)
    occ[img < 50] = 100
    occ[img > 200] = 0
    occ[(img >= 50) & (img <= 200)] = -1
    occ = np.flipud(occ)
    h, w = occ.shape
    return occ, meta['resolution'], (meta['origin'][0], meta['origin'][1]), w, h


def path_metrics(path):
    if not path or len(path) < 2:
        return {"path_length_m": 0.0, "total_turn_rad": 0.0, "num_turns": 0}
    length = 0.0
    angle = 0.0
    turns = 0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        length += math.hypot(dx, dy)
    for i in range(1, len(path) - 1):
        a = math.atan2(path[i][1] - path[i-1][1], path[i][0] - path[i-1][0])
        b = math.atan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])
        d = abs(b - a)
        if d > math.pi:
            d = 2 * math.pi - d
        angle += d
        if d > math.radians(20):
            turns += 1
    return {"path_length_m": length, "total_turn_rad": angle, "num_turns": turns}


def run_improved(occ, res, origin, w, h, start, goal):
    _rospy_log.clear()
    planner = ImprovedAStar(occ, res, origin, w, h)
    t0 = time.perf_counter()
    path, keys = planner.plan(start, goal)
    dt = time.perf_counter() - t0
    expanded_f = expanded_b = 0
    for m in _rospy_log:
        mo = re.search(r"expanded (\d+)\+(\d+) nodes", m)
        if mo:
            expanded_f, expanded_b = int(mo.group(1)), int(mo.group(2))
            break
    metrics = {
        "ok": bool(path),
        "planning_time_s": dt,
        "expanded": expanded_f + expanded_b,
        "expanded_detail": f"{expanded_f}+{expanded_b}",
        "raw_cells": 0,
    }
    # improved_astar logs "A* path: N raw -> M smooth -> K key"
    for m in _rospy_log:
        mo = re.search(r"(\d+) raw -> (\d+) smooth -> (\d+) key", m)
        if mo:
            metrics['raw_cells'] = int(mo.group(1))
            metrics['smooth_points'] = int(mo.group(2))
            metrics['key_points'] = int(mo.group(3))
            break
    metrics.update(path_metrics(path))
    return path, keys, metrics


def run_traditional(occ, res, origin, w, h, start, goal):
    planner = TraditionalAStar(occ, res, origin, w, h)
    t0 = time.perf_counter()
    path, keys = planner.plan(start, goal)
    dt = time.perf_counter() - t0
    m = dict(planner.last_metrics)
    m['planning_time_s'] = dt
    m.update(path_metrics(path))
    return path, keys, m


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    occ, res, origin, w, h = load_occupancy(MAP_YAML)
    print(f"Map: {w}x{h} res={res} origin={origin}")

    results = []
    for tc in TEST_CASES:
        print(f"\n=== {tc['name']}: {tc['start']} -> {tc['goal']} ===")
        t_path, _, tm = run_traditional(occ, res, origin, w, h, tc['start'], tc['goal'])
        i_path, _, im = run_improved   (occ, res, origin, w, h, tc['start'], tc['goal'])
        print("  traditional:", tm)
        print("  improved   :", im)
        results.append({
            "name": tc['name'],
            "start": tc['start'],
            "goal": tc['goal'],
            "traditional": tm,
            "improved": im,
            "traditional_path": t_path,
            "improved_path": i_path,
        })

    def to_json(o):
        if isinstance(o, (np.floating, np.integer)):
            return float(o)
        raise TypeError(f"{type(o)} not json serialisable")
    with open(os.path.join(OUT_DIR, "astar_benchmark.json"), "w") as f:
        json.dump(results, f, indent=2, default=to_json)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(TEST_CASES)
        fig, axes = plt.subplots(1, n, figsize=(5 * n, 5))
        if n == 1:
            axes = [axes]
        extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
        for ax, r in zip(axes, results):
            ax.imshow(occ, cmap='gray_r', origin='lower', extent=extent, alpha=0.4)
            if r['traditional_path']:
                tp = np.array(r['traditional_path'])
                ax.plot(tp[:, 0], tp[:, 1], 'r-', lw=2, label='Traditional A*')
            if r['improved_path']:
                ip = np.array(r['improved_path'])
                ax.plot(ip[:, 0], ip[:, 1], 'b-', lw=2, label='Improved A*')
            ax.plot(*r['start'], 'go', ms=10, label='start')
            ax.plot(*r['goal'], 'r*', ms=14, label='goal')
            ax.set_title(r['name'])
            ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, "astar_benchmark.png"), dpi=120)
        print(f"\nSaved plot to {OUT_DIR}/astar_benchmark.png")
    except Exception as e:
        print("Plot skipped:", e)

    print("\n" + "=" * 100)
    print("{:<14}{:>10}{:>10}{:>10}{:>10}{:>12}{:>12}{:>12}".format(
        "case", "T_len", "I_len", "T_exp", "I_exp", "T_turns", "I_turns", "T_time_ms"))
    print("-" * 100)
    for r in results:
        tm, im = r['traditional'], r['improved']
        if tm.get('ok') and im.get('ok'):
            print("{:<14}{:>10.3f}{:>10.3f}{:>10d}{:>10d}{:>12d}{:>12d}{:>12.1f}".format(
                r['name'],
                tm.get('path_length_m', 0), im.get('path_length_m', 0),
                tm.get('expanded', 0), im.get('expanded', 0),
                tm.get('num_turns', 0), im.get('num_turns', 0),
                tm.get('planning_time_s', 0) * 1000))


if __name__ == "__main__":
    main()
