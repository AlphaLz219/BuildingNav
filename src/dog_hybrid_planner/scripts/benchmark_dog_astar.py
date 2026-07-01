#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline A* benchmark for the ASK-3 quadruped configuration.

Re-uses the same `traditional_astar.py` and `improved_astar.py` from the
two packages, but instantiates ImprovedAStar with the dog-sized
inflation radius (0.22 m) to expose the higher inflation cost. The
test scenarios were re-selected (vs. the wheeled benchmark) so that
both start and goal remain reachable under the larger inflation.

Output:
  experiments/results/dog_astar_benchmark.json
  experiments/results/dog_astar_benchmark.png
"""
import os, sys, json, math, re, time, types
import yaml
import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WHEELED_SCRIPTS = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "py_hybrid_planner", "scripts"))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, WHEELED_SCRIPTS)

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
try:
    from traditional_astar import TraditionalAStar
    HAS_TRADITIONAL = True
except Exception as e:
    print(f"Warning: traditional_astar not importable: {e}")
    HAS_TRADITIONAL = False

MAP_YAML = os.path.join(SCRIPT_DIR, "..", "maps", "map.yaml")
OUT_DIR  = os.path.join(SCRIPT_DIR, "..", "experiments", "results")

# Dog-friendly scenarios on the same indoor.world.
# All start/goal are inside open corridors with the dog 0.22 m inflation.
TEST_CASES = [
    {"name": "short_open",     "start": (0.5, -0.7), "goal": (2.0, -0.7)},
    {"name": "long_corridor",  "start": (0.5, -0.7), "goal": (6.0, -0.7)},
    {"name": "diag_via_top",   "start": (0.5, -0.7), "goal": (6.0,  3.5)},
]

DOG_INFLATE = 0.22


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
    planner = ImprovedAStar(occ, res, origin, w, h, inflate_radius=DOG_INFLATE)
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
    if not HAS_TRADITIONAL:
        return [], [], {"ok": False, "reason": "traditional_astar not available"}
    try:
        try:
            planner = TraditionalAStar(occ, res, origin, w, h,
                                       inflate_radius=DOG_INFLATE)
        except TypeError:
            planner = TraditionalAStar(occ, res, origin, w, h)
        t0 = time.perf_counter()
        path, keys = planner.plan(start, goal)
        dt = time.perf_counter() - t0
        m = dict(getattr(planner, "last_metrics", {}))
        m['ok'] = bool(path)
        m['planning_time_s'] = dt
        m.update(path_metrics(path))
        return path, keys, m
    except Exception as e:
        return [], [], {"ok": False, "reason": f"exception: {e}"}


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    occ, res, origin, w, h = load_occupancy(MAP_YAML)
    print(f"Map: {w}x{h} res={res} origin={origin} (dog inflate={DOG_INFLATE} m)")

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
            "improved_path": i_path,
            "traditional_path": t_path,
        })

    out_json = os.path.join(OUT_DIR, "dog_astar_benchmark.json")
    with open(out_json, "w") as f:
        json.dump([{k: v for k, v in r.items() if "path" not in k}
                   for r in results], f, indent=2)
    print(f"\nWrote {out_json}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        fig, axes = plt.subplots(1, len(TEST_CASES),
                                 figsize=(4 * len(TEST_CASES), 4))
        if len(TEST_CASES) == 1:
            axes = [axes]
        disp = (occ < 50).astype(np.uint8)
        for ax, tc, res_entry in zip(axes, TEST_CASES, results):
            ax.imshow(disp, cmap=ListedColormap(["#222222", "#dddddd"]),
                      extent=[origin[0],
                              origin[0] + w * res,
                              origin[1],
                              origin[1] + h * res], origin='lower')
            for path, color, label in [
                (res_entry['traditional_path'], 'tab:orange', 'Traditional'),
                (res_entry['improved_path'],    'tab:blue',   'Improved'),
            ]:
                if path:
                    xs = [p[0] for p in path]; ys = [p[1] for p in path]
                    ax.plot(xs, ys, color=color, lw=1.8, label=label)
            ax.plot(*tc['start'], 'g*', ms=12, label='start')
            ax.plot(*tc['goal'],  'rX', ms=10, label='goal')
            ax.set_title(tc['name'])
            ax.set_aspect('equal')
            ax.legend(fontsize=7, loc='upper right')
        plt.tight_layout()
        png = os.path.join(OUT_DIR, "dog_astar_benchmark.png")
        plt.savefig(png, dpi=140)
        print(f"Wrote {png}")
    except Exception as e:
        print(f"Plot skipped: {e}")


if __name__ == "__main__":
    main()
