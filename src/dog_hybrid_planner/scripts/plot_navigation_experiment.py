#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate paper-style navigation figures from the ASK-3 experiment log."""
import argparse
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULT_DIR = os.path.join(PKG_DIR, 'experiments', 'results')


def load_map(map_yaml):
    with open(map_yaml, 'r') as f:
        info = yaml.safe_load(f)
    image_path = info['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)
    img = np.array(Image.open(image_path).convert('L'))
    if int(info.get('negate', 0)) == 1:
        img = 255 - img
    res = float(info['resolution'])
    ox, oy, _ = info['origin']
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]
    return np.flipud(img), extent


def load_trajectory(log_path):
    pts = []
    with open(log_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            cols = line.split()
            if len(cols) >= 3:
                pts.append((float(cols[0]), float(cols[1]), float(cols[2])))
    return pts


def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(points[i][0] - points[i - 1][0],
                          points[i][1] - points[i - 1][1])
               for i in range(1, len(points)))


def plot_map(map_yaml, json_path, log_path, out_path):
    img, extent = load_map(map_yaml)
    with open(json_path, 'r') as f:
        data = json.load(f)
    traj = load_trajectory(log_path)

    fig, ax = plt.subplots(figsize=(8.5, 6.2), dpi=180)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=extent, origin='lower')

    for i, item in enumerate(data.get('per_goal', [])):
        path = item.get('global_path', [])
        if len(path) >= 2:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, color='#1f5fbf', linestyle='--', linewidth=1.25,
                    alpha=0.85, label='A* smooth path' if i == 0 else None)

    if traj:
        xs = [p[1] for p in traj]
        ys = [p[2] for p in traj]
        ax.plot(xs, ys, color='#d62728', linewidth=1.8,
                label='ASK-3 trajectory')
        ax.scatter([xs[0]], [ys[0]], s=60, marker='*', color='#2ca02c',
                   edgecolor='black', linewidth=0.4, label='Start', zorder=5)

    goals = data.get('goals', [])
    if goals:
        gx = [g[0] for g in goals]
        gy = [g[1] for g in goals]
        ax.scatter(gx, gy, s=38, marker='s', color='white',
                   edgecolor='black', linewidth=0.9, label='Goals', zorder=5)
        for i, (x, y, _) in enumerate(goals, 1):
            ax.text(x + 0.08, y + 0.08, 'G%d' % i, fontsize=8, color='black')

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel('x / m')
    ax.set_ylabel('y / m')
    ax.set_title('ASK-3 indoor navigation experiment')
    ax.grid(True, color='#b0b0b0', linewidth=0.35, alpha=0.35)
    ax.legend(loc='upper right', fontsize=8, framealpha=0.92)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_metrics(json_path, out_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    per_goal = data.get('per_goal', [])
    labels = ['G%d' % (i + 1) for i in range(len(per_goal))]
    times = [float(g.get('time_s', 0.0)) for g in per_goal]
    actual = [float(g.get('actual_path_length_m', 0.0)) for g in per_goal]
    planned = [float(g.get('global_path_length_m', 0.0)) for g in per_goal]

    x = np.arange(len(labels))
    width = 0.36
    fig, ax1 = plt.subplots(figsize=(7.2, 4.4), dpi=180)
    ax1.bar(x - width / 2.0, planned, width, color='#4c78a8',
            label='Planned length / m')
    ax1.bar(x + width / 2.0, actual, width, color='#f58518',
            label='Actual length / m')
    ax1.set_ylabel('Path length / m')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    ax1.grid(True, axis='y', color='#b0b0b0', linewidth=0.35, alpha=0.35)

    ax2 = ax1.twinx()
    ax2.plot(x, times, color='#d62728', marker='o', linewidth=1.7,
             label='Time / s')
    ax2.set_ylabel('Time / s')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right',
               fontsize=8, framealpha=0.92)
    ax1.set_title('ASK-3 navigation metrics')
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=os.path.join(PKG_DIR, 'maps', 'ask3_lab.yaml'))
    parser.add_argument('--json', default=os.path.join(RESULT_DIR, 'dog_integration.json'))
    parser.add_argument('--log', default=os.path.join(RESULT_DIR, 'dog_integration_trajectory.log'))
    parser.add_argument('--out-map', default=os.path.join(RESULT_DIR, 'dog_navigation_paper_plot.png'))
    parser.add_argument('--out-metrics', default=os.path.join(RESULT_DIR, 'dog_navigation_metrics_plot.png'))
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.out_map), exist_ok=True)
    plot_map(args.map, args.json, args.log, args.out_map)
    plot_metrics(args.json, args.out_metrics)
    print(args.out_map)
    print(args.out_metrics)


if __name__ == '__main__':
    main()
