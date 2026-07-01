#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Parse a Gazebo SDF containing box-shaped walls and rasterize them to an
occupancy-grid PGM + YAML pair that map_server can consume.

Usage:
  python3 sdf_to_map.py [SDF_PATH] [OUT_DIR]

Defaults:
  SDF_PATH = /home/cjx/catkin_ws/src/py_hybrid_planner/maps2/maps2.sdf
  OUT_DIR  = /home/cjx/catkin_ws/src/py_hybrid_planner/maps     (map.pgm / map.yaml)
"""
import sys
import xml.etree.ElementTree as ET
import math
import os
from PIL import Image, ImageDraw

DEFAULT_SDF = "/home/cjx/catkin_ws/src/py_hybrid_planner/maps2/maps2.sdf"
DEFAULT_OUT = "/home/cjx/catkin_ws/src/py_hybrid_planner/maps"

RESOLUTION = 0.05
PADDING    = 1.0
FREE       = 254
OCCUPIED   = 0

def parse_pose(text):
    vals = [float(x) for x in text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    return vals

def compose(parent, child):
    px, py, _, _, _, pyaw = parent
    cx, cy, cz, cr, cp, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return [px + cx*c - cy*s, py + cx*s + cy*c, cz, cr, cp, pyaw + cyaw]

def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF
    out_dir  = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT
    out_pgm  = os.path.join(out_dir, "map.pgm")
    out_yaml = os.path.join(out_dir, "map.yaml")

    tree = ET.parse(sdf_path)
    model = tree.getroot().find("model")
    mp = model.find("pose")
    mpose = parse_pose(mp.text) if mp is not None else [0]*6

    walls = []
    for link in model.findall("link"):
        lp = parse_pose(link.find("pose").text) if link.find("pose") is not None else [0]*6
        link_world = compose(mpose, lp)
        col = link.find("collision")
        if col is None:
            continue
        box = col.find("geometry/box/size")
        if box is None:
            continue
        size = [float(x) for x in box.text.split()]
        cpe = col.find("pose")
        cpose = parse_pose(cpe.text) if cpe is not None else [0]*6
        w = compose(link_world, cpose)
        walls.append((w[0], w[1], w[5], size[0], size[1]))

    xs, ys = [], []
    for wx, wy, wyaw, sx, sy in walls:
        hx, hy = sx/2.0, sy/2.0
        c, s = math.cos(wyaw), math.sin(wyaw)
        for dx in (-hx, hx):
            for dy in (-hy, hy):
                xs.append(wx + dx*c - dy*s)
                ys.append(wy + dx*s + dy*c)

    min_x = math.floor((min(xs) - PADDING) / RESOLUTION) * RESOLUTION
    min_y = math.floor((min(ys) - PADDING) / RESOLUTION) * RESOLUTION
    max_x = max(xs) + PADDING
    max_y = max(ys) + PADDING
    W = int(math.ceil((max_x - min_x) / RESOLUTION))
    H = int(math.ceil((max_y - min_y) / RESOLUTION))
    if W % 2: W += 1
    if H % 2: H += 1

    print(f"SDF:   {sdf_path}")
    print(f"Walls: {len(walls)}")
    print(f"Origin: ({min_x:.3f}, {min_y:.3f})")
    print(f"Size  : {W}x{H} px   {W*RESOLUTION:.2f}x{H*RESOLUTION:.2f} m")

    img = Image.new("L", (W, H), FREE)
    draw = ImageDraw.Draw(img)

    def w2px(wx, wy):
        return ((wx - min_x)/RESOLUTION, (wy - min_y)/RESOLUTION)

    for wx, wy, wyaw, sx, sy in walls:
        hx, hy = sx/2.0, sy/2.0
        c, s = math.cos(wyaw), math.sin(wyaw)
        pts_w = [(wx + dx*c - dy*s, wy + dx*s + dy*c)
                 for (dx, dy) in [(-hx,-hy), (hx,-hy), (hx,hy), (-hx,hy)]]
        draw.polygon([w2px(x, y) for x, y in pts_w], fill=OCCUPIED)

    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    os.makedirs(out_dir, exist_ok=True)
    img.save(out_pgm)
    with open(out_yaml, "w") as f:
        f.write(
            f"image: map.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{min_x:.4f}, {min_y:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
    print(f"Wrote {out_pgm}")
    print(f"Wrote {out_yaml}")

if __name__ == "__main__":
    main()
