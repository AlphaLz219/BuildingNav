#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate an ASK-3 friendly Gazebo world and matching 2-D occupancy map.

The original indoor map was built for a small wheeled robot. ASK-3 has a
0.39 x 0.22 m base box, a half diagonal of about 0.224 m, and swinging
legs that benefit from extra wall clearance. This lab map uses a more
complex multi-corridor layout while keeping passable gaps wide enough for
the ASK-3 navigation inflation used by the lidar/DWA stack.
"""

import argparse
import math
from collections import deque
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = ROOT / "worlds" / "ask3_lab.world"
MAP_PATH = ROOT / "maps" / "ask3_lab.pgm"
YAML_PATH = ROOT / "maps" / "ask3_lab.yaml"

RESOLUTION = 0.05
ORIGIN_X = -0.50
ORIGIN_Y = -0.50
WIDTH = 260
HEIGHT = 190
FREE = 254
OCCUPIED = 0

START = (1.20, 1.20)
GOALS = [
    (10.80, 7.30),
    (10.80, 1.10),
    (1.30, 7.20),
    (6.40, 4.50),
]
CHECK_INFLATION = 0.32

# name, center_x, center_y, size_x, size_y, size_z, material
BOXES = [
    ("outer_south", 6.00, -0.04, 12.16, 0.08, 0.55, "Gazebo/DarkGrey"),
    ("outer_north", 6.00, 8.54, 12.16, 0.08, 0.55, "Gazebo/DarkGrey"),
    ("outer_west", -0.04, 4.25, 0.08, 8.66, 0.55, "Gazebo/DarkGrey"),
    ("outer_east", 12.04, 4.25, 0.08, 8.66, 0.55, "Gazebo/DarkGrey"),

    # Main maze walls. Each wall is thin enough to rasterize cleanly but
    # arranged so ASK-3 has multiple wide route choices and dead-end tests.
    ("wall_left_spine", 2.20, 2.00, 0.12, 3.20, 0.55, "Gazebo/Grey"),
    ("wall_lower_gate", 3.55, 3.70, 2.70, 0.12, 0.55, "Gazebo/Grey"),
    ("wall_mid_spine", 5.20, 6.65, 0.12, 3.50, 0.55, "Gazebo/Grey"),
    ("wall_lower_band", 6.95, 2.20, 2.90, 0.12, 0.55, "Gazebo/Grey"),
    ("wall_right_spine", 9.30, 4.25, 0.12, 3.90, 0.55, "Gazebo/Grey"),
    ("wall_upper_band", 6.75, 6.25, 3.10, 0.12, 0.55, "Gazebo/Grey"),
    ("wall_top_left", 3.60, 6.70, 0.12, 2.80, 0.55, "Gazebo/Grey"),

    # Short branches and islands make the map less trivial without creating
    # sub-0.9 m passages after inflation.
    ("dead_end_left", 1.20, 5.55, 1.50, 0.12, 0.45, "Gazebo/Grey"),
    ("dead_end_right", 10.95, 5.25, 1.00, 0.12, 0.45, "Gazebo/Grey"),
    ("lower_slalom", 6.70, 0.95, 0.12, 1.25, 0.45, "Gazebo/Grey"),
    ("island_start_side", 3.70, 1.35, 0.55, 0.55, 0.45, "Gazebo/Blue"),
    ("island_center_low", 6.45, 3.65, 0.55, 0.55, 0.45, "Gazebo/Blue"),
    ("island_center_high", 7.85, 5.10, 0.35, 0.35, 0.45, "Gazebo/Blue"),
    ("island_right_low", 10.80, 3.45, 0.55, 0.55, 0.45, "Gazebo/Blue"),
    ("island_top_left", 4.15, 7.35, 0.45, 0.45, 0.45, "Gazebo/Blue"),
]


def box_link(name, x, y, sx, sy, sz, material):
    return f"""    <link name="{name}">
      <pose>{x:.3f} {y:.3f} {sz / 2.0:.3f} 0 0 0</pose>
      <collision name="{name}_collision">
        <geometry>
          <box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>
        </geometry>
      </collision>
      <visual name="{name}_visual">
        <geometry>
          <box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
"""


def write_world():
    links = "".join(box_link(*box) for box in BOXES)
    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="ask3_lab">

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <scene>
      <shadows>false</shadows>
      <ambient>0.55 0.55 0.55 1</ambient>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>6.0 4.2 16.0 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <model name="ask3_lab_map">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
{links}    </model>
  </world>
</sdf>
"""
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.write_text(world, encoding="utf-8")


def is_inside_box(wx, wy, box):
    _, x, y, sx, sy, _sz, _material = box
    return abs(wx - x) <= sx / 2.0 and abs(wy - y) <= sy / 2.0


def build_grid():
    grid = bytearray(WIDTH * HEIGHT)
    for row in range(HEIGHT):
        wy = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(WIDTH):
            wx = ORIGIN_X + (col + 0.5) * RESOLUTION
            occ = any(is_inside_box(wx, wy, box) for box in BOXES)
            grid[row * WIDTH + col] = OCCUPIED if occ else FREE
    return grid


def write_map():
    grid = build_grid()
    MAP_PATH.parent.mkdir(parents=True, exist_ok=True)
    with MAP_PATH.open("wb") as f:
        f.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        f.write(grid)
    YAML_PATH.write_text(
        "image: ask3_lab.pgm\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{ORIGIN_X:.4f}, {ORIGIN_Y:.4f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    return grid


def world_to_map(x, y):
    return int((x - ORIGIN_X) / RESOLUTION), int((y - ORIGIN_Y) / RESOLUTION)


def inflate(grid, cells):
    occ = [[False for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for row in range(HEIGHT):
        for col in range(WIDTH):
            if grid[row * WIDTH + col] == OCCUPIED:
                occ[row][col] = True

    inflated = [[False for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for row in range(HEIGHT):
        for col in range(WIDTH):
            if not occ[row][col]:
                continue
            r0 = max(0, row - cells)
            r1 = min(HEIGHT - 1, row + cells)
            c0 = max(0, col - cells)
            c1 = min(WIDTH - 1, col + cells)
            for rr in range(r0, r1 + 1):
                for cc in range(c0, c1 + 1):
                    inflated[rr][cc] = True
    return inflated


def check_connectivity(grid):
    cells = int(math.ceil(CHECK_INFLATION / RESOLUTION))
    occ = inflate(grid, cells)
    start = world_to_map(*START)
    goals = [world_to_map(*goal) for goal in GOALS]

    def blocked(p):
        col, row_from_bottom = p
        row = HEIGHT - 1 - row_from_bottom
        return col < 0 or col >= WIDTH or row < 0 or row >= HEIGHT or occ[row][col]

    if blocked(start) or any(blocked(goal) for goal in goals):
        return False

    queue = deque([start])
    seen = {start}
    remaining = set(goals)
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while queue:
        cur = queue.popleft()
        remaining.discard(cur)
        if not remaining:
            return True
        for dx, dy in dirs:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt not in seen and not blocked(nxt):
                seen.add(nxt)
                queue.append(nxt)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="verify start-to-goal connectivity after 0.32 m inflation")
    args = parser.parse_args()

    write_world()
    grid = write_map()
    print(f"Wrote {WORLD_PATH}")
    print(f"Wrote {MAP_PATH}")
    print(f"Wrote {YAML_PATH}")
    print(f"Map size: {WIDTH}x{HEIGHT} px, {WIDTH * RESOLUTION:.2f}x{HEIGHT * RESOLUTION:.2f} m")
    if args.check:
        ok = check_connectivity(grid)
        print(f"Inflated connectivity from {START} to {len(GOALS)} goals: {'OK' if ok else 'FAILED'}")
        if not ok:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
