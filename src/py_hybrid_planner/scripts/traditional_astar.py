#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traditional A* (baseline for comparison with the paper's improved version).

Reference implementation of classical A*:
- 8-neighbour expansion
- Constant weight: f(n) = g(n) + h(n), Euclidean heuristic
- No smoothing, no key-point extraction (path is a dense list of cell centres)
- Inflation radius identical to the improved planner so comparisons are fair

Exposes the same plan(start, goal) API as ImprovedAStar and also publishes
metrics via the `last_metrics` attribute.
"""

import numpy as np
import heapq
import math
import time


class TraditionalAStar:
    def __init__(self, costmap_array, resolution, origin, width, height,
                 inflate_radius=0.12):
        self.resolution = resolution
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.width = width
        self.height = height

        raw = np.where(costmap_array < 0, 100, costmap_array)
        inflate_cells = max(1, int(inflate_radius / resolution))
        self.costmap = self._inflate(raw, inflate_cells)

        self._neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                self._neighbors.append((dx, dy, math.hypot(dx, dy)))

        self.last_metrics = {}

    @staticmethod
    def _inflate(grid, r):
        obstacle = (grid >= 80).astype(np.uint8)
        h = obstacle.copy()
        for dr in range(1, r + 1):
            h[dr:, :] |= obstacle[:-dr, :]
            h[:-dr, :] |= obstacle[dr:, :]
        inflated = h.copy()
        for dr in range(1, r + 1):
            inflated[:, dr:] |= h[:, :-dr]
            inflated[:, :-dr] |= h[:, dr:]
        out = grid.copy()
        out[inflated > 0] = np.maximum(out[inflated > 0], 80)
        return out

    def world_to_map(self, wx, wy):
        return int((wx - self.origin_x) / self.resolution), int((wy - self.origin_y) / self.resolution)

    def map_to_world(self, mx, my):
        return mx * self.resolution + self.origin_x + self.resolution * 0.5, \
               my * self.resolution + self.origin_y + self.resolution * 0.5

    def is_obstacle(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return True
        return self.costmap[my, mx] >= 80

    @staticmethod
    def _heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start_world, goal_world):
        t0 = time.perf_counter()
        sx, sy = self.world_to_map(*start_world)
        gx, gy = self.world_to_map(*goal_world)
        if self.is_obstacle(sx, sy) or self.is_obstacle(gx, gy):
            self.last_metrics = {"ok": False, "reason": "start/goal occupied"}
            return [], []

        open_heap = []
        heapq.heappush(open_heap, (self._heuristic((sx, sy), (gx, gy)), 0.0, (sx, sy)))
        came_from = {}
        gscore = {(sx, sy): 0.0}
        closed = set()
        expanded = 0
        goal = (gx, gy)

        while open_heap:
            f, g, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            expanded += 1
            if cur == goal:
                break
            for dx, dy, step in self._neighbors:
                nx, ny = cur[0] + dx, cur[1] + dy
                if self.is_obstacle(nx, ny):
                    continue
                tentative = g + step
                if tentative < gscore.get((nx, ny), float('inf')):
                    gscore[(nx, ny)] = tentative
                    came_from[(nx, ny)] = cur
                    f_new = tentative + self._heuristic((nx, ny), goal)
                    heapq.heappush(open_heap, (f_new, tentative, (nx, ny)))

        if goal not in came_from and (sx, sy) != goal:
            self.last_metrics = {"ok": False, "reason": "no path", "expanded": expanded,
                                 "planning_time_s": time.perf_counter() - t0}
            return [], []

        path_cells = [goal]
        while path_cells[-1] != (sx, sy):
            path_cells.append(came_from[path_cells[-1]])
        path_cells.reverse()
        path_world = [self.map_to_world(cx, cy) for cx, cy in path_cells]

        # key points = every 10th cell, plus goal (so DWA has something to track)
        key = path_world[::10]
        if key[-1] != path_world[-1]:
            key.append(path_world[-1])

        # metrics
        length = sum(math.hypot(path_world[i+1][0]-path_world[i][0],
                                path_world[i+1][1]-path_world[i][1])
                     for i in range(len(path_world)-1))
        angle_change = 0.0
        turns = 0
        for i in range(1, len(path_world) - 1):
            a = math.atan2(path_world[i][1] - path_world[i-1][1],
                           path_world[i][0] - path_world[i-1][0])
            b = math.atan2(path_world[i+1][1] - path_world[i][1],
                           path_world[i+1][0] - path_world[i][0])
            d = abs(b - a)
            if d > math.pi:
                d = 2 * math.pi - d
            angle_change += d
            if d > math.radians(20):
                turns += 1
        self.last_metrics = {
            "ok": True,
            "expanded": expanded,
            "path_length_m": length,
            "total_turn_rad": angle_change,
            "num_turns": turns,
            "planning_time_s": time.perf_counter() - t0,
            "raw_cells": len(path_cells),
        }
        return path_world, key
