#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Improved A* (bidirectional + dynamic-weight + 24-neighbour + Bezier smoothing).

Identical structure to the wheeled-robot baseline (Tang Ningye thesis 2.5):
  1. Bidirectional search (2.5.1)
  2. Dynamic weighting f(n)=g(n)+(1+k/2L)*h(n) (2.5.2, eq. 2.10-2.12)
  3. 24-neighbour expansion (2.5.3)
  4. Cubic Bezier smoothing (2.5.4)

The ASK-3 adaptation keeps the thesis algorithmic structure, but treats
the quadruped footprint explicitly:
  * circular obstacle inflation by footprint radius instead of square
    cell dilation;
  * an additional soft clearance cost, so paths prefer the middle of
    usable corridors instead of grazing the inflated obstacle boundary;
  * a small turn penalty and line-of-sight shortcutting step to remove
    meaningless zig-zags before curve smoothing;
  * 24-neighbour moves are checked by a super-sampled line segment so
    long moves cannot cut a corner;
  * corner smoothing is accepted only when the smoothed curve remains in
    inflated free space, otherwise the shortcut path is used.
"""

import numpy as np
import heapq
import math
import rospy


class ImprovedAStar:
    def __init__(self, costmap_array, resolution, origin, width, height,
                 inflate_radius=0.22,
                 clearance_radius=0.50,
                 clearance_weight=1.20,
                 turn_weight=0.08,
                 shortcut_clearance=0.30,
                 resample_spacing=0.10,
                 chaikin_iterations=2):
        """
        costmap_array: 2D numpy (height x width), 0=free, 100=occupied, -1=unknown
        inflate_radius: obstacle inflation radius in metres. Default 0.22m
            covers the ASK-3 base half-diagonal (~0.225m) plus a small margin.
        """
        self.resolution = resolution
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.width = width
        self.height = height
        self.clearance_radius = clearance_radius
        self.clearance_weight = clearance_weight
        self.turn_weight = turn_weight
        self.shortcut_clearance = shortcut_clearance
        self.resample_spacing = resample_spacing
        self.chaikin_iterations = chaikin_iterations

        raw = np.where(costmap_array < 0, 100, costmap_array)
        inflate_cells = max(1, int(math.ceil(inflate_radius / resolution)))
        clearance_cells = max(1, int(math.ceil(clearance_radius / resolution)))
        self.clearance_map = self._distance_to_obstacles(raw, clearance_cells)
        self.costmap = self._inflate(raw, inflate_cells)
        self._segment_step = max(self.resolution * 0.5, 0.01)

        # Pre-compute the 24-neighbour offsets (8 inner + 16 outer)
        self._inner_offsets = []
        self._outer_offsets = []
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if dx == 0 and dy == 0:
                    continue
                dist = math.hypot(dx, dy)
                if max(abs(dx), abs(dy)) <= 1:
                    self._inner_offsets.append((dx, dy, dist))
                else:
                    self._outer_offsets.append((dx, dy, dist))

    @staticmethod
    def _inflate(grid, r):
        obstacle = (grid >= 80)
        inflated = np.zeros_like(obstacle, dtype=bool)
        offsets = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if math.hypot(dx, dy) <= r + 0.5:
                    offsets.append((dx, dy))

        for dx, dy in offsets:
            src_x0 = max(0, -dx)
            src_x1 = min(grid.shape[1], grid.shape[1] - dx)
            src_y0 = max(0, -dy)
            src_y1 = min(grid.shape[0], grid.shape[0] - dy)
            dst_x0 = max(0, dx)
            dst_x1 = min(grid.shape[1], grid.shape[1] + dx)
            dst_y0 = max(0, dy)
            dst_y1 = min(grid.shape[0], grid.shape[0] + dy)
            if src_x0 < src_x1 and src_y0 < src_y1:
                inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= \
                    obstacle[src_y0:src_y1, src_x0:src_x1]

        result = grid.copy()
        result[inflated > 0] = np.maximum(result[inflated > 0], 80)
        return result

    @staticmethod
    def _distance_to_obstacles(grid, max_cells):
        obstacle = (grid >= 80)
        dist = np.full(grid.shape, np.inf, dtype=float)
        heap = []
        ys, xs = np.where(obstacle)
        for y, x in zip(ys, xs):
            dist[y, x] = 0.0
            heapq.heappush(heap, (0.0, y, x))
        if not heap:
            return dist

        neighbours = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        ]
        height, width = grid.shape
        while heap:
            d, y, x = heapq.heappop(heap)
            if d != dist[y, x] or d >= max_cells:
                continue
            for dx, dy, step in neighbours:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                nd = d + step
                if nd <= max_cells and nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(heap, (nd, ny, nx))
        return dist

    def world_to_map(self, wx, wy):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = mx * self.resolution + self.origin_x + self.resolution * 0.5
        wy = my * self.resolution + self.origin_y + self.resolution * 0.5
        return wx, wy

    def is_obstacle(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return True
        return self.costmap[my, mx] >= 80

    def clearance_penalty(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return self.clearance_weight
        cells = self.clearance_map[my, mx]
        if not np.isfinite(cells):
            return 0.0
        clearance = cells * self.resolution
        if clearance >= self.clearance_radius:
            return 0.0
        ratio = 1.0 - clearance / max(self.clearance_radius, 1e-3)
        return self.clearance_weight * ratio * ratio

    def turn_penalty(self, parent, node, nxt):
        if parent is None or self.turn_weight <= 0.0:
            return 0.0
        v1x = node[0] - parent[0]
        v1y = node[1] - parent[1]
        v2x = nxt[0] - node[0]
        v2y = nxt[1] - node[1]
        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_da = (v1x * v2x + v1y * v2y) / (n1 * n2)
        cos_da = max(-1.0, min(1.0, cos_da))
        return self.turn_weight * (1.0 - cos_da)

    def _segment_is_free_map(self, x0, y0, x1, y1):
        steps = max(abs(x1 - x0), abs(y1 - y0)) * 2
        steps = max(1, int(steps))
        last = None
        for i in range(0, steps + 1):
            u = float(i) / float(steps)
            mx = int(round(x0 + (x1 - x0) * u))
            my = int(round(y0 + (y1 - y0) * u))
            node = (mx, my)
            if node == last:
                continue
            last = node
            if self.is_obstacle(mx, my):
                return False
        return True

    def segment_is_free_world(self, p0, p1):
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, int(math.ceil(dist / self._segment_step)))
        for i in range(0, steps + 1):
            u = float(i) / float(steps)
            wx = p0[0] + (p1[0] - p0[0]) * u
            wy = p0[1] + (p1[1] - p0[1]) * u
            mx, my = self.world_to_map(wx, wy)
            if self.is_obstacle(mx, my):
                return False
        return True

    def path_is_safe(self, path):
        if not path:
            return False
        prev = path[0]
        mx, my = self.world_to_map(prev[0], prev[1])
        if self.is_obstacle(mx, my):
            return False
        for p in path[1:]:
            mx, my = self.world_to_map(p[0], p[1])
            if self.is_obstacle(mx, my):
                return False
            if not self.segment_is_free_world(prev, p):
                return False
            prev = p
        return True

    def get_neighbors_24(self, x, y):
        neighbors = []
        for dx, dy, cost in self._inner_offsets:
            nx, ny = x + dx, y + dy
            if (not self.is_obstacle(nx, ny)
                    and self._segment_is_free_map(x, y, nx, ny)):
                neighbors.append((nx, ny, cost))
        for dx, dy, cost in self._outer_offsets:
            nx, ny = x + dx, y + dy
            if self.is_obstacle(nx, ny):
                continue
            if self._segment_is_free_map(x, y, nx, ny):
                neighbors.append((nx, ny, cost))
        return neighbors

    @staticmethod
    def heuristic(x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def bidirectional_astar(self, start_w, goal_w):
        sx, sy = self.world_to_map(start_w[0], start_w[1])
        gx, gy = self.world_to_map(goal_w[0], goal_w[1])

        if self.is_obstacle(sx, sy):
            rospy.logwarn("[A*] Start on obstacle, searching nearby free cell")
            sx, sy = self._find_free_nearby(sx, sy)
        if self.is_obstacle(gx, gy):
            rospy.logwarn("[A*] Goal on obstacle, searching nearby free cell")
            gx, gy = self._find_free_nearby(gx, gy)
        if self.is_obstacle(sx, sy) or self.is_obstacle(gx, gy):
            rospy.logerr("[A*] Cannot find free start/goal")
            return []

        L = float(abs(gx - sx) + abs(gy - sy))
        if L == 0:
            return [start_w, goal_w]

        counter = 0

        g_f = {}
        parent_f = {}
        closed_f = set()
        open_f = []
        g_f[(sx, sy)] = 0.0
        f_current = [sx, sy]

        g_b = {}
        parent_b = {}
        closed_b = set()
        open_b = []
        g_b[(gx, gy)] = 0.0
        b_current = [gx, gy]

        def dynamic_weight():
            k = abs(b_current[0] - f_current[0]) + abs(b_current[1] - f_current[1])
            return 1.0 + k / (2.0 * L)

        def push_f(node):
            nonlocal counter
            x, y = node
            g = g_f[node]
            h = self.heuristic(x, y, gx, gy)
            w = dynamic_weight()
            f = g + w * h
            counter += 1
            heapq.heappush(open_f, (f, counter, x, y))

        def push_b(node):
            nonlocal counter
            x, y = node
            g = g_b[node]
            h = self.heuristic(x, y, sx, sy)
            w = dynamic_weight()
            f = g + w * h
            counter += 1
            heapq.heappush(open_b, (f, counter, x, y))

        push_f((sx, sy))
        push_b((gx, gy))

        best_cost = float('inf')
        meet_node = None
        max_iter = self.width * self.height

        for _ in range(max_iter):
            if not open_f and not open_b:
                break

            if open_f:
                _, _, cx, cy = heapq.heappop(open_f)
                node = (cx, cy)
                if node not in closed_f:
                    closed_f.add(node)
                    f_current[0] = cx
                    f_current[1] = cy
                    if node in closed_b:
                        cost = g_f[node] + g_b[node]
                        if cost < best_cost:
                            best_cost = cost
                            meet_node = node
                        break
                    for nx, ny, step_cost in self.get_neighbors_24(cx, cy):
                        nnode = (nx, ny)
                        if nnode in closed_f:
                            continue
                        new_g = (g_f[node] + step_cost *
                                 (1.0 + self.clearance_penalty(nx, ny)) +
                                 self.turn_penalty(
                                     parent_f.get(node), node, nnode))
                        if nnode not in g_f or new_g < g_f[nnode]:
                            g_f[nnode] = new_g
                            parent_f[nnode] = node
                            push_f(nnode)
                            if nnode in closed_b:
                                cost = new_g + g_b[nnode]
                                if cost < best_cost:
                                    best_cost = cost
                                    meet_node = nnode

            if open_b:
                _, _, cx, cy = heapq.heappop(open_b)
                node = (cx, cy)
                if node not in closed_b:
                    closed_b.add(node)
                    b_current[0] = cx
                    b_current[1] = cy
                    if node in closed_f:
                        cost = g_f[node] + g_b[node]
                        if cost < best_cost:
                            best_cost = cost
                            meet_node = node
                        break
                    for nx, ny, step_cost in self.get_neighbors_24(cx, cy):
                        nnode = (nx, ny)
                        if nnode in closed_b:
                            continue
                        new_g = (g_b[node] + step_cost *
                                 (1.0 + self.clearance_penalty(nx, ny)) +
                                 self.turn_penalty(
                                     parent_b.get(node), node, nnode))
                        if nnode not in g_b or new_g < g_b[nnode]:
                            g_b[nnode] = new_g
                            parent_b[nnode] = node
                            push_b(nnode)
                            if nnode in closed_f:
                                cost = g_f[nnode] + new_g
                                if cost < best_cost:
                                    best_cost = cost
                                    meet_node = nnode

        if meet_node is None:
            rospy.logerr("[A*] Bidirectional A* failed: no path found")
            return []

        rospy.loginfo("[A*] meet at %s, expanded %d+%d nodes",
                      meet_node, len(closed_f), len(closed_b))

        path_f = []
        node = meet_node
        while node in parent_f:
            path_f.append(node)
            node = parent_f[node]
        path_f.append((sx, sy))
        path_f.reverse()

        path_b = []
        node = meet_node
        while node in parent_b:
            node = parent_b[node]
            path_b.append(node)

        full_map_path = path_f + path_b
        return [self.map_to_world(x, y) for (x, y) in full_map_path]

    def _find_free_nearby(self, mx, my, radius=15):
        for r in range(1, radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) == r:
                        nx, ny = mx + dx, my + dy
                        if not self.is_obstacle(nx, ny):
                            return nx, ny
        return mx, my

    @staticmethod
    def extract_key_points(path, angle_thresh=0.15, dist_thresh=0.4):
        """
        For an omnidirectional quadruped we can space the local sub-goals
        more loosely than for a TurtleBot, because there is no minimum
        turning radius. dist_thresh is therefore raised from 0.30 m
        (wheeled) to 0.40 m here.
        """
        if len(path) <= 2:
            return list(path)
        keys = [path[0]]
        for i in range(1, len(path) - 1):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            a1 = math.atan2(y1 - y0, x1 - x0)
            a2 = math.atan2(y2 - y1, x2 - x1)
            da = abs(a2 - a1)
            if da > math.pi:
                da = 2 * math.pi - da
            if da > angle_thresh:
                keys.append(path[i])
            elif math.hypot(x1 - keys[-1][0], y1 - keys[-1][1]) > dist_thresh:
                keys.append(path[i])
        keys.append(path[-1])
        return keys

    @staticmethod
    def bezier_smooth(path_points, num_interp=8):
        if len(path_points) < 4:
            return list(path_points)

        smoothed = [path_points[0]]
        i = 0
        while i + 3 < len(path_points):
            p0 = path_points[i]
            p1 = path_points[i + 1]
            p2 = path_points[i + 2]
            p3 = path_points[i + 3]
            for t_idx in range(1, num_interp + 1):
                u = t_idx / num_interp
                u2 = u * u
                u3 = u2 * u
                inv = 1 - u
                inv2 = inv * inv
                inv3 = inv2 * inv
                x = inv3*p0[0] + 3*inv2*u*p1[0] + 3*inv*u2*p2[0] + u3*p3[0]
                y = inv3*p0[1] + 3*inv2*u*p1[1] + 3*inv*u2*p2[1] + u3*p3[1]
                smoothed.append((x, y))
            i += 3
        for j in range(i + 1, len(path_points)):
            smoothed.append(path_points[j])
        unique = [smoothed[0]]
        for p in smoothed[1:]:
            if math.hypot(p[0] - unique[-1][0], p[1] - unique[-1][1]) > 0.01:
                unique.append(p)
        return unique

    def segment_min_clearance_world(self, p0, p1):
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, int(math.ceil(dist / self._segment_step)))
        best = float('inf')
        for i in range(0, steps + 1):
            u = float(i) / float(steps)
            wx = p0[0] + (p1[0] - p0[0]) * u
            wy = p0[1] + (p1[1] - p0[1]) * u
            mx, my = self.world_to_map(wx, wy)
            if self.is_obstacle(mx, my):
                return 0.0
            if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
                return 0.0
            cells = self.clearance_map[my, mx]
            if np.isfinite(cells):
                best = min(best, cells * self.resolution)
        return self.clearance_radius if best == float('inf') else best

    def shortcut_path(self, path, min_clearance=None):
        if len(path) <= 2:
            return list(path)
        if min_clearance is None:
            min_clearance = self.shortcut_clearance

        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            best = i + 1
            for j in range(len(path) - 1, i, -1):
                if not self.segment_is_free_world(path[i], path[j]):
                    continue
                if (min_clearance <= 0.0 or
                        self.segment_min_clearance_world(path[i], path[j]) >= min_clearance):
                    best = j
                    break
            if best == i + 1 and min_clearance > 0.0:
                for j in range(len(path) - 1, i, -1):
                    if self.segment_is_free_world(path[i], path[j]):
                        best = j
                        break
            result.append(path[best])
            i = best
        return result

    @staticmethod
    def chaikin_smooth(path, iterations=2, weight=0.25):
        if len(path) <= 2 or iterations <= 0:
            return list(path)
        pts = list(path)
        for _ in range(iterations):
            smoothed = [pts[0]]
            for p0, p1 in zip(pts[:-1], pts[1:]):
                qx = (1.0 - weight) * p0[0] + weight * p1[0]
                qy = (1.0 - weight) * p0[1] + weight * p1[1]
                rx = weight * p0[0] + (1.0 - weight) * p1[0]
                ry = weight * p0[1] + (1.0 - weight) * p1[1]
                smoothed.append((qx, qy))
                smoothed.append((rx, ry))
            smoothed.append(pts[-1])
            pts = smoothed
        return pts

    def resample_path(self, path, spacing=None):
        if len(path) <= 1:
            return list(path)
        if spacing is None:
            spacing = self.resample_spacing
        spacing = max(spacing, self.resolution)

        resampled = [path[0]]
        for p0, p1 in zip(path[:-1], path[1:]):
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            steps = max(1, int(math.ceil(dist / spacing)))
            for i in range(1, steps + 1):
                u = float(i) / float(steps)
                p = (p0[0] + (p1[0] - p0[0]) * u,
                     p0[1] + (p1[1] - p0[1]) * u)
                if math.hypot(p[0] - resampled[-1][0],
                              p[1] - resampled[-1][1]) > 0.01:
                    resampled.append(p)
        return resampled

    def plan(self, start_w, goal_w):
        raw_path = self.bidirectional_astar(start_w, goal_w)
        if not raw_path:
            return [], []
        shortcut_path = self.shortcut_path(raw_path)
        smooth_base = shortcut_path
        for iterations in range(self.chaikin_iterations, 0, -1):
            candidate = self.chaikin_smooth(shortcut_path, iterations=iterations)
            if self.path_is_safe(candidate):
                smooth_base = candidate
                break
        if smooth_base is shortcut_path and len(shortcut_path) > 2:
            rospy.logwarn("[A*] Corner smoothing unsafe; using line-of-sight shortcut path")
        smooth_path = self.resample_path(smooth_base)
        key_points = self.extract_key_points(smooth_path)
        rospy.loginfo("[A*] %d raw -> %d shortcut -> %d smooth -> %d key points",
                      len(raw_path), len(shortcut_path),
                      len(smooth_path), len(key_points))
        return smooth_path, key_points
