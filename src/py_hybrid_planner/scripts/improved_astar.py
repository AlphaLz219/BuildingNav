#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进 A* 算法 -- 复现汤宁业论文第二章
特性:
  1. 双向搜索策略 (2.5.1)
  2. 动态权重启发函数 f(n)=g(n)+(1+k/2L)*h(n) (2.5.2, 式2.10-2.12)
  3. 24邻域扩展搜索 (2.5.3)
  4. 三阶贝塞尔曲线路径平滑 (2.5.4)
"""

import numpy as np
import heapq
import math
import rospy


class ImprovedAStar:
    def __init__(self, costmap_array, resolution, origin, width, height,
                 inflate_radius=0.12):
        """
        costmap_array: 2D numpy (height x width), 0=free, 100=occupied, -1=unknown
        inflate_radius: 障碍物膨胀半径(m)
        """
        self.resolution = resolution
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.width = width
        self.height = height

        # 构建膨胀代价地图
        raw = np.where(costmap_array < 0, 100, costmap_array)
        inflate_cells = max(1, int(inflate_radius / resolution))
        self.costmap = self._inflate(raw, inflate_cells)

        # 预计算 24 邻域偏移 (内层8 + 外层16)
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
        """高效最大值滤波膨胀 (两次 1D 滚动取最大值)"""
        obstacle = (grid >= 80).astype(np.uint8)
        # 水平膨胀
        h_inflated = obstacle.copy()
        for dr in range(1, r + 1):
            h_inflated[dr:, :] |= obstacle[:-dr, :]
            h_inflated[:-dr, :] |= obstacle[dr:, :]
        # 垂直膨胀
        inflated = h_inflated.copy()
        for dr in range(1, r + 1):
            inflated[:, dr:] |= h_inflated[:, :-dr]
            inflated[:, :-dr] |= h_inflated[:, dr:]
        result = grid.copy()
        result[inflated > 0] = np.maximum(result[inflated > 0], 80)
        return result

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

    def get_neighbors_24(self, x, y):
        """
        论文 2.5.3: 内层8邻域 + 外层16邻域
        外层节点需确保途经内层节点均可通过
        """
        neighbors = []
        # 内层 8 邻域
        for dx, dy, cost in self._inner_offsets:
            nx, ny = x + dx, y + dy
            if not self.is_obstacle(nx, ny):
                neighbors.append((nx, ny, cost))
        # 外层 16 邻域
        for dx, dy, cost in self._outer_offsets:
            nx, ny = x + dx, y + dy
            if self.is_obstacle(nx, ny):
                continue
            # 检查途径内层节点 (论文图2.13)
            blocked = False
            steps = max(abs(dx), abs(dy))
            for s in range(1, steps):
                ix = x + int(round(dx * s / steps))
                iy = y + int(round(dy * s / steps))
                if self.is_obstacle(ix, iy):
                    blocked = True
                    break
            if not blocked:
                neighbors.append((nx, ny, cost))
        return neighbors

    @staticmethod
    def heuristic(x1, y1, x2, y2):
        """欧几里得距离 (论文式2.9)"""
        return math.hypot(x1 - x2, y1 - y2)

    def bidirectional_astar(self, start_w, goal_w):
        """
        论文 2.5.1 + 2.5.2: 双向搜索 + 动态权重
        f(n) = g(n) + (1 + k/(2L)) * h(n)
        k = |gx-sx| + |gy-sy| (正/反向当前节点曼哈顿距离)
        L = |xgoal-xstart| + |ygoal-ystart|
        """
        sx, sy = self.world_to_map(start_w[0], start_w[1])
        gx, gy = self.world_to_map(goal_w[0], goal_w[1])

        if self.is_obstacle(sx, sy):
            rospy.logwarn("Start on obstacle, searching nearby free cell")
            sx, sy = self._find_free_nearby(sx, sy)
        if self.is_obstacle(gx, gy):
            rospy.logwarn("Goal on obstacle, searching nearby free cell")
            gx, gy = self._find_free_nearby(gx, gy)
        if self.is_obstacle(sx, sy) or self.is_obstacle(gx, gy):
            rospy.logerr("Cannot find free start/goal")
            return []

        # L: 起终点曼哈顿距离 (论文式2.12)
        L = float(abs(gx - sx) + abs(gy - sy))
        if L == 0:
            return [start_w, goal_w]

        counter = 0

        # 正向数据
        g_f = {}
        parent_f = {}
        closed_f = set()
        open_f = []
        g_f[(sx, sy)] = 0.0
        f_current = [sx, sy]

        # 反向数据
        g_b = {}
        parent_b = {}
        closed_b = set()
        open_b = []
        g_b[(gx, gy)] = 0.0
        b_current = [gx, gy]

        def dynamic_weight():
            """论文式 2.10: w = 1 + k / (2L)"""
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

        for iteration in range(max_iter):
            if not open_f and not open_b:
                break

            # 正向扩展
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
                        new_g = g_f[node] + step_cost
                        if nnode not in g_f or new_g < g_f[nnode]:
                            g_f[nnode] = new_g
                            parent_f[nnode] = node
                            push_f(nnode)
                            if nnode in closed_b:
                                cost = new_g + g_b[nnode]
                                if cost < best_cost:
                                    best_cost = cost
                                    meet_node = nnode

            # 反向扩展
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
                        new_g = g_b[node] + step_cost
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
            rospy.logerr("Bidirectional A* failed: no path found")
            return []

        rospy.loginfo("A* meet at %s, expanded %d+%d nodes",
                      meet_node, len(closed_f), len(closed_b))

        # 重建路径
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
        world_path = [self.map_to_world(x, y) for (x, y) in full_map_path]
        return world_path

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
    def extract_key_points(path, angle_thresh=0.15, dist_thresh=0.3):
        """
        论文4.2: 从全局路径中筛选关键点作为DWA局部目标
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
        """
        论文 2.5.4: 连续三阶贝塞尔曲线平滑
        """
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

    def plan(self, start_w, goal_w):
        """完整规划: 双向A* -> 贝塞尔平滑 -> 关键点"""
        raw_path = self.bidirectional_astar(start_w, goal_w)
        if not raw_path:
            return [], []
        smooth_path = self.bezier_smooth(raw_path)
        key_points = self.extract_key_points(smooth_path)
        rospy.loginfo("A* path: %d raw -> %d smooth -> %d key points",
                      len(raw_path), len(smooth_path), len(key_points))
        return smooth_path, key_points
