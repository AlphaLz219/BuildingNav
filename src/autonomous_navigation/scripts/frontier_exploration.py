#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前沿探索节点（Frontier Exploration Node）
=========================================

功能流程：
  1. 订阅 SLAM Toolbox 发布的 /map（OccupancyGrid）
  2. 检测前沿 —— 已知自由空间与未知空间的交界
  3. 对前沿点做连通域聚类，过滤噪声小簇
  4. 用比值法评分（Score = Gain / (Cost + epsilon)）选出最优前沿簇
  5. 通过 move_base ActionClient 驱动机器人前往
  6. 到达后地图更新，重新检测前沿，循环直到探索完毕

依赖：rospy, numpy, cv2, actionlib, tf, move_base_msgs
"""

import rospy
import numpy as np
import cv2
import actionlib
import tf

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import ColorRGBA


# ═══════════════════════════════════════════════════════════
#  前沿探索主类
# ═══════════════════════════════════════════════════════════
class FrontierExplorer:
    """自主前沿探索：检测地图前沿 → 选目标 → 导航 → 循环"""

    # ────────────── 初始化 ──────────────
    def __init__(self):
        self._load_params()
        self._init_state()
        self._init_ros_interface()
        rospy.loginfo("[前沿探索] 节点初始化完成，等待地图数据...")

    def _load_params(self):
        """从参数服务器加载可调参数（带默认值）"""
        # 前沿簇最小像素数，低于此值视为噪声
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 20)
        # 前沿检测频率（Hz），不需要太高，地图更新较慢
        self.detection_freq = rospy.get_param('~detection_frequency', 0.5)
        # ── 比值法评分参数: Score = Gain / (Cost + epsilon) ──
        # 计算 Gain 时，以簇目标点为中心膨胀的半径（栅格数）
        self.gain_unknown_radius = rospy.get_param('~gain_unknown_radius', 15)
        # 方向惩罚系数 λ，掉头（180°）时 Cost 增加此比例
        self.direction_penalty = rospy.get_param('~direction_penalty', 0.4)
        # 分母常数（米），防止近距离前沿得分爆炸
        self.cost_epsilon = rospy.get_param('~cost_epsilon', 0.5)
        # Cost 使用 make_plan 真实路径长度（True）还是欧氏距离（False）
        self.use_path_length = rospy.get_param('~use_path_length', True)
        # 连续无前沿次数阈值，超过则判定探索完成
        self.max_no_frontier = rospy.get_param('~max_no_frontier_count', 5)
        # 导航超时（秒），超过则取消当前目标
        self.nav_timeout = rospy.get_param('~navigation_timeout', 120.0)
        # 黑名单半径（米），导航失败的目标周围不再重复尝试
        self.blacklist_radius = rospy.get_param('~blacklist_radius', 1.5)
        # 距机器人太近的前沿不选（已在附近，无需导航）
        self.min_goal_distance = rospy.get_param('~min_goal_distance', 0.8)
        # 目标点距最近障碍物的最小净空（米），低于此值的簇直接跳过
        self.min_goal_clearance = rospy.get_param('~min_goal_clearance', 0.3)
        # 梯度推进最大步数（从簇内最深点沿距离梯度向内走），0=关闭推进
        self.max_push_steps = rospy.get_param('~max_push_steps', 20)
        # make_plan 预检查超时（秒），超时则假定不可达
        self.plan_check_timeout = rospy.get_param('~plan_check_timeout', 2.0)
        # 建图期周期性重规划间隔（秒），强制 move_base 基于最新地图刷新全局路径
        # 设为 0 或负数则关闭此功能
        self.replan_interval = rospy.get_param('~replan_interval', 6.0)
        # 距目标小于此距离时停止重规划，避免接近目标时反复取消导致振荡
        self.replan_min_dist = rospy.get_param('~replan_min_distance', 1.5)
        # ── 目标过时检测 ──
        # Gain 衰减阈值，当前 Gain 低于出发时 Gain × 此比例即判定过时
        self.gain_decay_threshold = rospy.get_param(
            '~gain_decay_threshold', 0.3)
        # 距目标小于此距离（米）时才开始检查过时，避免远距离误判
        self.stale_check_distance = rospy.get_param(
            '~stale_check_distance', 3.0)
        # 过时检查频率（秒），独立于重规划周期，可更快响应目标过时
        self.stale_check_interval = rospy.get_param(
            '~stale_check_interval', 2.0)
        # 重规划前要求 /map 至少更新过一次（避免 SLAM 未就绪时反复重发）
        self._last_map_stamp = None

        # ── 启动前初始化扫描 ──
        # 前进距离（米），设为 0 则跳过前进
        self.init_forward_distance = rospy.get_param('~init_forward_distance', 3.0)
        # 前进速度（m/s）
        self.init_forward_speed    = rospy.get_param('~init_forward_speed', 0.3)
        # 是否在启动前旋转 360° 扫描
        self.init_rotate_enabled   = rospy.get_param('~init_rotate_enabled', True)
        # 调试模式：完成初始扫描后暂停，机器人原地待命，不进入探索循环
        # 设为 True 后，需手动 kill 节点或置 False 重新启动
        self.init_pause_enabled    = rospy.get_param('~init_pause_enabled', True)

        # ── 走廊入口检测 ──
        # 走廊宽度判定阈值（米），沿 y 轴扫描时宽度首次低于此值即视为进入走廊
        self.corridor_width_thresh = rospy.get_param('~corridor_width_thresh', 3.0)
        # 扫描步长（米），沿 y 轴向前扫描时的采样间隔
        self.corridor_scan_step    = rospy.get_param('~corridor_scan_step', 0.25)
        # 连续窄判定次数，连续 N 个点宽度 < 阈值才确认进入走廊
        self.corridor_narrow_count = rospy.get_param('~corridor_narrow_count', 3)
        # 走廊深入距离（米），入口检测完成后向走廊内推进的距离
        # 用于在房门扫描前让 LiDAR 覆盖更多走廊区域
        self.corridor_entry_depth = rospy.get_param('~corridor_entry_depth', 5.0)

    def _init_state(self):
        """初始化内部状态变量"""
        # 地图相关
        self.map_data = None          # 2D numpy 数组 (height × width)
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # 距离变换缓存（detect_frontiers 计算，select_best_goal 使用）
        self._dist_transform = None

        # 探索状态
        self.is_navigating = False     # 是否正在导航中
        self.no_frontier_count = 0     # 连续无前沿计数
        self.exploration_complete = False
        self.blacklist = []            # 导航失败的目标列表 [(x, y), ...]
        self.goal_start_time = None    # 当前导航开始时间（用于超时检测）

        # 周期性重规划状态
        self.current_goal_xy = None    # (x, y) 当前正在导航的目标坐标
        self.last_replan_time = None   # 上次重规划触发时刻
        self.goal_gain = None          # 发送目标时记录的初始 Gain（用于过时检测）
        self.last_stale_check_time = None  # 上次过时检查时刻
        self.forbidden_zones = []      # [(x_min, x_max, y_min, y_max), ...]
        self.door_positions = []       # [(door_x, door_y, side), ...] side='left'/'right'

    def _init_ros_interface(self):
        """初始化 ROS 接口：TF、ActionClient、发布器、订阅器"""
        # TF 监听器 —— 查询机器人位姿
        self.tf_listener = tf.TransformListener()

        # move_base ActionClient —— 发送导航目标并获取结果
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[前沿探索] 等待 move_base 服务启动...")
        self.ac.wait_for_server()
        rospy.loginfo("[前沿探索] move_base 已连接")

        # make_plan 服务 —— 发送目标前预检查可达性
        rospy.loginfo("[前沿探索] 等待 /move_base/make_plan 服务...")
        rospy.wait_for_service('/move_base/make_plan', timeout=10.0)
        self._plan_service = rospy.ServiceProxy(
            '/move_base/make_plan', GetPlan)
        rospy.loginfo("[前沿探索] /move_base/make_plan 已连接")

        # 发布器 —— 前沿标记可视化（MarkerArray）
        self.marker_pub = rospy.Publisher(
            '/frontier_markers', MarkerArray, queue_size=1, latch=True)
        # 发布器 —— 当前探索目标标记
        self.goal_marker_pub = rospy.Publisher(
            '/current_goal_marker', Marker, queue_size=1, latch=True)
        # 发布器 —— 直接发送零速度强制停车（取消目标后使用）
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        # 发布器 —— 禁区可视化（独立 topic，避免被 frontier markers 覆盖）
        self.forbidden_pub = rospy.Publisher(
            '/forbidden_zone_markers', MarkerArray, queue_size=1, latch=True)

        # 订阅器 —— SLAM Toolbox 的栅格地图
        rospy.Subscriber('/map', OccupancyGrid, self._map_callback)

    # ────────────── 地图回调 ──────────────
    def _map_callback(self, msg):
        """
        接收 SLAM Toolbox 发布的栅格地图

        OccupancyGrid.data 值含义：
            0   = 自由空间（已探索，可通行）
            100 = 障碍物（墙壁等）
            -1  = 未知空间（未探索）
        """
        self._last_map_stamp = msg.header.stamp
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        # 将 flat list 转为 2D numpy 数组（行=height, 列=width）
        # 使用 int8 保留 -1（未知）的值
        raw = np.array(msg.data, dtype=np.int8)
        self.map_data = raw.reshape((self.map_height, self.map_width))

        # 每次地图更新后重新标记禁区，避免被 SLAM 覆盖
        if self.forbidden_zones:
            self._mark_forbidden_zones()
            self._publish_forbidden_zone_markers()

    # ────────────── 坐标转换 ──────────────
    def _grid_to_world(self, col, row):
        """栅格坐标 (col, row) → 世界坐标 (x, y)"""
        x = self.map_origin_x + (col + 0.5) * self.map_resolution
        y = self.map_origin_y + (row + 0.5) * self.map_resolution
        return x, y

    def _push_inward(self, start_row, start_col):
        """
        沿距离变换梯度向内推进，从边界走向通道/房间的几何中心。

        每一步检查 8 邻域，移动到距离变换值最大的邻居。
        当周围没有更大的值（到达局部最大）或达到最大步数时停止。
        如果推进过程中踩到非自由空间则提前终止。

        返回：(final_row, final_col)
        """
        if self._dist_transform is None or self.max_push_steps <= 0:
            return start_row, start_col

        r, c = start_row, start_col
        h, w = self._dist_transform.shape

        for _ in range(self.max_push_steps):
            current_val = self._dist_transform[r, c]

            # 检查 8 邻域，找最大值
            best_val = current_val
            best_r, best_c = r, c

            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        nv = self._dist_transform[nr, nc]
                        if nv > best_val:
                            best_val = nv
                            best_r, best_c = nr, nc

            # 没有更大的邻居 → 到达局部最大值，停止
            if best_val <= current_val:
                break

            # 安全检查：目标必须是自由空间
            if self.map_data[best_r, best_c] != 0:
                break

            r, c = best_r, best_c

        return r, c

    def _get_robot_pose(self):
        """
        通过 TF 查询机器人当前位置和朝向（map 坐标系）
        返回 (x, y, yaw) 或 (None, None, None) 如果查询失败
        yaw 单位为弧度，范围 [-π, π]，0 = 朝 x 正方向
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            (_, _, yaw) = tf.transformations.euler_from_quaternion(
                [rot[0], rot[1], rot[2], rot[3]])
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn("[前沿探索] TF 查询 /map → /base_footprint 失败")
            return None, None, None

    # ────────────── 前沿检测 ──────────────
    def detect_frontiers(self):
        """
        检测前沿点并聚类，用距离变换找到每个簇的"最深处"作为导航目标。

        前沿定义：自由空间栅格，且至少有一个 4-邻域栅格是未知空间。

        算法步骤：
          1. 创建 free_mask 和 unknown_mask
          2. 计算自由空间的距离变换（每个像素 = 距最近障碍物/未知的像素数）
          3. 膨胀未知空间，前沿 = 膨胀(未知) ∩ 自由
          4. 连通域聚类
          5. 过滤小簇；从每个簇中选距离变换值最大的点（最深处）作为目标
          6. 过滤净空不足的簇

        返回：[(world_x, world_y, cluster_size), ...]
        """
        if self.map_data is None:
            return []

        # 二值掩码
        free_mask = (self.map_data == 0).astype(np.uint8)       # 自由空间
        unknown_mask = (self.map_data == -1).astype(np.uint8)   # 未知空间

        # ── 距离变换：每个自由栅格的值 = 到最近墙壁的欧氏距离（像素）──
        # 注意：只把障碍物(100)当作"墙"，未知(-1)视为可达空间
        # 这样前沿目标会被推到远离墙壁的通道中央，而非远离未知边界
        obstacle_mask = (self.map_data == 100).astype(np.uint8)
        self._dist_transform = cv2.distanceTransform(
            1 - obstacle_mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        )
        # 自由空间中值 ≥1，障碍物上值为 0

        # 膨胀未知空间 1 像素（3×3 核）
        kernel = np.ones((3, 3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel, iterations=1)

        # 前沿 = 膨胀后的未知区域 ∩ 自由区域
        frontier_mask = (dilated_unknown & free_mask).astype(np.uint8)

        # 连通域分析
        num_labels, labels = cv2.connectedComponents(frontier_mask)

        # 最小净空（栅格数），低于此值的簇视为"贴墙"直接跳过
        min_clearance_cells = self.min_goal_clearance / self.map_resolution

        frontiers = []
        for label_id in range(1, num_labels):  # label 0 是背景
            rows, cols = np.where(labels == label_id)
            size = len(rows)

            # 过滤太小的簇
            if size < self.min_frontier_size:
                continue

            # ── 在簇内找距离变换值最大的像素（即离所有障碍物最远的地方）──
            cluster_dists = self._dist_transform[rows, cols]
            best_idx = np.argmax(cluster_dists)
            center_row = rows[best_idx]
            center_col = cols[best_idx]
            max_clearance = cluster_dists[best_idx] * self.map_resolution  # 米

            # ── 安全检查：净空不足则跳过 ──
            if max_clearance < self.min_goal_clearance:
                rospy.loginfo(
                    "[前沿探索] 跳过贴墙前沿簇 (size=%d)，最大净空仅 %.2fm",
                    size, max_clearance)
                continue

            # ── 梯度推进：沿距离梯度向内走，逼近通道/房间中心 ──
            pushed_r, pushed_c = self._push_inward(center_row, center_col)
            pushed_clearance = (
                self._dist_transform[pushed_r, pushed_c] * self.map_resolution)
            rospy.loginfo(
                "[前沿探索] 梯度推进: (%.2f,%.2f)[%.2fm] → (%.2f,%.2f)[%.2fm]",
                self._grid_to_world(center_col, center_row)[0],
                self._grid_to_world(center_col, center_row)[1],
                max_clearance,
                self._grid_to_world(pushed_c, pushed_r)[0],
                self._grid_to_world(pushed_c, pushed_r)[1],
                pushed_clearance)

            # 转换为世界坐标
            wx, wy = self._grid_to_world(pushed_c, pushed_r)
            frontiers.append((wx, wy, size))

        return frontiers

    # ────────────── 信息增益计算 ──────────────
    def _compute_gain(self, world_x, world_y):
        """
        计算目标点周围的信息增益（未知像素占比）。

        以 (world_x, world_y) 为中心，在 gain_unknown_radius 栅格半径的
        圆形区域内，统计未知空间像素占有效像素（非地图边界外）的比例。

        返回：0.0 ~ 1.0，值越高表示该区域越"未知"，探索价值越大
        """
        if self.map_data is None:
            return 0.0

        col = int((world_x - self.map_origin_x) / self.map_resolution)
        row = int((world_y - self.map_origin_y) / self.map_resolution)

        r = self.gain_unknown_radius
        h, w = self.map_data.shape

        # 裁剪到地图边界
        r_min = max(0, row - r)
        r_max = min(h, row + r + 1)
        c_min = max(0, col - r)
        c_max = min(w, col + r + 1)

        if r_min >= r_max or c_min >= c_max:
            return 0.0

        patch = self.map_data[r_min:r_max, c_min:c_max]

        # 构建圆形掩码
        rr, cc = np.mgrid[r_min:r_max, c_min:c_max]
        dist_sq = (rr - row)**2 + (cc - col)**2
        circle = dist_sq <= r * r

        valid_pixels = np.sum(circle)
        if valid_pixels == 0:
            return 0.0

        unknown_pixels = np.sum(circle & (patch == -1))
        return float(unknown_pixels) / float(valid_pixels)

    # ────────────── 目标选择 ──────────────
    def select_best_goal(self, frontiers):
        """
        从前沿簇中选择评分最高的导航目标（比值法）。

        评分函数：
            Score = Gain / (Cost + epsilon)

            Gain = 目标点周围圆形区域内未知像素占比 (0~1)
            Cost = distance × (1 + direction_penalty × angle_diff / 180)
                   distance 为 make_plan 路径长度或欧氏距离
            epsilon = cost_epsilon，防止分母趋近零

        流程：
          1. 过滤黑名单和过近的前沿
          2. 计算 Gain 和初步 Cost（欧氏距离 + 方向惩罚）
          3. 按初步评分排序，依次调用 make_plan 验证可达性
          4. 若 use_path_length=True，用真实路径长度重算 Cost
          5. 返回最终评分最高的目标

        返回：(goal_x, goal_y, score) 或 None
        """
        if not frontiers:
            return None

        robot_x, robot_y, robot_yaw = self._get_robot_pose()
        if robot_x is None:
            return None

        candidates = []
        for (fx, fy, fsize) in frontiers:
            if self._is_blacklisted(fx, fy):
                continue

            dist = np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            if dist < self.min_goal_distance:
                continue

            # 计算信息增益
            gain = self._compute_gain(fx, fy)

            # 方向惩罚：计算目标方位角与机器人朝向的差
            if robot_yaw is not None:
                target_angle = np.arctan2(fy - robot_y, fx - robot_x)
                angle_diff = abs(target_angle - robot_yaw)
                if angle_diff > np.pi:
                    angle_diff = 2.0 * np.pi - angle_diff
                angle_deg = np.degrees(angle_diff)
            else:
                angle_deg = 90.0  # 无朝向信息时给中间值

            # 初步 Cost（用欧氏距离，用于排序候选）
            prelim_cost = dist * (
                1.0 + self.direction_penalty * angle_deg / 180.0)
            prelim_score = gain / (prelim_cost + self.cost_epsilon)

            candidates.append(
                (fx, fy, fsize, gain, angle_deg, prelim_score))

        if not candidates:
            return None

        # 按初步评分降序排列（优先验证高分候选）
        candidates.sort(key=lambda c: c[5], reverse=True)

        best_result = None
        best_score = -1.0

        for (fx, fy, fsize, gain, angle_deg, _) in candidates:
            reachable, path_length = self._is_goal_reachable(fx, fy)

            if not reachable:
                rospy.loginfo(
                    "[前沿探索] make_plan 预检失败，跳过 (%.2f,%.2f) "
                    "Gain=%.2f", fx, fy, gain)
                self.blacklist.append((fx, fy))
                continue

            # 根据配置选择 Cost 的距离来源
            if self.use_path_length and path_length > 0.0:
                distance = path_length
            else:
                distance = np.sqrt(
                    (fx - robot_x)**2 + (fy - robot_y)**2)

            # 最终 Cost = 距离 × (1 + λ × 方向角/180)
            cost = distance * (
                1.0 + self.direction_penalty * angle_deg / 180.0)
            score = gain / (cost + self.cost_epsilon)

            rospy.loginfo(
                "[前沿探索] 候选 (%.2f,%.2f): Gain=%.3f Cost=%.2fm "
                "方向=%.0f° Score=%.4f",
                fx, fy, gain, cost, angle_deg, score)

            if score > best_score:
                best_score = score
                best_result = (fx, fy, score)

        if best_result is not None:
            rospy.loginfo(
                "[前沿探索] make_plan 预检通过 (%.2f,%.2f) Score=%.4f",
                best_result[0], best_result[1], best_result[2])

        return best_result

    def _is_blacklisted(self, x, y):
        """检查 (x, y) 是否在黑名单中（之前导航失败的目标附近）"""
        for bx, by in self.blacklist:
            if (x - bx)**2 + (y - by)**2 < self.blacklist_radius**2:
                return True
        return False

    def _is_goal_reachable(self, x, y):
        """
        通过 /move_base/make_plan 服务预检查目标是否可达，并计算路径长度。

        校验条件：
          1. 路径至少 5 个位姿（排除仅有起点的退化情况）
          2. 路径终点与目标距离 < 1.0m（确认真到达目标附近，而非半路中断）

        返回：(True, path_length) 可达时，path_length 为路径总长（米）
              (False, 0.0) 不可达时
        """
        robot_x, robot_y, _ = self._get_robot_pose()
        if robot_x is None:
            return False, 0.0

        req = GetPlanRequest()
        req.start.header.frame_id = 'map'
        req.start.header.stamp = rospy.Time.now()
        req.start.pose.position.x = robot_x
        req.start.pose.position.y = robot_y
        req.start.pose.position.z = 0.0
        req.start.pose.orientation.w = 1.0

        req.goal.header.frame_id = 'map'
        req.goal.header.stamp = rospy.Time.now()
        req.goal.pose.position.x = x
        req.goal.pose.position.y = y
        req.goal.pose.position.z = 0.0
        req.goal.pose.orientation.w = 1.0

        req.tolerance = 0.5

        try:
            resp = self._plan_service.call(req)
            if resp.plan is None or len(resp.plan.poses) == 0:
                rospy.loginfo(
                    "[前沿探索] make_plan 返回空路径 (%.2f,%.2f)", x, y)
                return False, 0.0

            # 校验 1：路径至少 5 个点，排除退化情况
            if len(resp.plan.poses) < 5:
                rospy.loginfo(
                    "[前沿探索] make_plan 路径太短 (%d 点)，跳过 (%.2f,%.2f)",
                    len(resp.plan.poses), x, y)
                return False, 0.0

            # 校验 2：路径终点必须在目标附近（< 1.0m），否则是半路中断
            last_pose = resp.plan.poses[-1].pose.position
            end_dist = np.sqrt((last_pose.x - x)**2 + (last_pose.y - y)**2)
            if end_dist > 1.0:
                rospy.loginfo(
                    "[前沿探索] make_plan 路径未达目标 (距目标 %.2fm)，"
                    "跳过 (%.2f,%.2f)",
                    end_dist, x, y)
                return False, 0.0

            # 计算路径总长度（逐段累加）
            path_length = 0.0
            for i in range(1, len(resp.plan.poses)):
                p1 = resp.plan.poses[i - 1].pose.position
                p2 = resp.plan.poses[i].pose.position
                path_length += np.sqrt(
                    (p2.x - p1.x)**2 + (p2.y - p1.y)**2)

            return True, path_length
        except rospy.ServiceException as e:
            rospy.logwarn("[前沿探索] make_plan 服务调用异常: %s", e)
            return False, 0.0

    # ────────────── 导航控制 ──────────────
    def send_goal(self, x, y):
        """
        发送导航目标到 move_base，朝向设定为从机器人当前位置指向目标的方向。

        这样机器人到达时自然面朝行进方向，无需额外旋转调姿。
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # ── 朝向 = 机器人当前位置 → 目标方向 ──
        robot_x, robot_y, _ = self._get_robot_pose()
        if robot_x is not None and (x != robot_x or y != robot_y):
            yaw = np.arctan2(y - robot_y, x - robot_x)
        else:
            yaw = 0.0
        goal.target_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = np.cos(yaw / 2.0)

        rospy.loginfo("[前沿探索] 发送导航目标: (%.2f, %.2f) yaw=%.1f°",
                      x, y, np.degrees(yaw))
        self.ac.send_goal(goal)
        self.is_navigating = True
        self.goal_start_time = rospy.Time.now()
        self.current_goal_xy = (x, y)
        self.last_replan_time = rospy.Time.now()
        # 记录初始 Gain 和检查时间，用于过时检测
        self.goal_gain = self._compute_gain(x, y)
        self.last_stale_check_time = rospy.Time.now()
        rospy.loginfo("[前沿探索] 初始 Gain=%.3f", self.goal_gain)

    def cancel_goal(self):
        """取消当前导航目标"""
        self.ac.cancel_goal()
        self._force_stop()
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_xy = None
        self.last_replan_time = None
        self.goal_gain = None
        self.last_stale_check_time = None
        rospy.loginfo("[前沿探索] 已取消当前导航目标")

    def _navigate_to(self, x, y, timeout=60.0):
        """
        通过 move_base 定点导航到 (x,y)，等待到达或超时。

        返回: True=到达, False=超时/失败
        """
        self.send_goal(x, y)
        rospy.loginfo("[导航] 等待到达 (%.2f, %.2f) 最多 %.0fs ...", x, y, timeout)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("[导航] ✓ 已到达 (%.2f, %.2f)", x, y)
                self.is_navigating = False
                self.goal_start_time = None
                self.current_goal_xy = None
                return True
            elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                           GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.logwarn("[导航] ✗ 导航失败 (状态码: %d)", state)
                self.is_navigating = False
                self.goal_start_time = None
                self.current_goal_xy = None
                return False
            rate.sleep()
        rospy.logwarn("[导航] ⏰ 导航超时")
        self.cancel_goal()
        return False

    def _force_stop(self):
        """
        平滑停车：逐步减速并等待 move_base 进入终态。

        时序保证：
          1. cancel_goal() 先通知 move_base 停止控制（已在外部调用）
          2. 等待 100ms 让 cancel 传播，避免与 move_base 的命令冲突
          3. 逐步减速发布零速度
          4. 同时等待 move_base 进入终态（最多 3 秒）
        """
        rospy.sleep(0.1)  # 等 cancel 传播

        # 逐步减速（8 步 × 100ms = 0.8s）
        decel_steps = [0.15, 0.10, 0.06, 0.04, 0.02, 0.01, 0.005, 0.0]
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        step_idx = 0

        while rospy.Time.now() < deadline:
            # 发布当前减速步的速度
            if step_idx < len(decel_steps):
                cmd = Twist()
                cmd.linear.x = decel_steps[step_idx]
                self.cmd_vel_pub.publish(cmd)
                step_idx += 1

            # 检查 move_base 是否已进入终态
            state = self.ac.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.LOST):
                break
            rospy.sleep(0.1)

        # 最终零速度确保停稳
        self.cmd_vel_pub.publish(Twist())

    def _rotate_360(self, angular_vel=0.8):
        """
        到达目标后原地旋转 360° 扫描周围环境。

        move_base 导航成功时已自动停稳，直接发布旋转指令即可。
        SLAM Toolbox 在旋转过程中会持续更新地图。

        参数:
            angular_vel: 旋转角速度 (rad/s)，默认 0.5，约 12.6s 完成一圈
        """
        rotation_time = 2.0 * np.pi / abs(angular_vel)
        rospy.loginfo(
            "[前沿探索] 🔄 到达目标，旋转扫描 360° (%.1fs)", rotation_time)

        cmd = Twist()
        cmd.angular.z = angular_vel

        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now() - start).to_sec() < rotation_time:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 停止旋转
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.3)  # 等待停稳 + SLAM 最后一次更新
        rospy.loginfo("[前沿探索] ✓ 旋转扫描完成")

    def _wiggle_scan(self, angle=30.0, angular_vel=1.2):
        """
        左右快速摆头扫描，让 LiDAR 从多个角度观察前方墙壁。

        相比 360° 旋转（~8s），摆头只需约 1.5s，大幅加速。
        摆头后 SLAM 能更准确地注册前方障碍物。

        参数:
            angle:       左右摆角（度），默认 ±30°
            angular_vel: 角速度 (rad/s)，默认 1.2
        """
        half_rad = np.radians(angle)
        duration = half_rad / abs(angular_vel)

        rospy.loginfo("[前沿探索] ↔ 摆头扫描 ±%.0f° (%.1fs)...", angle, duration * 2)

        cmd = Twist()
        rate = rospy.Rate(30)

        # 先向左转
        cmd.angular.z = angular_vel
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 再向右转（两倍行程）
        cmd.angular.z = -angular_vel
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration * 2:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 回中
        cmd.angular.z = angular_vel
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 停稳
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.2)
        rospy.loginfo("[前沿探索] ✓ 摆头扫描完成")

    # ────────────── 直线前进 ──────────────
    def _move_forward(self, distance, speed=0.15):
        """
        机器人直线前进指定距离（开环控制，基于时间估算）。

        参数:
            distance: 前进距离（米）
            speed:    前进速度（m/s），默认 0.15
        """
        duration = distance / abs(speed)
        rospy.loginfo("[前沿探索] ➡ 前进 %.2fm (速度 %.2fm/s, 预计 %.1fs)",
                      distance, speed, duration)

        cmd = Twist()
        cmd.linear.x = speed

        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now() - start).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 停止并等待停稳
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.3)
        rospy.loginfo("[前沿探索] ✓ 前进完成")

    # ────────────── 走廊入口检测 ──────────────
    def _detect_corridor_entrance(self):
        """
        沿机器人朝向（+y）扫描地图，检测走廊入口位置。

        原理：
          - 建筑关于 x=0 对称，走廊中心在 x=0
          - 大厅宽度 ~20m，走廊宽度 ~1.6-2.2m
          - 沿 y 轴向前步进，在每个采样点测横向自由空间宽度
          - 宽度从 >10m 骤降到 <3m 的转折点即为走廊入口

        返回:
          (entrance_x, entrance_y) 世界坐标，或 (None, None)
        """
        if self.map_data is None:
            rospy.logwarn("[走廊检测] 地图未就绪")
            return None, None

        robot_x, robot_y, robot_yaw = self._get_robot_pose()
        if robot_x is None:
            rospy.logwarn("[走廊检测] 无法获取机器人位姿")
            return None, None

        res = self.map_resolution
        h, w = self.map_data.shape
        max_scan_y = robot_y + 20.0  # 最远向前扫描 20m

        rospy.loginfo("[走廊检测] 从机器人位置 (%.2f, %.2f) 开始向前扫描",
                      robot_x, robot_y)

        narrow_streak = 0  # 连续窄判定计数器
        entrance_y = None
        width_profile = []  # 记录宽度剖面用于日志

        scan_y = robot_y + 0.5  # 从机器人前方 0.5m 开始
        while scan_y <= max_scan_y:
            # 世界坐标 → 栅格坐标
            col = int((robot_x - self.map_origin_x) / res)
            row = int((scan_y - self.map_origin_y) / res)

            if row < 0 or row >= h:
                break

            # ── 向 -x 方向扫描直到碰墙 ──
            left_dist = 0
            for dc in range(1, int(15.0 / res)):  # 最多 15m
                lc = col - dc
                if lc < 0 or lc >= w:
                    break
                val = self.map_data[row, lc]
                if val == 100:   # 碰到障碍物（墙壁）
                    left_dist = dc * res
                    break
                if val == -1:    # 未知区域也停止
                    left_dist = dc * res
                    break
                if dc == int(15.0 / res) - 1:  # 到头也没碰墙
                    left_dist = dc * res

            # ── 向 +x 方向扫描直到碰墙 ──
            right_dist = 0
            for dc in range(1, int(15.0 / res)):
                rc = col + dc
                if rc < 0 or rc >= w:
                    break
                val = self.map_data[row, rc]
                if val == 100:
                    right_dist = dc * res
                    break
                if val == -1:
                    right_dist = dc * res
                    break
                if dc == int(15.0 / res) - 1:
                    right_dist = dc * res

            total_width = left_dist + right_dist
            width_profile.append((scan_y, total_width))

            rospy.loginfo("[走廊检测]   y=%.2f  宽度=%.2fm (L=%.2f R=%.2f)",
                          scan_y, total_width, left_dist, right_dist)

            # ── 判定：宽度低于阈值则累积窄计数 ──
            if total_width < self.corridor_width_thresh:
                narrow_streak += 1
                if narrow_streak >= self.corridor_narrow_count and entrance_y is None:
                    # 首次确认：回退到第一次变窄的位置
                    entrance_y = (
                        width_profile[-self.corridor_narrow_count][0]
                        if len(width_profile) >= self.corridor_narrow_count
                        else scan_y
                    )
                    rospy.loginfo(
                        "[走廊检测] ✓ 检测到走廊入口: (%.2f, %.2f) "
                        "连续 %d 点宽度 < %.1fm",
                        robot_x, entrance_y,
                        self.corridor_narrow_count,
                        self.corridor_width_thresh)
                    break
            else:
                narrow_streak = 0  # 宽度回升，重置计数器

            scan_y += self.corridor_scan_step

        if entrance_y is None:
            rospy.loginfo(
                "[走廊检测] 未检测到走廊入口（扫描到 y=%.2f），"
                "可能地图覆盖不足或场景为开阔区域", scan_y)
            return None, None

        # ── 发布走廊入口标记到 RViz ──
        self._publish_entrance_marker(robot_x, entrance_y)
        return robot_x, entrance_y

    def _publish_entrance_marker(self, x, y):
        """发布走廊入口的柱状标记到 RViz（黄色）"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'corridor_entrance'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=0.5)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 1.0
        m.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)
        m.lifetime = rospy.Duration(0)
        self.goal_marker_pub.publish(m)

    # ────────────── 走廊尽头检测 ──────────────
    def _detect_corridor_end(self, entrance_y):
        """
        从机器人当前位置沿走廊中心线 (x=0) 向 +y 扫描，检测走廊尽头。

        只做一个动作：沿 x=0 向前看，遇到什么？
          - 碰 100（真实墙壁）且前方自由 < 0.5m → 尽头确认
          - 碰 -1（未知区域）→ 地图未覆盖，需前进建图
          - 全是 0（自由）→ 走廊还很长，但当前地图已覆盖到头

        返回: (end_y, definitive)
        """
        if self.map_data is None or entrance_y is None:
            return None, False

        # 从机器人当前位置开始扫描（而非 entrance_y）
        rx, ry, _ = self._get_robot_pose()
        if ry is None:
            ry = entrance_y

        res = self.map_resolution
        h, w = self.map_data.shape
        col_c = int((0.0 - self.map_origin_x) / res)
        step = 0.25
        scan_y = ry + 0.3       # 从机器人前方开始
        max_scan = ry + 50.0    # 最多前扫 50m

        rospy.loginfo("[尽头检测] ====== 从机器人 y=%.2f 沿 x=0 向前扫描 ======", ry)

        while scan_y <= max_scan:
            row = int((scan_y - self.map_origin_y) / res)
            if row < 0 or row >= h:
                break

            # 先看当前格：如果是未知，地图没覆盖到这里
            cur = self.map_data[row, col_c]
            if cur == -1:
                rospy.loginfo("[尽头检测] y=%.2f 未覆盖 → 需前进建图", scan_y)
                return scan_y, False
            if cur == 100:
                rospy.loginfo("[尽头检测] y=%.2f 直接碰墙 → 尽头", scan_y)
                return scan_y, True

            # 向前看连续自由格数
            forward_cells = 0
            blocker = 0
            for dr in range(1, int(10.0 / res)):
                r_fwd = row + dr
                if r_fwd >= h:
                    break
                v = self.map_data[r_fwd, col_c]
                if v == 100:
                    blocker = 100
                    break
                if v == -1:
                    blocker = -1
                    break
                forward_cells += 1
            forward_dist = forward_cells * res

            if forward_dist < 0.5:
                if blocker == 100:
                    rospy.loginfo("[尽头检测] ★ 真实尽头: y=%.2f (前方 %.2fm 碰墙)",
                                  scan_y, forward_dist)
                    return scan_y, True
                elif blocker == -1:
                    rospy.loginfo("[尽头检测] ★ 前方未知: y=%.2f (前方 %.2fm 后为未知)",
                                  scan_y, forward_dist)
                    return scan_y, False

            # 前方还很开阔，但不用逐格全扫，跳一大步
            # 如果前方自由 > 2m，直接跳到自由段的末尾继续
            if forward_dist > 2.0:
                scan_y += forward_dist - 0.5
            else:
                scan_y += step

        rospy.loginfo("[尽头检测] 扫描 %dm 未发现尽头，视为未知",
                      max_scan - ry)
        return scan_y, False

    # ────────────── 走廊房门扫描 ──────────────
    def _scan_corridor_doors(self, entrance_y, corridor_end_y=None):
        """
        从走廊入口到尽头，沿 x=0 中线逐行扫描左右墙距，检测房门。

        核心改动：自由空间包含未知(-1)，仅真实墙壁(100)才视为边界。
        这样即使门内房间未被完整扫图，墙距也会骤增 → 正确识别为门。

        检测逻辑：
          - 走廊墙距基线 ~1-1.5m
          - 墙距跳到 >2.5m → 房门开始
          - 墙距回落到 <2.5m → 房门结束

        参数:
            entrance_y:      走廊入口 y 坐标
            corridor_end_y:  走廊尽头 y 坐标（可选，不传则默认 +30m）
        返回: [(door_y, side), ...]  side='left' 或 'right'
        """
        if self.map_data is None or entrance_y is None:
            return []

        res = self.map_resolution
        h, w = self.map_data.shape
        max_scan_dist = 12.0  # 单侧最大扫描距离

        max_y = corridor_end_y if corridor_end_y else entrance_y + 30.0
        scan_y = entrance_y + 0.5
        step = 0.2
        door_thresh = 2.5

        doors = []
        prev_left = None
        prev_right = None
        in_left_door = False
        in_right_door = False
        left_door_start = None
        right_door_start = None

        rospy.loginfo("[房门扫描] ====== y=%.2f → y=%.2f ======", scan_y, max_y)

        while scan_y <= max_y:
            col = int((0.0 - self.map_origin_x) / res)
            row = int((scan_y - self.map_origin_y) / res)
            if row < 0 or row >= h:
                break

            # ── 测左墙距：仅真实墙壁(100)停止，自由(0)和未知(-1)都继续 ──
            left = 0.0
            for dc in range(1, int(max_scan_dist / res)):
                lc = col - dc
                if lc < 0 or lc >= w:
                    break
                val = self.map_data[row, lc]
                if val == 100:          # 只碰真墙才停
                    left = dc * res
                    break
                if dc == int(max_scan_dist / res) - 1:
                    left = dc * res     # 到头也没碰墙 → 大开口

            # ── 测右墙距 ──
            right = 0.0
            for dc in range(1, int(max_scan_dist / res)):
                rc = col + dc
                if rc < 0 or rc >= w:
                    break
                val = self.map_data[row, rc]
                if val == 100:
                    right = dc * res
                    break
                if dc == int(max_scan_dist / res) - 1:
                    right = dc * res

            rospy.loginfo("[房门扫描] y=%.2f  L=%.2fm  R=%.2fm",
                          scan_y, left, right)

            # ── 左门检测 ──
            if left > door_thresh and not in_left_door:
                in_left_door = True
                left_door_start = scan_y
                rospy.loginfo("[房门扫描]   左侧门开始 @ y=%.2f", scan_y)
            elif left <= door_thresh and in_left_door:
                in_left_door = False
                door_y = (left_door_start + scan_y) / 2.0
                doors.append((door_y, 'left'))
                rospy.loginfo("[房门扫描] ★ 左侧门: y=%.2f (宽 %.2fm)",
                              door_y, scan_y - left_door_start)

            # ── 右门检测 ──
            if right > door_thresh and not in_right_door:
                in_right_door = True
                right_door_start = scan_y
                rospy.loginfo("[房门扫描]   右侧门开始 @ y=%.2f", scan_y)
            elif right <= door_thresh and in_right_door:
                in_right_door = False
                door_y = (right_door_start + scan_y) / 2.0
                doors.append((door_y, 'right'))
                rospy.loginfo("[房门扫描] ★ 右侧门: y=%.2f (宽 %.2fm)",
                              door_y, scan_y - right_door_start)

            prev_left = left
            prev_right = right
            scan_y += step

        # 扫描结束还在门内 → 记录
        if in_left_door and left_door_start is not None:
            doors.append(((left_door_start + scan_y) / 2.0, 'left'))
            rospy.loginfo("[房门扫描] ★ 左侧门(未闭合): y=%.2f",
                          (left_door_start + scan_y) / 2.0)
        if in_right_door and right_door_start is not None:
            doors.append(((right_door_start + scan_y) / 2.0, 'right'))
            rospy.loginfo("[房门扫描] ★ 右侧门(未闭合): y=%.2f",
                          (right_door_start + scan_y) / 2.0)

        rospy.loginfo("[房门扫描] ====== 共检测到 %d 扇门 ======", len(doors))
        return doors

    def _publish_door_markers(self, doors):
        """发布房门标记到 RViz（左侧=蓝色，右侧=绿色）"""
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'doors'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, (door_y, side) in enumerate(doors):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'doors'
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = Point(x=(1.2 if side == 'right' else -1.2), y=door_y, z=0.4)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.8
            if side == 'left':
                m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8)
            else:
                m.color = ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.8)
            m.lifetime = rospy.Duration(0)
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)
        rospy.loginfo("[房门扫描] 📍 已发布 %d 个房门标记", len(doors))

    # ────────────── 高度剖面构建（内部工具） ──────────────
    def _build_height_profile(self, y_start, dy_sign, max_dist_m):
        """
        沿 x 轴扫描，在每列从 y_start 沿 dy_sign 方向累计高度直到碰墙(100)。

        返回: [(x, height_m), ...]
        """
        res = self.map_resolution
        h, w = self.map_data.shape
        y_row = int((y_start - self.map_origin_y) / res)
        profile = []

        x = -10.0
        while x <= 10.0:
            col = int((x - self.map_origin_x) / res)
            if col < 0 or col >= w:
                x += 0.2
                continue
            if y_row < 0 or y_row >= h:
                x += 0.2
                continue
            if self.map_data[y_row, col] == -1:  # 起始点未知，跳过
                x += 0.2
                continue

            cnt = 0
            for d in range(int(max_dist_m / res)):
                r = y_row + dy_sign * d
                if r < 0 or r >= h:
                    break
                if self.map_data[r, col] == 100:
                    break
                cnt += 1
            profile.append((x, cnt * res))
            x += 0.2

        return profile

    # ────────────── 楼梯/电梯检测 ──────────────
    def _detect_stair_and_elevator(self, entrance_y):
        """
        综合分析向上和向下两个自由高度剖面，定位楼梯和电梯中心。

        策略：
          - 楼梯在 x<0：找"低平台"两侧的跳变 → 左墙和右墙
          - 电梯在 x>0：同上
          - 两次扫描互相验证，取平均 x
          - y = entrance_y * 比例（楼梯 ~0.5，电梯 ~0.35）
        """
        # ── 获取两个剖面 ──
        prof_up = self._build_height_profile(0.5, 1, 15.0)
        prof_down = self._build_height_profile(entrance_y - 0.3, -1, entrance_y - 0.5)

        def _find_jumps(profile, x_sign, jump_thresh=3.0):
            """在 profile 中找 x_sign 侧的大跳变。返回 [(x_before, x_after, jump_m, direction), ...]"""
            jumps = []
            for i in range(1, len(profile)):
                px, ph = profile[i - 1]
                cx, ch = profile[i]
                if x_sign == -1 and cx > 0:
                    break
                if x_sign == 1 and px < 0:
                    continue
                delta = ch - ph
                if abs(delta) > jump_thresh:
                    jumps.append((px, cx, delta, 'up' if delta > 0 else 'down'))
            return jumps

        def _get_wall_pair(jumps):
            """从跳变列表中找墙壁对，用平台宽度（1.5~6m）过滤噪声。"""
            if not jumps:
                return None, None
            best_left = None
            best_right = None
            wall_left = None
            for jx_before, jx_after, delta, direction in jumps:
                if direction == 'down':
                    wall_left = (jx_before + jx_after) / 2.0
                elif direction == 'up' and wall_left is not None:
                    wall_right = (jx_before + jx_after) / 2.0
                    platform_width = wall_right - wall_left
                    if 1.5 < platform_width < 6.0:
                        best_left = wall_left
                        best_right = wall_right
            return best_left, best_right

        # ── 分析向上扫 ──
        jumps_up_left = _find_jumps(prof_up, -1)
        jumps_up_right = _find_jumps(prof_up, 1)

        stair_L_up, stair_R_up = _get_wall_pair(jumps_up_left)
        elev_L_up, elev_R_up = _get_wall_pair(jumps_up_right)

        rospy.loginfo("[楼梯检测] 向上扫: stair_L=%.2f stair_R=%.2f elev_L=%.2f elev_R=%.2f",
                      stair_L_up or -1, stair_R_up or -1, elev_L_up or -1, elev_R_up or -1)

        # ── 分析向下扫 ──
        jumps_down_left = _find_jumps(prof_down, -1)
        jumps_down_right = _find_jumps(prof_down, 1)

        stair_L_down, stair_R_down = _get_wall_pair(jumps_down_left)
        elev_L_down, elev_R_down = _get_wall_pair(jumps_down_right)

        rospy.loginfo("[楼梯检测] 向下扫: stair_L=%.2f stair_R=%.2f elev_L=%.2f elev_R=%.2f",
                      stair_L_down or -1, stair_R_down or -1,
                      elev_L_down or -1, elev_R_down or -1)

        # ── 融合：取两个剖面中有效的墙，求平均 ──
        def _avg(*vals):
            v = [v for v in vals if v is not None]
            return sum(v) / len(v) if v else None

        stair_L = _avg(stair_L_up, stair_L_down)
        stair_R = _avg(stair_R_up, stair_R_down)
        elev_L = _avg(elev_L_up, elev_L_down)
        elev_R = _avg(elev_R_up, elev_R_down)

        # ── 计算中心 ──
        if stair_L is not None and stair_R is not None:
            sx = (stair_L + stair_R) / 2.0
            sy = entrance_y * 0.5
            rospy.loginfo("[楼梯检测] ★★★ 楼梯中心: (%.2f, %.2f) ★★★", sx, sy)
            self._publish_block_marker(sx, sy, 'stairs',
                                       ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.7))
        else:
            rospy.loginfo("[楼梯检测] 未检测到完整楼梯墙壁")

        if elev_L is not None and elev_R is not None:
            ex = (elev_L + elev_R) / 2.0
            ey = entrance_y * 0.35
            rospy.loginfo("[楼梯检测] ★★★ 电梯中心: (%.2f, %.2f) ★★★", ex, ey)
            self._publish_block_marker(ex, ey, 'elevator',
                                       ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.7))
        else:
            rospy.loginfo("[楼梯检测] 未检测到完整电梯墙壁")

        # ── 提取走廊 x 边界，划定走廊两侧大厅为禁区 ──
        corridor_L, corridor_R = self._find_corridor_x_bounds(prof_up)
        if corridor_L is not None and corridor_R is not None:
            self.forbidden_zones.append(
                (-20.0, corridor_L, 0.5, entrance_y))
            self.forbidden_zones.append(
                (corridor_R, 20.0, 0.5, entrance_y))
            rospy.loginfo("[禁区] 走廊 x[%.2f,%.2f] 两侧均标记为禁区",
                          corridor_L, corridor_R)

    def _find_corridor_x_bounds(self, profile):
        """从向上剖面中找走廊的 x 范围（高度 >12m 的最宽连续区域）。"""
        segments = []
        in_seg = False
        seg_start = None
        for x, h in profile:
            if h > 12.0:
                if not in_seg:
                    seg_start = x
                    in_seg = True
            else:
                if in_seg:
                    segments.append((seg_start, x))
                    in_seg = False
        if in_seg:
            segments.append((seg_start, profile[-1][0]))

        if not segments:
            rospy.logwarn("[禁区] 未找到走廊区域（无高度>12m的连续段）")
            return None, None

        best = max(segments, key=lambda s: s[1] - s[0])
        rospy.loginfo("[禁区] 走廊 x 边界: [%.2f, %.2f]", best[0], best[1])
        return best[0], best[1]

    def _mark_forbidden_zones(self):
        """将禁区矩形在地图上标记为障碍物(100)。"""
        if not self.forbidden_zones or self.map_data is None:
            return
        res = self.map_resolution
        h, w = self.map_data.shape
        for (x_min, x_max, y_min, y_max) in self.forbidden_zones:
            c_min = max(0, min(w - 1, int((x_min - self.map_origin_x) / res)))
            c_max = max(0, min(w - 1, int((x_max - self.map_origin_x) / res)))
            r_min = max(0, min(h - 1, int((y_min - self.map_origin_y) / res)))
            r_max = max(0, min(h - 1, int((y_max - self.map_origin_y) / res)))
            if c_min >= c_max or r_min >= r_max:
                continue
            self.map_data[r_min:r_max + 1, c_min:c_max + 1] = 100
            rospy.logdebug("[禁区] ✓ 标记: x[%.2f,%.2f] y[%.2f,%.2f] → %d×%d",
                           x_min, x_max, y_min, y_max,
                           r_max - r_min + 1, c_max - c_min + 1)

    def _publish_forbidden_zone_markers(self):
        """发布禁区矩形可视化标记到 RViz（红色半透明）。"""
        if not self.forbidden_zones:
            return
        marker_array = MarkerArray()
        # 清除旧标记
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'forbidden_zones'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, (x_min, x_max, y_min, y_max) in enumerate(self.forbidden_zones):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'forbidden_zones'
            m.id = i + 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            # 中心位置
            m.pose.position = Point(
                x=(x_min + x_max) / 2.0,
                y=(y_min + y_max) / 2.0,
                z=0.3)
            m.pose.orientation.w = 1.0
            # 缩放 = 矩形尺寸
            m.scale.x = abs(x_max - x_min)
            m.scale.y = abs(y_max - y_min)
            m.scale.z = 0.1
            # 红色半透明
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.4)
            m.lifetime = rospy.Duration(0)
            marker_array.markers.append(m)

        self.forbidden_pub.publish(marker_array)
        # rospy.loginfo("[禁区] 📍 已发布 %d 个禁区可视化标记", len(self.forbidden_zones))

    def _publish_block_marker(self, x, y, ns, color):
        """发布彩色方块标记到 RViz，覆盖检测到的位置"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=0.5)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.8
        m.scale.y = 0.8
        m.scale.z = 1.0
        m.color = color
        m.lifetime = rospy.Duration(0)
        self.goal_marker_pub.publish(m)

    # ────────────── 自由高度扫描 (测试用) ──────────────
    def _scan_free_height(self, entrance_y=None):
        """
        扫描 x 轴自由高度剖面。调用两次：
          - 第一次：从 y=0.5 向上扫 → 拿到面朝大厅的墙壁（楼梯右墙、电梯左墙）
          - 第二次：从入口 y 向下扫 → 拿到背面的墙壁（楼梯左墙、电梯右墙）
        """
        if self.map_data is None:
            return

        res = self.map_resolution
        h, w = self.map_data.shape

        # 调试：打印地图边界信息
        map_y_min = self.map_origin_y
        map_y_max = self.map_origin_y + h * res
        map_x_min = self.map_origin_x
        map_x_max = self.map_origin_x + w * res
        rospy.loginfo(
            "[自由高度] 地图范围: x[%.2f, %.2f] y[%.2f, %.2f] "
            "size=%dx%d res=%.3f",
            map_x_min, map_x_max, map_y_min, map_y_max, w, h, res)

        # 世界 y=0.5 对应的栅格行（避开门口可能的噪声）
        y_start = 0.5
        y_start_row = int((y_start - self.map_origin_y) / res)
        rospy.loginfo("[自由高度] 世界 y=%.2f 对应栅格行号: %d / %d",
                      y_start, y_start_row, h)

        # 扫描范围：x 从 -10m 到 +10m，步进 0.2m
        x_start = -10.0
        x_end = 10.0
        x_step = 0.2
        max_scan_y = 15.0  # 最远向上扫 15m

        rospy.loginfo(
            "[自由高度] ====== x 轴自由高度剖面 (从 y=%.2f 向上) ======",
            y_start)

        profile = []
        x = x_start
        while x <= x_end:
            col = int((x - self.map_origin_x) / res)
            if col < 0 or col >= w:
                x += x_step
                continue

            # 起始点检查：如果是未知空间，该列跳过（LiDAR 未覆盖）
            if y_start_row < 0 or y_start_row >= h:
                x += x_step
                continue
            start_val = self.map_data[y_start_row, col]
            if start_val == -1:  # 未知，跳过
                x += x_step
                continue

            # 从 y_start 向上扫描：障碍物(100)停止，自由(0)和未知(-1)均计数
            free_count = 0
            for dy in range(int(max_scan_y / res)):
                row = y_start_row + dy
                if row < 0 or row >= h:
                    break
                val = self.map_data[row, col]
                if val == 100:   # 障碍物 → 停止
                    break
                free_count += 1  # 自由(0) 或 未知(-1) 都算高度

            free_height = free_count * res
            profile.append((x, free_height))

            # 检测附近的跳变（与左边相邻比较）
            if len(profile) >= 2:
                prev_h = profile[-2][1]
                curr_h = free_height
                jump = curr_h - prev_h
                if abs(jump) > 1.5:  # 跳变 > 1.5m 标记
                    direction = "↗ 变高" if jump > 0 else "↘ 变低"
                    rospy.loginfo(
                        "[自由高度] x=%.2f 高度=%.2fm  ← %s (Δ=%.2fm) ★",
                        x, free_height, direction, jump)
                else:
                    rospy.loginfo(
                        "[自由高度] x=%.2f 高度=%.2fm", x, free_height)
            else:
                rospy.loginfo(
                    "[自由高度] x=%.2f 高度=%.2fm", x, free_height)

            x += x_step

        # ── 汇总 ──
        rospy.loginfo("[自由高度] ====== 跳变点汇总 (向上扫) ======")
        for i in range(1, len(profile)):
            prev_x, prev_h = profile[i - 1]
            curr_x, curr_h = profile[i]
            jump = curr_h - prev_h
            if abs(jump) > 1.5:
                direction = "↑ 变高" if jump > 0 else "↓ 变低"
                rospy.loginfo(
                    "[自由高度] ★ x=%.2f → x=%.2f  %s: "
                    "%.2fm → %.2fm (Δ=%.2fm)",
                    prev_x, curr_x, direction, prev_h, curr_h, jump)

        # ═══════════════════════════════════════════════════
        #  向下扫描：从走廊入口向下看楼梯/电梯背面
        # ═══════════════════════════════════════════════════
        if entrance_y is None:
            return

        y_down = entrance_y - 0.3              # 入口稍微下一点开始
        y_down_row = int((y_down - self.map_origin_y) / res)
        min_scan_y = 0.5                       # 最远向下扫到 y=0.5

        if y_down_row < 0 or y_down_row >= h:
            rospy.logwarn("[自由高度] 入口 y=%.2f 超出地图范围，跳过向下扫", y_down)
            return

        rospy.loginfo(
            "[自由高度] ====== x 轴自由高度剖面 (从 y=%.2f 向下) ======",
            y_down)

        profile = []
        x = x_start
        while x <= x_end:
            col = int((x - self.map_origin_x) / res)
            if col < 0 or col >= w:
                x += x_step
                continue

            # 起始点未知则跳过
            if self.map_data[y_down_row, col] == -1:
                x += x_step
                continue

            free_count = 0
            for dy in range(int((y_down - min_scan_y) / res)):
                row = y_down_row - dy  # 向下
                if row < 0 or row >= h:
                    break
                val = self.map_data[row, col]
                if val == 100:
                    break
                free_count += 1

            free_height = free_count * res
            profile.append((x, free_height))

            if len(profile) >= 2:
                prev_h = profile[-2][1]
                curr_h = free_height
                jump = curr_h - prev_h
                if abs(jump) > 1.5:
                    direction = "↗ 变深" if jump > 0 else "↘ 变浅"
                    rospy.loginfo(
                        "[自由高度↓] x=%.2f 深度=%.2fm  ← %s (Δ=%.2fm) ★",
                        x, free_height, direction, jump)
                else:
                    rospy.loginfo(
                        "[自由高度↓] x=%.2f 深度=%.2fm", x, free_height)
            else:
                rospy.loginfo(
                    "[自由高度↓] x=%.2f 深度=%.2fm", x, free_height)

            x += x_step

        # ── 汇总 ──
        rospy.loginfo("[自由高度↓] ====== 跳变点汇总 (向下扫) ======")
        for i in range(1, len(profile)):
            prev_x, prev_h = profile[i - 1]
            curr_x, curr_h = profile[i]
            jump = curr_h - prev_h
            if abs(jump) > 1.5:
                direction = "↑ 变深" if jump > 0 else "↓ 变浅"
                rospy.loginfo(
                    "[自由高度↓] ★ x=%.2f → x=%.2f  %s: "
                    "%.2fm → %.2fm (Δ=%.2fm)",
                    prev_x, curr_x, direction, prev_h, curr_h, jump)

    # ────────────── 启动前初始化扫描 ──────────────
    def _initial_scan(self):
        """
        启动前沿探索前的初始化扫描流程：
          1. 原地旋转 360°（出生点周围）
          2. 前进一段距离（推开视野）
          3. 原地旋转 360°（扫描深处）
          4. 检测走廊入口，前进到入口前 0.5m
          5. 再次旋转 360°（近距离扫描走廊入口区域）

        各步骤的行为由 ROS 参数控制，可在运行时或 launch 文件中调节。
        """
        rospy.loginfo("[前沿探索] 等待首帧地图...")
        while not rospy.is_shutdown() and self.map_data is None:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return

        # ── 第一阶段：出生点旋转扫描 ──
        if self.init_rotate_enabled:
            rospy.loginfo("[前沿探索] 🔄 第一阶段：出生点旋转扫描...")
            self._rotate_360()

        # ── 第二阶段：导航前进 ──
        if self.init_forward_distance > 0:
            rx, ry, _ = self._get_robot_pose()
            if rx is not None:
                target_y = ry + self.init_forward_distance
                rospy.loginfo(
                    "[前沿探索] 🔄 第二阶段：导航前进 %.2fm 到 "
                    "(%.2f, %.2f)...",
                    self.init_forward_distance, rx, target_y)
                self._navigate_to(rx, target_y, timeout=30.0)

        # ── 第三阶段：深入区域旋转扫描 （暂不开启）──
        # if self.init_rotate_enabled:
        #     rospy.loginfo("[前沿探索] 🔄 第三阶段：深入后旋转扫描...")
        #     self._rotate_360()

        # ── 第四阶段：检测走廊入口 → 定点导航到入口前 → 再扫描 ──
        rospy.loginfo("[前沿探索] 🔍 检测走廊入口...")
        entrance_x, entrance_y = self._detect_corridor_entrance()
        if entrance_y is not None:
            rospy.loginfo(
                "[走廊检测] ★ 走廊入口坐标: (%.2f, %.2f)",
                entrance_x, entrance_y)
            target_y = entrance_y - 0.3
            rx, ry, _ = self._get_robot_pose()
            if rx is not None:
                dist = target_y - ry
                if dist > 0.3:
                    rospy.loginfo(
                        "[前沿探索] 🔄 第四阶段：move_base 导航到 "
                        "(%.2f, %.2f)...", entrance_x, target_y)
                    if self._navigate_to(entrance_x, target_y, timeout=40.0):
                        if self.init_rotate_enabled:
                            rospy.loginfo(
                                "[前沿探索] 🔄 第四阶段：入口前旋转扫描...")
                            # self._rotate_360()
                        # 重新检测（更精确）
                        old_entrance_x = entrance_x
                        old_entrance_y = entrance_y
                        entrance_x, entrance_y = (
                            self._detect_corridor_entrance())
                        if entrance_y is not None:
                            rospy.loginfo(
                                "[走廊检测] ★ 走廊入口(精): (%.2f, %.2f)",
                                entrance_x, entrance_y)
                            # 比较精确定位与初定位的偏差
                            delta = np.sqrt(
                                (entrance_x - old_entrance_x) ** 2
                                + (entrance_y - old_entrance_y) ** 2)
                            if delta > 0.2:
                                rospy.loginfo(
                                    "[走廊检测] 精确定位偏差 %.2fm > 0.2m，"
                                    "重新导航到精确入口前...", delta)
                                precise_target_y = entrance_y - 0.2
                                if self._navigate_to(
                                        entrance_x, precise_target_y,
                                        timeout=40.0):
                                    if self.init_rotate_enabled:
                                        rospy.loginfo(
                                            "[前沿探索] 🔄 精确位置摆头扫描...")
                                        self._wiggle_scan()
                                else:
                                    rospy.logwarn(
                                        "[前沿探索] 导航到精确入口前失败，"
                                        "继续在当前位姿执行后续检测")
                            else:
                                rospy.loginfo(
                                    "[走廊检测] 精确定位偏差 %.2fm ≤ 0.2m，"
                                    "无需重新导航", delta)
                    else:
                        rospy.logwarn(
                            "[前沿探索] 导航到入口前失败，"
                            "继续在当前位姿执行后续检测")
                else:
                    rospy.loginfo(
                        "[前沿探索] 已在走廊入口附近 (距 %.2fm)，跳过导航",
                        dist)
        else:
            rospy.loginfo("[走廊检测] 未检测到走廊入口，跳过第四阶段")

        # ── 楼梯/电梯检测 ──
        if entrance_y is not None:
            rospy.loginfo("[前沿探索] 🔍 检测楼梯和电梯...")
            self._detect_stair_and_elevator(entrance_y)

            # ── 标记禁区：走廊两侧大厅全标障碍物 ──
            rospy.loginfo("[前沿探索] 🚫 标记禁区...")
            self._mark_forbidden_zones()
            self._publish_forbidden_zone_markers()

            # ── 走廊深入循环：走一段 → 检测尽头 → 未到头则继续走 ──
            # 必须亲身走过去，SLAM 才能把 -1(未知) 更新为真实的 0/100
            corridor_end_y = None
            corridor_end_definitive = False
            if self.corridor_entry_depth > 0:
                max_steps = 10
                for step_i in range(max_steps):
                    # 检测尽头
                    end_y, definitive = self._detect_corridor_end(
                        entrance_y)
                    if definitive:
                        corridor_end_y = end_y
                        corridor_end_definitive = True
                        rospy.loginfo(
                            "[尽头检测] ★★★ 走廊尽头确认: y=%.2f ★★★",
                            corridor_end_y)
                        self._publish_entrance_marker(0.0, corridor_end_y)
                        break
                    elif end_y is not None:
                        corridor_end_y = end_y
                        rospy.loginfo(
                            "[尽头检测] 前方未知，当前覆盖到 y=%.2f，"
                            "继续前进建图 (step %d/%d)",
                            end_y, step_i + 1, max_steps)
                    else:
                        rospy.logwarn("[尽头检测] 无法检测，停止走廊深入")
                        break

                    # 计算下一个导航目标（x=0 固定）
                    next_y = entrance_y + self.corridor_entry_depth * (
                        step_i + 1)
                    rospy.loginfo(
                        "[前沿探索] 🔄 走廊深入 step%d: "
                        "导航到 (0.00, %.2f)...",
                        step_i + 1, next_y)
                    if not self._navigate_to(0.0, next_y, timeout=60.0):
                        rospy.logwarn(
                            "[前沿探索] 走廊深入导航失败，停止推进")
                        break
                    # 摆头扫描：让 LiDAR 多角度观察前方，SLAM 才能
                    # 把墙从未知(-1)正确更新为障碍物(100)
                    self._wiggle_scan()
                else:
                    rospy.loginfo(
                        "[前沿探索] 走廊深入达到最大步数 %d，停止推进",
                        max_steps)

            # ── 回到走廊 3/4 处：靠近房门群，LiDAR 能更好覆盖门内 ──
            if corridor_end_definitive and corridor_end_y is not None:
                observe_y = entrance_y + (corridor_end_y - entrance_y) * 0.75
                rospy.loginfo(
                    "[前沿探索] 🔄 回走到走廊 3/4 处 (0.00, %.2f) 观察房门...",
                    observe_y)
                if self._navigate_to(0.0, observe_y, timeout=60.0):
                    self._wiggle_scan()
                else:
                    rospy.logwarn("[前沿探索] 回走导航失败，在当前位置扫门")

            # ── 走廊房门扫描 ──
            if corridor_end_y is not None:
                rospy.loginfo(
                    "[前沿探索] 🚪 扫描走廊房门 "
                    "(入口=%.2f → 尽头=%.2f)...",
                    entrance_y, corridor_end_y)
            else:
                rospy.loginfo("[前沿探索] 🚪 扫描走廊房门...")
            doors = self._scan_corridor_doors(entrance_y, corridor_end_y)
            if doors:
                doors.sort(key=lambda d: d[0], reverse=True)
                self.door_positions = doors
                self._publish_door_markers(doors)
            else:
                rospy.loginfo("[房门扫描] 未检测到房门")

        # ── 调试暂停：完成初始扫描后原地待命，不进入探索循环 ──
        if self.init_pause_enabled:
            rospy.loginfo("[前沿探索] ⏸ 调试暂停已启用，机器人原地待命。"
                          "按 Ctrl+C 退出或 kill 该节点。")
            while not rospy.is_shutdown():
                self.cmd_vel_pub.publish(Twist())
                rospy.sleep(0.5)
            return  # 节点被关闭，直接返回

        rospy.loginfo("[前沿探索] ✓ 初始扫描完成，开始前沿探索")

    # ────────────── 周期性重规划（建图期）──────────────
    def _maybe_replan(self):
        """
        在导航过程中周期性取消并重发同一目标，强制 move_base
        基于最新 global_costmap 重新调用全局规划器。

        解决：建图探索期 SLAM 地图持续扩展，但 move_base 仅在路径失效
        时才重规划，导致机器人死守旧路径、无视新发现捷径的问题。

        触发条件：
          1. replan_interval > 0（功能开启）
          2. 距上次重规划已超过 replan_interval 秒
          3. 机器人距目标仍大于 replan_min_dist（避免接近目标时振荡）
          4. /map 话题有数据（SLAM 正在工作）
        """
        if self.replan_interval <= 0.0:
            return
        if self.current_goal_xy is None or self.last_replan_time is None:
            return
        if self._last_map_stamp is None:
            return

        now = rospy.Time.now()
        elapsed = (now - self.last_replan_time).to_sec()
        if elapsed < self.replan_interval:
            return

        gx, gy = self.current_goal_xy
        robot_x, robot_y, _ = self._get_robot_pose()
        if robot_x is None:
            return

        dist = np.sqrt((gx - robot_x) ** 2 + (gy - robot_y) ** 2)
        if dist < self.replan_min_dist:
            return

        rospy.loginfo(
            "[前沿探索] ♻ 触发周期性重规划: 距目标 %.2fm, 已导航 %.1fs",
            dist, (now - self.goal_start_time).to_sec()
            if self.goal_start_time else 0.0)

        # 取消当前导航
        self.ac.cancel_goal()
        # 给 move_base 短暂时间处理 cancel，避免新旧 goal 状态冲突
        rospy.sleep(0.1)

        # 重发同一目标；send_goal 会重置 last_replan_time
        self.send_goal(gx, gy)
        # 重置导航开始时间：超时按"当前段"计算，避免长途导航被误杀
        self.goal_start_time = rospy.Time.now()

    # ────────────── 目标过时检测 ──────────────
    def _check_stale_goal(self):
        """
        独立于重规划周期，检查当前导航目标是否过时。

        判定逻辑：
          1. 距上次检查已超过 stale_check_interval 秒
          2. 机器人距目标小于 stale_check_distance（进入传感器有效范围）
          3. 当前 Gain < 出发时 Gain × gain_decay_threshold

        满足以上条件时取消当前导航，回到空闲状态重新选目标。
        返回 True 表示目标已过时（调用方应跳过后续状态检查）。
        """
        if self.current_goal_xy is None:
            return False
        if self.goal_gain is None or self.goal_gain <= 0.0:
            return False
        if self.last_stale_check_time is None:
            return False

        now = rospy.Time.now()
        if (now - self.last_stale_check_time).to_sec() < self.stale_check_interval:
            return False

        self.last_stale_check_time = now

        gx, gy = self.current_goal_xy
        robot_x, robot_y, _ = self._get_robot_pose()
        if robot_x is None:
            return False

        dist = np.sqrt((gx - robot_x) ** 2 + (gy - robot_y) ** 2)
        if dist >= self.stale_check_distance:
            return False

        current_gain = self._compute_gain(gx, gy)
        threshold = self.goal_gain * self.gain_decay_threshold
        if current_gain >= threshold:
            return False

        rospy.loginfo(
            "[前沿探索] ⚠ 目标过时: Gain %.3f → %.3f "
            "(阈值 %.3f, 距目标 %.2fm), 取消并重新选择",
            self.goal_gain, current_gain, threshold, dist)
        self.cancel_goal()
        self._clear_goal_marker()
        return True

    # ────────────── 可视化 ──────────────
    def publish_frontier_markers(self, frontiers, selected_goal=None):
        """
        发布前沿簇标记到 RViz
        
        每个前沿簇显示为一个绿色球体，大小与簇的像素数成正比
        选中的目标显示为红色球体
        """
        marker_array = MarkerArray()

        # 第 0 号标记：清除所有旧标记
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'frontiers'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # 为每个前沿簇创建球体标记
        for i, (fx, fy, fsize) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position = Point(x=fx, y=fy, z=0.15)
            m.pose.orientation.w = 1.0

            # 球体直径与簇大小成正比（0.15~0.5m）
            scale = max(0.15, min(0.5, fsize / 80.0))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            # 选中目标：红色，其他：绿色
            if (selected_goal and
                    abs(fx - selected_goal[0]) < 0.1 and
                    abs(fy - selected_goal[1]) < 0.1):
                m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
            else:
                m.color = ColorRGBA(r=0.0, g=0.9, b=0.2, a=0.6)

            # 标记存活 3 秒后自动消失（下次更新会重新发布）
            m.lifetime = rospy.Duration(3.0)
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def publish_goal_marker(self, x, y):
        """发布当前探索目标的柱状标记（橙色）"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'current_goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=0.25)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.5
        m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
        # lifetime=0 表示永久显示（直到被新标记替换）
        m.lifetime = rospy.Duration(0)
        self.goal_marker_pub.publish(m)

    def _clear_goal_marker(self):
        """清除当前目标标记"""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'current_goal'
        m.id = 0
        m.action = Marker.DELETE
        self.goal_marker_pub.publish(m)

    # ────────────── 主循环 ──────────────
    def run(self):
        """
        探索主循环
        
        状态机：
          IDLE       → 检测前沿 → 有前沿 → 选目标 → 发送导航
          NAVIGATING → 等待结果 → 成功 → IDLE
                                 → 失败 → 加入黑名单 → IDLE
                                 → 超时 → 取消 → 加入黑名单 → IDLE
          COMPLETE   → 连续 N 次无前沿 → 退出
        """
        rate = rospy.Rate(self.detection_freq)
        rospy.loginfo("[前沿探索] ====== 开始自主探索（%.1f Hz）======",
                      self.detection_freq)

        # ── 启动前初始化扫描（前进 + 旋转），建立初始地图 ──
        self._initial_scan()

        while not rospy.is_shutdown():
            # ── 探索完成 ──
            if self.exploration_complete:
                self._clear_goal_marker()
                rospy.loginfo("[前沿探索] ====== 探索完成，所有区域已覆盖 ======")
                return

            # ── 等待地图 ──
            if self.map_data is None:
                rate.sleep()
                continue

            # ── 正在导航中：检查状态 ──
            if self.is_navigating:
                state = self.ac.get_state()

                # ── 周期性重规划 + 目标过时检测 ──
                # 仅在导航仍活跃（PENDING/ACTIVE）时触发，避免与成功/失败分支冲突
                # if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                #     self._maybe_replan()
                #     # 独立计时器的过时检查（比重规划更频繁）
                #     if self._check_stale_goal():
                #         rate.sleep()
                #         continue

                # 导航成功
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("[前沿探索] ✓ 导航成功")
                    # 到达目标后旋转 360° 扫描环境，SLAM 会更新地图
                    self._rotate_360()
                    self.is_navigating = False
                    self.goal_start_time = None
                    self.goal_gain = None
                    self.last_stale_check_time = None
                    self.no_frontier_count = 0  # 重置计数
                    self._clear_goal_marker()
                    rospy.loginfo("[前沿探索] 继续检测前沿")

                # 导航失败（中止/拒绝/抢占）
                elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                               GoalStatus.PREEMPTED):
                    rospy.logwarn(
                        "[前沿探索] ✗ 导航失败 (状态码: %d)，目标加入黑名单",
                        state)
                    self.is_navigating = False
                    self.goal_start_time = None
                    self.goal_gain = None
                    self.last_stale_check_time = None
                    # 注意：黑名单在 send_goal 之前已加入，此处无需重复
                    self._clear_goal_marker()

                # 导航超时检查
                elif state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                    if self.goal_start_time is not None:
                        elapsed = (rospy.Time.now() -
                                   self.goal_start_time).to_sec()
                        if elapsed > self.nav_timeout:
                            rospy.logwarn(
                                "[前沿探索] ⏰ 导航超时 (%.0f s)，取消目标",
                                elapsed)
                            self.cancel_goal()
                            self._clear_goal_marker()

                rate.sleep()
                continue

            # ── 空闲状态：检测前沿 ──
            frontiers = self.detect_frontiers()
            rospy.loginfo("[前沿探索] 检测到 %d 个有效前沿簇",
                          len(frontiers))

            # 无前沿
            if not frontiers:
                self.no_frontier_count += 1
                if self.no_frontier_count >= self.max_no_frontier:
                    self.exploration_complete = True
                else:
                    rospy.loginfo(
                        "[前沿探索] 暂无前沿 (%d/%d)，继续等待地图更新",
                        self.no_frontier_count, self.max_no_frontier)
                rate.sleep()
                continue

            # 有前沿：重置计数
            self.no_frontier_count = 0

            # 选择最优目标
            result = self.select_best_goal(frontiers)
            if result is None:
                rospy.loginfo(
                    "[前沿探索] 所有前沿被过滤（黑名单/太近），等待地图更新")
                rate.sleep()
                continue

            gx, gy, score = result
            rospy.loginfo(
                "[前沿探索] ★ 选中目标 (%.2f, %.2f) 评分=%.3f",
                gx, gy, score)

            # 发布可视化标记
            self.publish_frontier_markers(frontiers, selected_goal=(gx, gy))
            self.publish_goal_marker(gx, gy)

            # 加入黑名单（防止导航失败后重复尝试同一位置）
            self.blacklist.append((gx, gy))

            # 发送导航目标
            self.send_goal(gx, gy)
            rate.sleep()


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════
if __name__ == '__main__':
    rospy.init_node('frontier_exploration')
    explorer = FrontierExplorer()
    try:
        explorer.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("[前沿探索] 节点退出")
