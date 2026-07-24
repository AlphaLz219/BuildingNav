#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前沿探索节点（Frontier Exploration Node）
=========================================

功能流程：
  1. 订阅 SLAM Toolbox 发布的 /map（OccupancyGrid）
  2. 检测前沿 —— 已知自由空间与未知空间的交界
  3. 对前沿点做连通域聚类，过滤噪声小簇
  4. 用评分函数选出最优前沿簇作为导航目标
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
from geometry_msgs.msg import Point, PoseStamped
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
        # 评分权重 —— 前沿大小（信息增益）
        self.weight_size = rospy.get_param('~score_size_weight', 0.6)
        # 评分权重 —— 距离（越近越好）
        self.weight_dist = rospy.get_param('~score_distance_weight', 0.4)
        # 评分权重 —— 目标点净空（距障碍物多远），0=不纳入评分
        self.weight_clearance = rospy.get_param('~score_clearance_weight', 0.3)
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
        # 重规划前要求 /map 至少更新过一次（避免 SLAM 未就绪时反复重发）
        self._last_map_stamp = None

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
        通过 TF 查询机器人当前位置（map 坐标系下的世界坐标）
        返回 (x, y) 或 (None, None) 如果查询失败
        """
        try:
            (trans, _) = self.tf_listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn("[前沿探索] TF 查询 /map → /base_footprint 失败")
            return None, None

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

    # ────────────── 目标选择 ──────────────
    def select_best_goal(self, frontiers):
        """
        从前沿簇中选择评分最高的导航目标。

        评分函数（三因素加权）：
            score = w_size × (size/max_size)       信息增益分量
                  + w_dist × 1/(dist+0.1)          距离分量
                  + w_clear × clearance_score      安全净空分量

        过滤条件：
            - 黑名单中的前沿
            - 距离机器人太近的前沿

        返回：(goal_x, goal_y, score) 或 None
        """
        if not frontiers:
            return None

        robot_x, robot_y = self._get_robot_pose()
        if robot_x is None:
            return None

        # 按权重分配比例，确保三个分量和为 1.0
        # 净空权重从已有的 size+dist 中匀出一部分
        w_clear = self.weight_clearance
        w_size = self.weight_size * (1.0 - w_clear)
        w_dist = self.weight_dist * (1.0 - w_clear)

        # 最大前沿簇大小（用于归一化）
        max_size = max(f[2] for f in frontiers)

        candidates = []
        for (fx, fy, fsize) in frontiers:
            # 跳过黑名单
            if self._is_blacklisted(fx, fy):
                continue

            # 计算到机器人的距离
            dist = np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            if dist < self.min_goal_distance:
                continue

            # ── 查询该目标点的净空（通过距离变换缓存）──
            clearance = 0.0
            if self._dist_transform is not None:
                col = int((fx - self.map_origin_x) / self.map_resolution)
                row = int((fy - self.map_origin_y) / self.map_resolution)
                if 0 <= row < self.map_height and 0 <= col < self.map_width:
                    clearance = (
                        self._dist_transform[row, col] * self.map_resolution)
            # 归一化：假设最大合理净空约 2m
            clearance_score = min(clearance / 2.0, 1.0)

            # 三项归一化评分
            size_score = fsize / max_size           # 0~1
            dist_score = 1.0 / (dist + 0.1)         # 越近越高
            score = (w_size * size_score +
                     w_dist * dist_score +
                     w_clear * clearance_score)
            candidates.append((fx, fy, fsize, score))

        if not candidates:
            return None

        # 按评分降序排列
        candidates.sort(key=lambda c: c[3], reverse=True)

        # 按评分从高到低遍历，选第一个通过 make_plan 预检的目标
        for (fx, fy, fsize, score) in candidates:
            if not self._is_goal_reachable(fx, fy):
                rospy.loginfo(
                    "[前沿探索] make_plan 预检失败，跳过 (%.2f,%.2f) 评分=%.3f",
                    fx, fy, score)
                # 不可达的目标也加入黑名单，避免后续重复检查
                self.blacklist.append((fx, fy))
                continue

            rospy.loginfo(
                "[前沿探索] make_plan 预检通过 (%.2f,%.2f) 评分=%.3f",
                fx, fy, score)
            return fx, fy, score

        # 所有候选都未通过预检
        return None

    def _is_blacklisted(self, x, y):
        """检查 (x, y) 是否在黑名单中（之前导航失败的目标附近）"""
        for bx, by in self.blacklist:
            if (x - bx)**2 + (y - by)**2 < self.blacklist_radius**2:
                return True
        return False

    def _is_goal_reachable(self, x, y):
        """
        通过 /move_base/make_plan 服务预检查目标是否可达。

        校验条件：
          1. 路径至少 5 个位姿（排除仅有起点的退化情况）
          2. 路径终点与目标距离 < 1.0m（确认真到达目标附近，而非半路中断）

        返回：True（可达） 或 False
        """
        robot_x, robot_y = self._get_robot_pose()
        if robot_x is None:
            return False

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
                return False

            # 校验 1：路径至少 5 个点，排除退化情况
            if len(resp.plan.poses) < 5:
                rospy.loginfo(
                    "[前沿探索] make_plan 路径太短 (%d 点)，跳过 (%.2f,%.2f)",
                    len(resp.plan.poses), x, y)
                return False

            # 校验 2：路径终点必须在目标附近（< 1.0m），否则是半路中断
            last_pose = resp.plan.poses[-1].pose.position
            end_dist = np.sqrt((last_pose.x - x)**2 + (last_pose.y - y)**2)
            if end_dist > 1.0:
                rospy.loginfo(
                    "[前沿探索] make_plan 路径未达目标 (距目标 %.2fm)，"
                    "跳过 (%.2f,%.2f)",
                    end_dist, x, y)
                return False

            return True
        except rospy.ServiceException as e:
            rospy.logwarn("[前沿探索] make_plan 服务调用异常: %s", e)
            return False

    # ────────────── 导航控制 ──────────────
    def send_goal(self, x, y):
        """
        发送导航目标到 move_base（通过 ActionClient）
        
        使用 MoveBaseAction 而非 /move_base_simple/goal，
        因为 ActionClient 支持：取消目标、获取详细状态、等待结果
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        # 不指定朝向（w=1 表示无旋转）
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("[前沿探索] 发送导航目标: (%.2f, %.2f)", x, y)
        self.ac.send_goal(goal)
        self.is_navigating = True
        self.goal_start_time = rospy.Time.now()
        self.current_goal_xy = (x, y)
        self.last_replan_time = rospy.Time.now()

    def cancel_goal(self):
        """取消当前导航目标"""
        self.ac.cancel_goal()
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_xy = None
        self.last_replan_time = None
        rospy.loginfo("[前沿探索] 已取消当前导航目标")

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
        robot_x, robot_y = self._get_robot_pose()
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

                # ── 周期性重规划（建图期核心机制）──
                # 仅在导航仍活跃（PENDING/ACTIVE）时触发，避免与成功/失败分支冲突
                if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                    self._maybe_replan()

                # 导航成功
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("[前沿探索] ✓ 导航成功，继续检测前沿")
                    self.is_navigating = False
                    self.goal_start_time = None
                    self.no_frontier_count = 0  # 重置计数
                    self._clear_goal_marker()

                # 导航失败（中止/拒绝/抢占）
                elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                               GoalStatus.PREEMPTED):
                    rospy.logwarn(
                        "[前沿探索] ✗ 导航失败 (状态码: %d)，目标加入黑名单",
                        state)
                    self.is_navigating = False
                    self.goal_start_time = None
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
