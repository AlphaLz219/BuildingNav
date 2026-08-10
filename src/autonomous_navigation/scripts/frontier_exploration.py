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

觉得代码太多可以 ctrl+K , 然后 ctrl+2 折叠函数 , 方便阅读.
"""

import rospy
import numpy as np
import cv2
import actionlib
import tf
import json
import os

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Twist, Pose2D
from std_msgs.msg import ColorRGBA
from building_generator_interfaces.srv import CallElevator, SetDoorState
from std_srvs.srv import Empty
from slam_toolbox_msgs.srv import SerializePoseGraph, DeserializePoseGraph


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
        self.init_forward_speed    = rospy.get_param('~init_forward_speed', 0.5)
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

        # ── 状态持久化：保存/加载初始扫描结果 ──
        self.save_state_path = rospy.get_param('~save_state_path', '')
        self.load_state_path = rospy.get_param('~load_state_path', '')

        # ── 多楼层会话管理 ──
        # SLAM 会话文件存放目录（serialize/deserialize 产物，每层一个 .session）
        self.floor_session_dir = rospy.get_param('~floor_session_dir', '')
        # 当前楼层（0-based），启动时所在楼层
        self.current_floor = rospy.get_param('~current_floor', 0)
        # 重启 slam_toolbox 用的 launch（替代 Noetic 缺失的 /slam_toolbox/reset 服务）
        self.slam_launch_pkg = rospy.get_param(
            '~slam_launch_pkg', 'autonomous_navigation')
        self.slam_launch_file = rospy.get_param(
            '~slam_launch_file', 'slam_toolbox.launch')
        # 重启后等待 /map 恢复的超时（秒）
        self.slam_restart_timeout = rospy.get_param(
            '~slam_restart_timeout', 30.0)

        # ── 电梯调试模式 ──
        self.debug_elevator = rospy.get_param('~debug_elevator', True)
        self.target_floor = rospy.get_param('~target_floor', 1)

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
        self.door_positions = []       # [(door_y, side), ...] side='left'/'right'
        self.stair_center = None       # (sx, sy) 楼梯中心
        self.elevator_center = None    # (ex, ey) 电梯中心（当前楼层地图帧）
        self.floor_elevator_center = {}  # {floor: (ex, ey)} 各楼层地图帧中的电梯中心
        # ── 帧机制 ──
        # 每层楼的地图帧不同，电梯中心是该帧中的固定锚点：
        #   启动楼层 = 实测 (ex, ey)；首次访问的楼层（重启 SLAM）=
        #   实测机器人在新地图中的位置。共享坐标以启动楼层帧存储，
        # 切换楼层时按"两帧电梯中心之差"平移（见 _shift_shared_to_floor）
        self._slam_proc = None                  # 后台 slam_toolbox roslaunch 进程
        self.room_positions = []       # [(center_x, center_y, side, y_bottom, y_top), ...]
        # 持久化字段（从文件加载时填充）
        self.saved_entrance_y = None
        self.saved_corridor_end_y = None

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

        # 电梯/门控制服务
        rospy.loginfo("[前沿探索] 等待 /call_elevator 服务...")
        rospy.wait_for_service('/call_elevator', timeout=5.0)
        self._call_elevator_srv = rospy.ServiceProxy(
            '/call_elevator', CallElevator)
        rospy.loginfo("[前沿探索] /call_elevator 已连接")

        rospy.loginfo("[前沿探索] 等待 /set_door_state 服务...")
        rospy.wait_for_service('/set_door_state', timeout=5.0)
        self._set_door_srv = rospy.ServiceProxy(
            '/set_door_state', SetDoorState)
        rospy.loginfo("[前沿探索] /set_door_state 已连接")

        # SLAM Toolbox 多楼层会话管理服务（serialize/deserialize）
        # 注意：Noetic 版 slam_toolbox 无 /slam_toolbox/reset 服务，
        # 会话重置改用"重启 slam_toolbox 节点"实现（见 _restart_slam）
        self._serialize_srv = None
        self._deserialize_srv = None
        try:
            rospy.loginfo("[会话] 等待 /slam_toolbox/serialize_map 服务...")
            rospy.wait_for_service('/slam_toolbox/serialize_map', timeout=5.0)
            self._serialize_srv = rospy.ServiceProxy(
                '/slam_toolbox/serialize_map', SerializePoseGraph)
            rospy.loginfo("[会话] /slam_toolbox/serialize_map 已连接")
        except rospy.ROSException:
            rospy.logwarn("[会话] serialize_map 服务不可用（跳过）")
        try:
            rospy.loginfo("[会话] 等待 /slam_toolbox/deserialize_map 服务...")
            rospy.wait_for_service('/slam_toolbox/deserialize_map', timeout=5.0)
            self._deserialize_srv = rospy.ServiceProxy(
                '/slam_toolbox/deserialize_map', DeserializePoseGraph)
            rospy.loginfo("[会话] /slam_toolbox/deserialize_map 已连接")
        except rospy.ROSException:
            rospy.logwarn("[会话] deserialize_map 服务不可用（跳过）")

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
        # 发布器 —— 当前扫描房间可视化（半透明彩色框）
        self.room_marker_pub = rospy.Publisher(
            '/room_scan_markers', MarkerArray, queue_size=1, latch=True)

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
    def detect_frontiers(self, bounds=None):
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

        参数:
            bounds: (x_min, x_max, y_min, y_max) 世界坐标，可选。
                    仅在该矩形范围内检测前沿（用于房间内探索）。

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

        # ── 房间边界掩码（如果提供了 bounds）──
        if bounds is not None:
            bx_min, bx_max, by_min, by_max = bounds
            h, w = self.map_data.shape
            res = self.map_resolution
            r_min = max(0, int((by_min - self.map_origin_y) / res))
            r_max = min(h, int((by_max - self.map_origin_y) / res))
            c_min = max(0, int((bx_min - self.map_origin_x) / res))
            c_max = min(w, int((bx_max - self.map_origin_x) / res))
            room_mask = np.zeros_like(frontier_mask)
            room_mask[r_min:r_max, c_min:c_max] = 1
            frontier_mask = frontier_mask & room_mask

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
    def _move_forward(self, distance, speed=0.35):
        """
        机器人直线前进指定距离（开环控制，基于时间估算）。

        参数:
            distance: 前进距离（米）
            speed:    前进速度（m/s），默认 0.35
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
        从机器人当前位置沿走廊中心线向 +y 扫描，三列同时检测尽头。

        检测点: x=0（中心）, x=-0.5, x=+0.5（走廊宽度内）
        只有三列同时在 <0.5m 内碰真墙(100)，才确认尽头，防止误判。

        返回: (end_y, definitive)
        """
        if self.map_data is None or entrance_y is None:
            return None, False

        rx, ry, _ = self._get_robot_pose()
        if ry is None:
            ry = entrance_y

        res = self.map_resolution
        h, w = self.map_data.shape
        # 三列检测点，x 坐标均保证在走廊宽度内
        check_xs = [0.0, -0.5, 0.5]
        check_cols = [int((x - self.map_origin_x) / res) for x in check_xs]
        step = 0.25
        scan_y = ry + 0.3
        max_scan = ry + 50.0

        rospy.loginfo("[尽头检测] ====== 从机器人 y=%.2f x=[%.1f,%.1f,%.1f] 向前扫描 ======",
                      ry, check_xs[0], check_xs[1], check_xs[2])

        while scan_y <= max_scan:
            row = int((scan_y - self.map_origin_y) / res)
            if row < 0 or row >= h:
                break

            # ── 三列各自测前方自由延伸 ──
            forward_dists = []
            blockers = []
            for col_c in check_cols:
                if col_c < 0 or col_c >= w:
                    forward_dists.append(0.0)
                    blockers.append(-1)
                    continue

                cur = self.map_data[row, col_c]
                if cur == -1:
                    rospy.loginfo("[尽头检测] y=%.2f x=%.1f 未覆盖 → 需前进建图",
                                  scan_y, check_xs[len(forward_dists)])
                    return scan_y, False
                if cur == 100:
                    rospy.loginfo("[尽头检测] y=%.2f x=%.1f 直接碰墙",
                                  scan_y, check_xs[len(forward_dists)])
                    forward_dists.append(0.0)
                    blockers.append(100)
                    continue

                fwd = 0
                blk = 0
                for dr in range(1, int(10.0 / res)):
                    r_fwd = row + dr
                    if r_fwd >= h:
                        break
                    v = self.map_data[r_fwd, col_c]
                    if v == 100:
                        blk = 100
                        break
                    if v == -1:
                        blk = -1
                        break
                    fwd += 1
                forward_dists.append(fwd * res)
                blockers.append(blk)

            min_fwd = min(forward_dists)
            rospy.loginfo("[尽头检测] y=%.2f 前向=[%.2f,%.2f,%.2f]m min=%.2f",
                          scan_y,
                          forward_dists[0], forward_dists[1], forward_dists[2],
                          min_fwd)

            # ── 判定：三列中有任一列前方为未知 → 需前进 ──
            if -1 in blockers:
                for i, blk in enumerate(blockers):
                    if blk == -1:
                        rospy.loginfo("[尽头检测] ★ 前方未知: y=%.2f x=%.1f (需前进建图)",
                                      scan_y, check_xs[i])
                return scan_y, False

            # ── 三列全部碰墙 <0.5m → 确认尽头 ──
            if min_fwd < 0.5 and all(b == 100 for b in blockers):
                rospy.loginfo("[尽头检测] ★★★ 三列全碰墙，尽头确认: y=%.2f ★★★",
                              scan_y)
                return scan_y, True

            # 跳步
            if min_fwd > 2.0:
                scan_y += min_fwd - 0.5
            else:
                scan_y += step

        rospy.loginfo("[尽头检测] 扫描 %dm 未发现尽头",
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
        min_door_width = 0.6  # 门至少宽0.6m，过滤墙壁扫描噪声

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

            rospy.logdebug("[房门扫描] y=%.2f  L=%.2fm  R=%.2fm",
                          scan_y, left, right)

            # ── 左门检测 ──
            if left > door_thresh and not in_left_door:
                in_left_door = True
                left_door_start = scan_y
                rospy.logdebug("[房门扫描]   左侧门开始 @ y=%.2f", scan_y)
            elif left <= door_thresh and in_left_door:
                in_left_door = False
                span = scan_y - left_door_start
                door_y = (left_door_start + scan_y) / 2.0
                if span >= min_door_width:
                    doors.append((door_y, 'left'))
                    rospy.loginfo("[房门扫描] ★ 左侧门: y=%.2f (宽 %.2fm)",
                                  door_y, span)
                else:
                    rospy.logdebug("[房门扫描]   左侧门跳过(宽%.2fm < %.2fm 噪声)",
                                  span, min_door_width)

            # ── 右门检测 ──
            if right > door_thresh and not in_right_door:
                in_right_door = True
                right_door_start = scan_y
                rospy.logdebug("[房门扫描]   右侧门开始 @ y=%.2f", scan_y)
            elif right <= door_thresh and in_right_door:
                in_right_door = False
                span = scan_y - right_door_start
                door_y = (right_door_start + scan_y) / 2.0
                if span >= min_door_width:
                    doors.append((door_y, 'right'))
                    rospy.loginfo("[房门扫描] ★ 右侧门: y=%.2f (宽 %.2fm)",
                                  door_y, span)
                else:
                    rospy.logdebug("[房门扫描]   右侧门跳过(宽%.2fm < %.2fm 噪声)",
                                  span, min_door_width)

            prev_left = left
            prev_right = right
            scan_y += step

        # 扫描结束还在门内 → 宽度足够才记录
        if in_left_door and left_door_start is not None:
            span = scan_y - left_door_start
            door_y = (left_door_start + scan_y) / 2.0
            if span >= min_door_width:
                doors.append((door_y, 'left'))
                rospy.loginfo("[房门扫描] ★ 左侧门(未闭合): y=%.2f (宽 %.2fm)",
                              door_y, span)
            else:
                rospy.logdebug("[房门扫描]   左侧门(未闭合)跳过(宽%.2fm 噪声)", span)
        if in_right_door and right_door_start is not None:
            span = scan_y - right_door_start
            door_y = (right_door_start + scan_y) / 2.0
            if span >= min_door_width:
                doors.append((door_y, 'right'))
                rospy.loginfo("[房门扫描] ★ 右侧门(未闭合): y=%.2f (宽 %.2fm)",
                              door_y, span)
            else:
                rospy.logdebug("[房门扫描]   右侧门(未闭合)跳过(宽%.2fm 噪声)", span)

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
        rospy.logdebug("[房门扫描] 📍 已发布 %d 个房门标记", len(doors))

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
            self.stair_center = (sx, sy)
            rospy.loginfo("[楼梯检测] ★★★ 楼梯中心: (%.2f, %.2f) ★★★", sx, sy)
            self._publish_block_marker(sx, sy, 'stairs',
                                       ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.7))
        else:
            rospy.loginfo("[楼梯检测] 未检测到完整楼梯墙壁")

        if elev_L is not None and elev_R is not None:
            ex = (elev_L + elev_R) / 2.0
            ey = entrance_y * 0.35
            self.elevator_center = (ex, ey)
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

    # ────────────── 房间扫描可视化 ──────────────
    def _publish_current_room_marker(self, room_idx, bounds, side, total_rooms):
        """
        发布当前扫描房间的半透明彩色框标记到 RViz。

        每间房间用不同的颜色，左侧房间用蓝色系，右侧房间用绿色系。
        已扫描过的房间保留为低透明度，当前房间高透明度。
        """
        x_min, x_max, y_min, y_max = bounds

        # 清除旧标记，重新发布所有房间状态
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'room_scan'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        # 发布所有已扫描房间（低透明度）
        for i in range(room_idx):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'room_scan'
            m.id = i + 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            s = self.room_positions[i]
            cx = (s[0])  # center_x
            cy = s[1]    # center_y
            s_side = s[2]
            s_y_min = s[3]
            s_y_max = s[4]
            if s_side == 'left':
                s_x_min = self.building_left if self.building_left else -9.7
                s_x_max = -1.1
            else:
                s_x_min = 1.1
                s_x_max = self.building_right if self.building_right else 9.7
            m.pose.position = Point(
                x=(s_x_min + s_x_max) / 2.0,
                y=(s_y_min + s_y_max) / 2.0,
                z=0.05)
            m.pose.orientation.w = 1.0
            m.scale.x = abs(s_x_max - s_x_min)
            m.scale.y = abs(s_y_max - s_y_min)
            m.scale.z = 0.1
            if s_side == 'left':
                m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.25)
            else:
                m.color = ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.25)
            m.lifetime = rospy.Duration(0)
            marker_array.markers.append(m)

        # 发布当前房间（高透明度 + 边框效果）
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'room_scan'
        m.id = room_idx + 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position = Point(
            x=(x_min + x_max) / 2.0,
            y=(y_min + y_max) / 2.0,
            z=0.05)
        m.pose.orientation.w = 1.0
        m.scale.x = abs(x_max - x_min)
        m.scale.y = abs(y_max - y_min)
        m.scale.z = 0.1
        if side == 'left':
            m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.5)
        else:
            m.color = ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.5)
        m.lifetime = rospy.Duration(0)
        marker_array.markers.append(m)

        # 房间标签
        txt = Marker()
        txt.header.frame_id = 'map'
        txt.header.stamp = rospy.Time.now()
        txt.ns = 'room_scan_label'
        txt.id = room_idx + 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position = Point(
            x=(x_min + x_max) / 2.0,
            y=(y_min + y_max) / 2.0,
            z=0.3)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.5
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        txt.text = f"Room {room_idx + 1}/{total_rooms} ({side})"
        txt.lifetime = rospy.Duration(0)
        marker_array.markers.append(txt)

        self.room_marker_pub.publish(marker_array)
        rospy.loginfo("[房间标记] 📍 当前扫描: 第 %d/%d 间 %s 房间",
                      room_idx + 1, total_rooms, side)

    def _clear_room_markers(self):
        """清除所有房间扫描标记"""
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = rospy.Time.now()
        clear.ns = 'room_scan'
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        clear2 = Marker()
        clear2.header.frame_id = 'map'
        clear2.header.stamp = rospy.Time.now()
        clear2.ns = 'room_scan_label'
        clear2.id = 0
        clear2.action = Marker.DELETEALL
        marker_array.markers.append(clear2)
        self.room_marker_pub.publish(marker_array)

    # ────────────── 电梯控制 ──────────────
    def _call_elevator(self, elevator_id, target_floor, open_doors=False):
        """
        呼叫电梯到指定楼层。

        参数:
            elevator_id: 电梯 ID（默认 'elevator_main'）
            target_floor: 目标楼层（0-based）
            open_doors: 是否开门

        返回: True 成功, False 失败
        """
        try:
            resp = self._call_elevator_srv(
                elevator_id=elevator_id,
                target_floor=target_floor,
                open_doors=open_doors)
            rospy.loginfo(
                "[电梯] ✓ 呼叫到 %d 楼: accepted=%s, floor=%d, state=%s",
                target_floor, resp.accepted, resp.current_floor, resp.state)
            if not resp.accepted:
                rospy.logwarn("[电梯] 呼叫被拒绝: %s", resp.message)
            return resp.accepted
        except rospy.ServiceException as e:
            rospy.logwarn("[电梯] 服务调用失败: %s", e)
            return False

    def _set_door(self, door_id, open):
        """
        开关电梯厅门。

        参数:
            door_id: 门 ID（如 'elevator_floor_0'）
            open: True 开门, False 关门

        返回: True 成功, False 失败
        """
        action = "开门" if open else "关门"
        try:
            resp = self._set_door_srv(door_id=door_id, open=open)
            rospy.loginfo(
                "[电梯] ✓ %s %s: accepted=%s, state=%s",
                action, door_id, resp.accepted, resp.state)
            if not resp.accepted:
                rospy.logwarn("[电梯] %s被拒绝: %s", action, resp.message)
            return resp.accepted
        except rospy.ServiceException as e:
            rospy.logwarn("[电梯] 服务调用失败: %s", e)
            return False

    def _rotate_to_face(self, target_x, target_y, angular_vel=0.8):
        """
        旋转机器人使其朝向目标点。

        计算目标点相对于机器人的方位角，然后旋转到该方向。

        参数:
            target_x: 目标点 x 坐标
            target_y: 目标点 y 坐标
            angular_vel: 旋转角速度 (rad/s)
        """
        robot_x, robot_y, robot_yaw = self._get_robot_pose()
        if robot_x is None:
            rospy.logwarn("[朝向] 无法获取机器人位姿")
            return

        # 计算目标方位角
        target_yaw = np.arctan2(target_y - robot_y, target_x - robot_x)

        # 计算需要旋转的角度差
        yaw_diff = target_yaw - robot_yaw
        # 归一化到 [-π, π]
        while yaw_diff > np.pi:
            yaw_diff -= 2.0 * np.pi
        while yaw_diff < -np.pi:
            yaw_diff += 2.0 * np.pi

        rospy.loginfo(
            "[朝向] 当前朝向: %.1f°, 目标朝向: %.1f°, 需旋转: %.1f°",
            np.degrees(robot_yaw), np.degrees(target_yaw), np.degrees(yaw_diff))

        # 如果角度差很小，不需要旋转
        if abs(yaw_diff) < np.radians(5.0):
            rospy.loginfo("[朝向] 已经朝向目标，跳过旋转")
            return

        # 执行旋转
        rotation_time = abs(yaw_diff) / abs(angular_vel)
        direction = 1.0 if yaw_diff > 0 else -1.0

        rospy.loginfo("[朝向] 旋转 %.1f° (%.1fs)...", np.degrees(yaw_diff), rotation_time)

        cmd = Twist()
        cmd.angular.z = direction * angular_vel

        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now() - start).to_sec() < rotation_time:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # 停止
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.3)
        rospy.loginfo("[朝向] ✓ 旋转完成")

    def _measure_free_distance(self, direction_x, direction_y, max_dist=5.0):
        """
        从机器人当前位置沿指定方向测量自由空间距离。

        沿方向射线扫描地图，遇到障碍物(100)或未知(-1)停止。

        参数:
            direction_x: 方向向量 x 分量（世界坐标）
            direction_y: 方向向量 y 分量
            max_dist: 最大测量距离（米）

        返回: 自由空间距离（米）
        """
        if self.map_data is None:
            return 0.0

        robot_x, robot_y, _ = self._get_robot_pose()
        if robot_x is None:
            return 0.0

        res = self.map_resolution

        # 归一化方向向量
        norm = np.sqrt(direction_x**2 + direction_y**2)
        if norm < 1e-6:
            return 0.0
        dx = direction_x / norm
        dy = direction_y / norm

        # 沿方向步进扫描
        step = res  # 每步一个栅格
        dist = 0.0
        while dist < max_dist:
            dist += step
            wx = robot_x + dx * dist
            wy = robot_y + dy * dist

            col = int((wx - self.map_origin_x) / res)
            row = int((wy - self.map_origin_y) / res)

            if col < 0 or col >= self.map_width or row < 0 or row >= self.map_height:
                break

            val = self.map_data[row, col]
            if val == 100 or val == -1:
                break

        rospy.loginfo(
            "[测距] 方向(%.1f, %.1f): 自由空间 %.2fm",
            direction_x, direction_y, dist)
        return dist

    # ────────────── 多楼层 SLAM 会话管理 ──────────────
    def _floor_session_path(self, floor):
        """楼层 SLAM 会话文件路径"""
        if not self.floor_session_dir:
            return ''
        return os.path.join(self.floor_session_dir, 'floor_%d.session' % floor)

    def _has_floor_session(self, floor):
        """该楼层是否已有保存的 SLAM 会话"""
        path = self._floor_session_path(floor)
        if not path:
            return False
        # serialize_map 生成 .data 和 .posegraph 两个文件
        return os.path.exists(path + '.posegraph') or os.path.exists(path)

    def _serialize_floor(self, floor):
        """
        保存当前楼层 SLAM 会话到文件（乘梯前调用）。

        通过 /slam_toolbox/serialize_map 把当前位姿图+地图写入 floor_N.session，
        之后回到该楼层可完整恢复。
        """
        if self._serialize_srv is None:
            rospy.logwarn("[会话] serialize_map 服务不可用，跳过保存")
            return False
        path = self._floor_session_path(floor)
        if not path:
            rospy.logwarn("[会话] 未配置 floor_session_dir，跳过保存")
            return False
        try:
            # 确保会话目录存在
            os.makedirs(self.floor_session_dir, exist_ok=True)
            self._serialize_srv(filename=path)
            rospy.loginfo("[会话] ✓ 楼层 %d 会话已保存: %s", floor, path)
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("[会话] serialize 失败: %s", e)
            return False

    def _deserialize_floor(self, floor, init_x=0.0, init_y=0.0, init_yaw=0.0):
        """
        恢复目标楼层的 SLAM 会话（乘梯到达后、开门前调用）。

        match_type=START_AT_GIVEN_POSE(2)：从指定初始位姿开始扫描匹配定位。
        初始位姿应为机器人在该楼层地图中的位置（电梯中心附近）。
        """
        if self._deserialize_srv is None:
            rospy.logwarn("[会话] deserialize_map 服务不可用，跳过恢复")
            return False
        path = self._floor_session_path(floor)
        if not path or not (os.path.exists(path + '.posegraph') or os.path.exists(path)):
            rospy.logwarn("[会话] 楼层 %d 无会话文件，无法恢复: %s", floor, path)
            return False
        try:
            self._deserialize_srv(
                filename=path,
                match_type=2,  # START_AT_GIVEN_POSE
                initial_pose=Pose2D(x=init_x, y=init_y, theta=init_yaw))
            rospy.loginfo(
                "[会话] ✓ 楼层 %d 会话已恢复 (初始位姿 %.1f,%.1f,%.1f°)",
                floor, init_x, init_y, np.degrees(init_yaw))
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("[会话] deserialize 失败: %s", e)
            return False

    def _restart_slam(self, timeout=None):
        """
        通过重启 slam_toolbox 节点实现会话重置（Noetic 版无 reset 服务）。

        流程：
          1. rosnode kill /slam_toolbox（旧 roslaunch 默认无 respawn，不会自动拉起）
          2. 后台 roslaunch slam_toolbox.launch（subprocess.Popen）
          3. 等待 /map 恢复，并刷新 serialize/deserialize 服务代理

        效果：位姿图清空、全新建图。重启后机器人在新地图中的位置
        由 SLAM 决定（map_start_pose 或 odom），调用方通过
        _get_robot_pose() 实测其位置作为该楼层电梯中心。
        返回: True 成功, False 失败/超时
        """
        import subprocess
        if timeout is None:
            timeout = self.slam_restart_timeout

        # ── 1. 关闭旧节点 ──
        rospy.loginfo("[会话] 关闭旧 slam_toolbox 节点...")
        try:
            subprocess.call(['rosnode', 'kill', '/slam_toolbox'],
                            timeout=10.0)
        except Exception as e:
            rospy.logwarn("[会话] rosnode kill 异常: %s", e)
        # 等旧节点彻底退出 + 在途 /map 帧处理完，避免误判为"新节点已恢复"
        rospy.sleep(3.0)

        # ── 2. 后台重新启动 ──
        rospy.loginfo("[会话] 重新启动 slam_toolbox (%s %s)...",
                      self.slam_launch_pkg, self.slam_launch_file)
        try:
            self._slam_proc = subprocess.Popen(
                ['roslaunch', self.slam_launch_pkg, self.slam_launch_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        except Exception as e:
            rospy.logerr("[会话] 启动 slam_toolbox 失败: %s", e)
            return False

        # ── 3. 清空缓存，等待新节点首帧 /map ──
        # 电梯内机器人静止，新节点可能只发首帧（minimum_travel_distance
        # 会跳过后续 scan），所以只要求"出现一帧新地图"即可。
        rospy.loginfo("[会话] 等待新节点首帧 /map (最多 %.0fs)...", timeout)
        self.map_data = None
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.map_data is not None:
                rospy.loginfo(
                    "[会话] ✓ SLAM 重启完成，/map 已恢复 "
                    "(地图原点 %.2f, %.2f)",
                    self.map_origin_x, self.map_origin_y)
                self._refresh_slam_services()
                return True
            rate.sleep()
        rospy.logwarn("[会话] 等待 /map 恢复超时 (%ds)", timeout)
        return False

    def _refresh_slam_services(self):
        """SLAM 节点重启后刷新 serialize/deserialize 服务代理"""
        try:
            rospy.wait_for_service('/slam_toolbox/serialize_map', timeout=5.0)
            self._serialize_srv = rospy.ServiceProxy(
                '/slam_toolbox/serialize_map', SerializePoseGraph)
        except rospy.ROSException:
            self._serialize_srv = None
            rospy.logwarn("[会话] 刷新 serialize_map 服务失败")
        try:
            rospy.wait_for_service('/slam_toolbox/deserialize_map', timeout=5.0)
            self._deserialize_srv = rospy.ServiceProxy(
                '/slam_toolbox/deserialize_map', DeserializePoseGraph)
        except rospy.ROSException:
            self._deserialize_srv = None
            rospy.logwarn("[会话] 刷新 deserialize_map 服务失败")

    def _clear_costmaps(self):
        """清除 move_base 代价地图，丢弃旧楼层障碍数据"""
        try:
            clear_proxy = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_proxy()
            rospy.loginfo("[会话] ✓ move_base 代价地图已清除")
        except rospy.ServiceException as e:
            rospy.logwarn("[会话] 清除代价地图失败: %s", e)

    def _restore_shared_features(self):
        """
        复用共享楼层布局坐标（门/房间/禁区/楼梯/电梯）并重发可视化标记。

        楼层布局除危险物/家具外完全相同，因此 door_positions、
        forbidden_zones、stair_center、elevator_center 等坐标可直接复用。
        """
        if self.forbidden_zones:
            self._mark_forbidden_zones()
            self._publish_forbidden_zone_markers()
        if self.saved_entrance_y is not None:
            self._publish_entrance_marker(0.0, self.saved_entrance_y)
        if self.saved_corridor_end_y is not None:
            self._publish_entrance_marker(0.0, self.saved_corridor_end_y)
        if self.door_positions:
            self._publish_door_markers(self.door_positions)
        if self.stair_center:
            sx, sy = self.stair_center
            self._publish_block_marker(sx, sy, 'stairs',
                                       ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.7))
        if self.elevator_center:
            ex, ey = self.elevator_center
            self._publish_block_marker(ex, ey, 'elevator',
                                       ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.7))

    def _translate_shared_features(self, dx, dy):
        """
        平移所有共享楼层布局坐标（跨楼层帧切换）。

        每层楼地图帧不同，切换时按"两帧电梯中心之差"平移，
        使门/房间/禁区/楼梯/电梯坐标对齐到目标楼层帧。
        """
        # 禁区 [(x_min, x_max, y_min, y_max)]
        self.forbidden_zones = [
            (x_min + dx, x_max + dx, y_min + dy, y_max + dy)
            for (x_min, x_max, y_min, y_max) in self.forbidden_zones]
        # 门 [(door_y, side)]（x 隐含为走廊墙边 ±1.2，不存数据）
        self.door_positions = [
            (dy_ + dy, side) for (dy_, side) in self.door_positions]
        # 房间 [(center_x, center_y, side, y_bottom, y_top)]
        self.room_positions = [
            (cx + dx, cy + dy, side, y_bottom + dy, y_top + dy)
            for (cx, cy, side, y_bottom, y_top) in self.room_positions]
        # 楼梯 / 电梯
        if self.stair_center:
            sx, sy = self.stair_center
            self.stair_center = (sx + dx, sy + dy)
        if self.elevator_center:
            ex, ey = self.elevator_center
            self.elevator_center = (ex + dx, ey + dy)
        # 走廊端点
        if self.saved_entrance_y is not None:
            self.saved_entrance_y += dy
        if self.saved_corridor_end_y is not None:
            self.saved_corridor_end_y += dy
        rospy.loginfo("[会话] ✓ 共享布局坐标已平移 (%.2f, %.2f)", dx, dy)

    def _floor_elevator_center(self, floor):
        """
        目标楼层地图帧中的电梯中心坐标（用于 deserialize 初始位姿）。

        已访问过的楼层记录于 floor_elevator_center（实测值）；
        未访问楼层返回 (0,0)（实际不会用于 deserialize，因为没有会话文件）。
        """
        return self.floor_elevator_center.get(floor, (0.0, 0.0))

    def _shift_shared_to_floor(self, target_floor):
        """
        把共享布局坐标从"当前楼层帧"平移到"目标楼层帧"。

        锚点 = 两帧中的电梯中心之差。电梯中心记录于 floor_elevator_center：
          - 启动楼层：实测 (ex, ey)
          - 首次访问的楼层：重启 SLAM 后实测机器人在新地图中的位置
          - 返回的楼层：deserialize 后即记录值
        平移后更新 self.elevator_center 为目标帧值。
        """
        cur_ec = self.floor_elevator_center.get(self.current_floor)
        tgt_ec = self.floor_elevator_center.get(target_floor)
        if cur_ec is None and self.elevator_center:
            cur_ec = tuple(self.elevator_center)
        if tgt_ec is None and self.elevator_center:
            tgt_ec = tuple(self.elevator_center)
        if cur_ec is None or tgt_ec is None:
            rospy.logwarn("[会话] 缺少电梯中心锚点，跳过共享坐标平移")
            return
        dx = tgt_ec[0] - cur_ec[0]
        dy = tgt_ec[1] - cur_ec[1]
        if abs(dx) > 0.01 or abs(dy) > 0.01:
            self._translate_shared_features(dx, dy)
        else:
            rospy.loginfo("[会话] 两帧电梯中心重合，无需平移 (%.2f, %.2f)",
                          dx, dy)
        self.elevator_center = tgt_ec

    def _switch_floor_session(self, target_floor, init_x=0.0, init_y=0.0,
                              init_yaw=0.0):
        """
        切换到目标楼层的 SLAM 会话（电梯内、开门前调用）。

        - 有会话文件 → deserialize 完整恢复该楼层
        - 无会话文件 → 重启 slam_toolbox 节点全新建图，实测新地图中
          机器人位置作为该楼层电梯中心（机器人此时物理上在电梯中心）
        切换后按"两帧电梯中心之差"平移共享布局坐标，清除代价地图，
        并复用共享楼层布局坐标。

        注意：不依赖 map_start_pose 假设（实测优先），若 SLAM 恰好把
        机器人放到 (0,0.5)，实测值即 (0,0.5)，行为自洽。
        """
        # 记录当前楼层地图帧中的电梯中心（供以后返回该楼层时 deserialize）
        if self.elevator_center:
            self.floor_elevator_center[self.current_floor] = \
                tuple(self.elevator_center)

        if self._has_floor_session(target_floor):
            ok = self._deserialize_floor(target_floor, init_x, init_y, init_yaw)
        else:
            # 首次访问：重启 SLAM 节点，全新建图
            ok = self._restart_slam()
            if ok:
                rospy.loginfo(
                    "[会话] 楼层 %d 首次访问，开始全新建图", target_floor)
                # 实测机器人在新地图中的位置（= 电梯中心在该楼层帧的坐标）
                rx, ry, _ = self._get_robot_pose()
                for _ in range(5):
                    if rx is not None:
                        break
                    rospy.sleep(0.5)
                    rx, ry, _ = self._get_robot_pose()
                if rx is not None:
                    self.floor_elevator_center[target_floor] = (rx, ry)
                    rospy.loginfo(
                        "[会话] 楼层 %d 电梯中心(实测) = (%.2f, %.2f)",
                        target_floor, rx, ry)
                else:
                    rospy.logwarn(
                        "[会话] 无法获取新地图中机器人位姿，"
                        "该楼层电梯中心记为 (0,0)")
                    self.floor_elevator_center[target_floor] = (0.0, 0.0)

        if ok:
            self._shift_shared_to_floor(target_floor)

        self.current_floor = target_floor
        # 清空本地缓存地图，等待新楼层 /map 更新
        self.map_data = None
        # 清除旧楼层代价地图
        self._clear_costmaps()
        # 复用共享布局
        self._restore_shared_features()
        return ok

    def _take_elevator(self, elevator_center, from_floor, to_floor):
        """
        完整乘梯流程。

        电梯在 x 方向长度固定 2.4m，门在电梯靠近走廊一侧 (ex - 1.2)。

        进入电梯：
          1. 导航到门前 0.3m (ex - 1.2 - 0.3, ey)
          2. 旋转正朝 +x 方向
          3. 呼叫电梯 + 开门（阻塞直到门完全打开）
          4. 微调朝向
          5. 前进 1m 进入电梯
          6. 导航到电梯中心

        离开电梯：
          7. 朝向 -x 方向
          8. 开门（阻塞直到完全打开）
          9. 前进离开电梯
          10. 导航到走廊
        """
        ex, ey = elevator_center
        from_door = f'elevator_floor_{from_floor}'
        to_door = f'elevator_floor_{to_floor}'

        # 电梯 x 方向长 2.4m，门面在 ex - 1.2
        door_x = ex - 1.2
        approach_x = door_x - 0.3  # 门前 0.3m

        rospy.loginfo(
            "[电梯] ====== 乘梯流程: %d 楼 → %d 楼 ======",
            from_floor, to_floor)
        rospy.loginfo(
            "[电梯] 电梯中心=(%.2f, %.2f), 门面 x=%.2f, 接近点 x=%.2f",
            ex, ey, door_x, approach_x)

        # ════════════════════════════════════════
        #  进入电梯
        # ════════════════════════════════════════

        # ── 1. 导航到门前 0.3m ──
        rospy.loginfo("[电梯] ① 导航到门前 (%.2f, %.2f)...", approach_x, ey)
        if not self._navigate_to(approach_x, ey, timeout=30.0):
            rospy.logwarn("[电梯] 导航到门前失败，在当前位置继续")

        # 等待 Gazebo 物理稳定（防止轮子惯性导致位姿漂移）
        rospy.sleep(0.5)

        # ── 2. 旋转正朝 +x 方向 ──
        rospy.loginfo("[电梯] ② 旋转正朝 +x 方向...")
        rx, ry, _ = self._get_robot_pose()
        if rx is not None:
            self._rotate_to_face(rx + 1.0, ry)

        # ── 3. 呼叫电梯 + 开门（阻塞直到门完全打开）──
        rospy.loginfo("[电梯] ③ 呼叫电梯到 %d 楼...", from_floor)
        if not self._call_elevator('elevator_main', from_floor, False):
            rospy.logerr("[电梯] 呼叫电梯失败")
            return

        rospy.loginfo("[电梯] ④ 开门 %s（等待完全打开）...", from_door)
        if not self._set_door(from_door, True):
            rospy.logerr("[电梯] 开门失败")
            return
        rospy.loginfo("[电梯] ✓ 门已完全打开")

        # ── 4. 微调朝向（开门过程可能有轻微偏移）──
        rospy.loginfo("[电梯] ⑤ 微调朝向 +x...")
        rx2, ry2, _ = self._get_robot_pose()
        if rx2 is not None:
            self._rotate_to_face(rx2 + 1.0, ry2)

        # ── 5. 前进 1m 进入电梯 ──
        rospy.loginfo("[电梯] ⑥ 前进 1m 进入电梯...")
        self._move_forward(1.0, speed=0.12)

        # ── 6. 导航到电梯中心 ──
        rospy.loginfo("[电梯] ⑦ 导航到电梯中心 (%.2f, %.2f)...", ex, ey)
        if not self._navigate_to(ex, ey, timeout=30.0):
            rospy.logwarn("[电梯] 导航到中心失败，已在电梯内")

        # ── 8. 关门 ──
        rospy.loginfo("[电梯] ⑧ 关门 %s...", from_door)
        self._set_door(from_door, False)
        rospy.sleep(1.0)

        # ── 8b. 保存当前楼层 SLAM 会话（乘梯前）──
        rospy.loginfo("[会话] 乘梯前保存楼层 %d 会话...", from_floor)
        self._serialize_floor(from_floor)

        # ════════════════════════════════════════
        #  电梯运行
        # ════════════════════════════════════════

        rospy.loginfo("[电梯] ⑨ 呼叫电梯到 %d 楼...", to_floor)
        if not self._call_elevator('elevator_main', to_floor, False):
            rospy.logerr("[电梯] 呼叫电梯失败")
            return

        rospy.loginfo("[电梯] 运行中...")
        rospy.sleep(2.0)  # 模拟电梯运行时间

        # ── 9b. 切换目标楼层 SLAM 会话（开门前，电梯内）──
        # 机器人在电梯中心，朝向 +x（进入电梯时的朝向）；
        # 初始位姿用目标楼层地图帧中的电梯中心坐标
        t_ex, t_ey = self._floor_elevator_center(to_floor)
        rospy.loginfo(
            "[会话] 开门前切换到楼层 %d 会话 (初始位姿 %.1f,%.1f,0°)...",
            to_floor, t_ex, t_ey)
        self._switch_floor_session(
            to_floor, init_x=t_ex, init_y=t_ey, init_yaw=0.0)

        # ── 10. 开门 ──
        rospy.loginfo("[电梯] ⑩ 开门 %s（等待完全打开）...", to_door)
        if not self._set_door(to_door, True):
            rospy.logerr("[电梯] 开门失败")
            return
        rospy.loginfo("[电梯] ✓ 门已完全打开")

        # ════════════════════════════════════════
        #  离开电梯
        # ════════════════════════════════════════

        # ── 11. 朝向 -x 方向（面向走廊）──
        rospy.loginfo("[电梯] ⑪ 朝向走廊 (-x 方向)...")
        rx3, ry3, _ = self._get_robot_pose()
        if rx3 is not None:
            self._rotate_to_face(rx3 - 1.0, ry3)

        # ── 12. 前进离开电梯 ──
        rospy.loginfo("[电梯] ⑫ 前进 2.5m 离开电梯...")
        self._move_forward(1.5, speed=0.12)

        # ── 13. 导航到走廊 ──
        exit_y = ey + 0.0
        rospy.loginfo("[电梯] ⑬ 导航到走廊 (0.00, %.2f)...", exit_y)
        if not self._navigate_to(0.0, exit_y, timeout=60.0):
            rospy.logwarn("[电梯] 导航到走廊失败")

        # ── 14. 关门 ──
        rospy.loginfo("[电梯] ⑭ 关门 %s...", to_door)
        self._set_door(to_door, False)

        rospy.loginfo(
            "[电梯] ====== 乘梯完成: 已到达 %d 楼 ======", to_floor)

    def _patch_map_at_door(self, elevator_center, floor):
        """
        手动更新地图 + 清除 move_base 代价地图，让导航能穿过电梯门。

        问题：电梯门打开后，SLAM 地图和 move_base 代价地图都不会自动更新，
        门区域仍显示为障碍物，导致 move_base 无法规划路径。

        解决：
          1. 将本地地图中电梯门到走廊的区域标记为自由空间 (0)
          2. 调用 /move_base/clear_costmaps 清除代价地图
          3. move_base 重建代价地图时，局部代价地图会用 LiDAR 数据
             （已能看到开着的门），全局代价地图会从 /map 重建

        参数:
            elevator_center: (ex, ey) 电梯中心坐标
            floor: 楼层（0-based）
        """
        if self.map_data is None:
            rospy.logwarn("[地图补丁] 地图未就绪，跳过")
            return

        ex, ey = elevator_center
        res = self.map_resolution

        # ── 电梯门在电梯和走廊之间 ──
        # 电梯中心在走廊右侧 (ex > 0)，门朝向走廊 (x=0 方向)
        # 补丁区域：从走廊边缘到电梯中心，y 方向覆盖门宽
        corridor_edge = 1.1   # 走廊右边缘 x
        door_half_width = 1.0  # 门半宽（y 方向）

        patch_x_min = corridor_edge
        patch_x_max = ex + 0.5  # 稍微超过电梯中心
        patch_y_min = ey - door_half_width
        patch_y_max = ey + door_half_width

        # 转换为栅格坐标
        col_min = max(0, int((patch_x_min - self.map_origin_x) / res))
        col_max = min(self.map_width, int((patch_x_max - self.map_origin_x) / res))
        row_min = max(0, int((patch_y_min - self.map_origin_y) / res))
        row_max = min(self.map_height, int((patch_y_max - self.map_origin_y) / res))

        # 标记为自由空间
        self.map_data[row_min:row_max, col_min:col_max] = 0

        rospy.loginfo(
            "[地图补丁] ✓ 楼层 %d: 标记电梯门区域为自由空间 "
            "x[%.2f, %.2f] y[%.2f, %.2f] → 栅格 col[%d:%d] row[%d:%d]",
            floor, patch_x_min, patch_x_max, patch_y_min, patch_y_max,
            col_min, col_max, row_min, row_max)

        # ── 清除 move_base 代价地图 ──
        rospy.loginfo("[地图补丁] 清除 move_base 代价地图...")
        try:
            clear_proxy = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_proxy()
            rospy.loginfo("[地图补丁] ✓ 代价地图已清除")
        except rospy.ServiceException as e:
            rospy.logwarn("[地图补丁] 清除代价地图失败: %s", e)

    # ────────────── 状态持久化 ──────────────
    def _save_state(self, entrance_y=None, corridor_end_y=None):
        """保存走廊检测结果到 JSON 文件"""
        if not self.save_state_path:
            return
        state = {
            'entrance_y': entrance_y,
            'corridor_end_y': corridor_end_y,
            'door_positions': self.door_positions,
            'forbidden_zones': self.forbidden_zones,
            'stair_center': self.stair_center,
            'elevator_center': self.elevator_center,
        }
        try:
            with open(self.save_state_path, 'w') as f:
                json.dump(state, f, indent=2)
            rospy.loginfo("[状态] ✓ 已保存到 %s", self.save_state_path)
            rospy.loginfo("[状态]   入口 y=%.2f  尽头 y=%.2f  门=%d  禁区=%d",
                          entrance_y or -1, corridor_end_y or -1,
                          len(self.door_positions), len(self.forbidden_zones))
        except IOError as e:
            rospy.logwarn("[状态] 保存失败: %s", e)

    def _load_state(self):
        """从 JSON 文件加载，成功返回 True"""
        if not self.load_state_path or not os.path.exists(self.load_state_path):
            return False
        try:
            with open(self.load_state_path, 'r') as f:
                state = json.load(f)
            self.saved_entrance_y = state.get('entrance_y')
            self.saved_corridor_end_y = state.get('corridor_end_y')
            self.door_positions = state.get('door_positions', [])
            self.forbidden_zones = state.get('forbidden_zones', [])
            self.stair_center = state.get('stair_center')
            self.elevator_center = state.get('elevator_center')
            rospy.loginfo("[状态] ✓ 已加载: 入口=%.2f 尽头=%.2f 门=%d 禁区=%d",
                          self.saved_entrance_y or -1,
                          self.saved_corridor_end_y or -1,
                          len(self.door_positions), len(self.forbidden_zones))
            return True
        except (IOError, json.JSONDecodeError) as e:
            rospy.logwarn("[状态] 加载失败: %s", e)
            return False

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

    # ────────────── 建筑边界测量 ──────────────
    def _measure_building_bounds(self):
        """
        测量建筑 x 方向边界。

        在 y=0 ~ y=2m 范围内，逐行扫描 x 方向自由空间，
        遇到障碍物(100)或未知空间(-1)停止。

        输出每行的左边界和右边界到日志，用于调试。
        """
        if self.map_data is None:
            rospy.logwarn("[建筑边界] 地图未就绪")
            return

        res = self.map_resolution
        h, w = self.map_data.shape

        y_min = 0.0
        y_max = 2.0
        step = 0.2

        rospy.loginfo(
            "[建筑边界] ====== 建筑 x 方向边界测量 (y=%.1f→%.1f) ======",
            y_min, y_max)
        rospy.loginfo(
            "[建筑边界] 地图范围: x[%.2f, %.2f] y[%.2f, %.2f] "
            "size=%dx%d res=%.3f",
            self.map_origin_x, self.map_origin_x + w * res,
            self.map_origin_y, self.map_origin_y + h * res,
            w, h, res)

        center_col = int((0.0 - self.map_origin_x) / res)
        max_scan = 20.0 / res

        results = []
        y = y_min
        while y <= y_max:
            row = int((y - self.map_origin_y) / res)
            if row < 0 or row >= h:
                y += step
                continue

            # ── 向左扫描（-x方向）──
            left_bound = None
            for dc in range(1, int(max_scan)):
                col = center_col - dc
                if col < 0 or col >= w:
                    left_bound = -(dc * res)
                    break
                val = self.map_data[row, col]
                if val == 100 or val == -1:
                    left_bound = -(dc * res)
                    break

            # ── 向右扫描（+x方向）──
            right_bound = None
            for dc in range(1, int(max_scan)):
                col = center_col + dc
                if col < 0 or col >= w:
                    right_bound = dc * res
                    break
                val = self.map_data[row, col]
                if val == 100 or val == -1:
                    right_bound = dc * res
                    break

            if left_bound is not None and right_bound is not None:
                total_width = right_bound - left_bound
                results.append((y, left_bound, right_bound, total_width))
                rospy.loginfo(
                    "[建筑边界] y=%.2f  左=%.2f  右=%.2f  宽度=%.2fm",
                    y, left_bound, right_bound, total_width)
            else:
                rospy.loginfo("[建筑边界] y=%.2f  测量失败", y)

            y += step

        # ── 汇总 ──
        if results:
            widths = [r[3] for r in results]
            lefts = [r[1] for r in results]
            rights = [r[2] for r in results]
            # 保存到 self，供 _estimate_room_positions 使用
            self.building_left = min(lefts)
            self.building_right = max(rights)
            self.building_width = (self.building_right - self.building_left)
            rospy.loginfo("[建筑边界] ====== 汇总 ======")
            rospy.loginfo("[建筑边界] 左边界范围: [%.2f, %.2f]",
                          min(lefts), max(lefts))
            rospy.loginfo("[建筑边界] 右边界范围: [%.2f, %.2f]",
                          min(rights), max(rights))
            rospy.loginfo("[建筑边界] 宽度范围: [%.2f, %.2f]",
                          min(widths), max(widths))
            rospy.loginfo("[建筑边界] 平均宽度: %.2fm",
                          sum(widths) / len(widths))
            rospy.loginfo("[建筑边界] 建筑左边界 ≈ %.2f, 右边界 ≈ %.2f",
                          self.building_left, self.building_right)
        else:
            self.building_left = None
            self.building_right = None
            self.building_width = None
            rospy.logwarn("[建筑边界] 未获取到有效测量数据")

    # ────────────── 房间位置估计 ──────────────
    def _estimate_room_positions(self, entrance_y, corridor_end_y):
        """
        从已测量的 door_positions 和建筑边界估计每个房间的位置。

        推导逻辑：
          1. 将 door_positions 按 y 配对（左+右在同一 y 为一对）
          2. 走廊墙 x 位置 = 从门扫描数据中取非门区域的墙距中位数
          3. 房间 y 范围 = 相邻门对之间的 y 区间
          4. 房间 x 范围 = 建筑边界 ~ 走廊墙

        输出每个房间的估计位置到日志。
        """
        if not self.door_positions:
            rospy.loginfo("[房间估计] 无房门数据，跳过房间位置估计")
            return

        if self.building_left is None or self.building_right is None:
            rospy.logwarn("[房间估计] 建筑边界未测量，跳过")
            return

        # ── 1. 将 door_positions 按 y 配对 ──
        # 左右门 y 值可能不完全相等（差 0.1m 左右），按容差分组
        y_tolerance = 0.5  # 左右门 y 差 < 0.5m 即视为同一对
        doors_sorted = sorted(self.door_positions, key=lambda d: d[0], reverse=True)

        pairs = []  # [(pair_y, 'left', 'right'), ...]
        used = set()
        for i, (yi, side_i) in enumerate(doors_sorted):
            if i in used:
                continue
            for j, (yj, side_j) in enumerate(doors_sorted):
                if j <= i or j in used:
                    continue
                if side_i != side_j and abs(yi - yj) < y_tolerance:
                    pair_y = (yi + yj) / 2.0
                    pairs.append(pair_y)
                    used.add(i)
                    used.add(j)
                    break

        pairs.sort(reverse=True)

        if not pairs:
            rospy.logwarn("[房间估计] 未找到完整的左右门配对")
            return

        corridor_half_width = 1.1  # 走廊宽度 2.2m 的一半

        rospy.loginfo(
            "[房间估计] ====== 房间位置估计 (共 %d 对, 建筑 x[%.1f, %.1f]) ======",
            len(pairs), self.building_left, self.building_right)
        rospy.loginfo("[房间估计] 走廊入口 y=%.2f, 走廊尽头 y=%.2f",
                      entrance_y, corridor_end_y)

        # ── 2. 等分走廊：每对房间平分走廊长度 ──
        # 走廊 y 范围 = [entrance_y, corridor_end_y]
        # 每段长度 = 总长 / 对数，门位于段中心
        segment_count = len(pairs)
        segment_length = (corridor_end_y - entrance_y) / segment_count

        for i, pair_y in enumerate(pairs):
            # i=0 是顶部对（靠近走廊尽头），i=1 是底部对（靠近入口）
            y_top = corridor_end_y - i * segment_length
            y_bottom = corridor_end_y - (i + 1) * segment_length

            # x 范围
            left_x_min = self.building_left
            left_x_max = -corridor_half_width
            right_x_min = corridor_half_width
            right_x_max = self.building_right

            # 房间中心点
            left_center_x = (left_x_min + left_x_max) / 2.0
            right_center_x = (right_x_min + right_x_max) / 2.0
            room_center_y = (y_bottom + y_top) / 2.0

            left_room_width = left_x_max - left_x_min
            right_room_width = right_x_max - right_x_min
            room_length = y_top - y_bottom

            rospy.loginfo(
                "[房间估计] ─── 第 %d 对 (门 y=%.2f, 段 [%.1f, %.1f]) ───",
                i + 1, pair_y, y_bottom, y_top)
            rospy.loginfo(
                "[房间估计]   左侧房间: x[%.1f, %.1f] y[%.1f, %.1f] "
                "→ 中心 (%.1f, %.1f)  (%.1f×%.1f)",
                left_x_min, left_x_max, y_bottom, y_top,
                left_center_x, room_center_y,
                left_room_width, room_length)
            rospy.loginfo(
                "[房间估计]   右侧房间: x[%.1f, %.1f] y[%.1f, %.1f] "
                "→ 中心 (%.1f, %.1f)  (%.1f×%.1f)",
                right_x_min, right_x_max, y_bottom, y_top,
                right_center_x, room_center_y,
                right_room_width, room_length)

            # 保存到 self，供后续房间扫描导航使用
            self.room_positions.append(
                (left_center_x, room_center_y, 'left', y_bottom, y_top))
            self.room_positions.append(
                (right_center_x, room_center_y, 'right', y_bottom, y_top))

        # 按 y 降序排列（从走廊尽头向入口）
        self.room_positions.sort(key=lambda r: r[1], reverse=True)
        rospy.loginfo("[房间估计] ====== 估计完毕 ======")

    # ────────────── 房间内前沿探索 ──────────────
    def _explore_room(self, bounds, max_iterations=20):
        """
        在指定房间边界内运行前沿探索循环。

        每次迭代：
          1. 在 bounds 范围内检测前沿
          2. 选择评分最高的前沿目标
          3. 导航到目标并旋转扫描
          4. 地图更新后重新检测前沿
          5. 无前沿时结束（房间已探索完成）

        参数:
            bounds:         (x_min, x_max, y_min, y_max) 房间边界
            max_iterations: 最大迭代次数（防止无限循环）

        返回: True 探索完成, False 达到最大迭代
        """
        x_min, x_max, y_min, y_max = bounds
        rospy.loginfo(
            "[房间探索] 开始前沿探索 x[%.1f, %.1f] y[%.1f, %.1f]",
            x_min, x_max, y_min, y_max)

        for iteration in range(max_iterations):
            if rospy.is_shutdown():
                return False

            # 在房间范围内检测前沿
            frontiers = self.detect_frontiers(bounds=bounds)
            rospy.loginfo(
                "[房间探索] 迭代 %d: 检测到 %d 个有效前沿簇",
                iteration + 1, len(frontiers))

            if not frontiers:
                rospy.loginfo("[房间探索] ✓ 房间内无前沿，探索完成")
                return True

            # 选择最优目标
            result = self.select_best_goal(frontiers)
            if result is None:
                rospy.loginfo("[房间探索] 所有前沿被过滤，结束探索")
                return True

            gx, gy, score = result
            rospy.loginfo(
                "[房间探索] ★ 选中目标 (%.2f, %.2f) 评分=%.3f",
                gx, gy, score)

            # 发布可视化标记
            self.publish_frontier_markers(frontiers, selected_goal=(gx, gy))
            self.publish_goal_marker(gx, gy)
            self.blacklist.append((gx, gy))

            # 导航到目标
            if not self._navigate_to(gx, gy, timeout=self.nav_timeout):
                rospy.logwarn("[房间探索] 导航失败，尝试下一个前沿")
                continue

            # 到达后旋转扫描
            self._rotate_360()

        rospy.logwarn("[房间探索] 达到最大迭代 %d 次", max_iterations)
        return False

    # ────────────── 逐门房间扫描 ──────────────
    def _scan_rooms_sequentially(self):
        """
        逐门扫描每个房间：导航到门口 → 进入 → 扫描 → 探索 → 返回门口。

        顺序：y 降序（从走廊尽头向入口），先左后右。

        流程：
          1. 导航到走廊门前 (0.0, door_y)
          2. 导航进入房间 2m
          3. 摆头扫描
          4. 房间内前沿探索直到无前沿
          5. 返回当前房间门口 (0.0, door_y)
          6. 下一个房间

        全部完成后：
          导航至最后一扇门门口 (0.0, door_y)
          导航至电梯位置 (0.0, elevator_y)
        """
        if not self.room_positions:
            rospy.loginfo("[房间扫描] 无房间可扫描")
            return

        # ── 按 y 降序 + 先左后右排序 ──
        # room_positions: [(center_x, center_y, side, y_bottom, y_top), ...]
        rooms_sorted = sorted(
            self.room_positions,
            key=lambda r: (-r[1], 0 if r[2] == 'left' else 1))

        corridor_half_width = 1.1
        entry_depth = 2.0  # 进入房间的深度

        rospy.loginfo(
            "[房间扫描] ====== 开始逐门扫描 (共 %d 间) ======",
            len(rooms_sorted))

        last_door_y = None

        for idx, (cx, cy, side, y_bottom, y_top) in enumerate(rooms_sorted):
            door_y = (y_bottom + y_top) / 2.0
            last_door_y = door_y

            # 房间前沿探索边界
            if side == 'left':
                room_bounds = (
                    self.building_left, -corridor_half_width,
                    y_bottom, y_top)
            else:
                room_bounds = (
                    corridor_half_width, self.building_right,
                    y_bottom, y_top)

            rospy.loginfo(
                "[房间扫描] ─── 第 %d/%d 间: %s 房间 "
                "门 y=%.2f, 段 y[%.1f, %.1f] ───",
                idx + 1, len(rooms_sorted), side,
                door_y, y_bottom, y_top)

            # ── 发布房间标记 ──
            self._publish_current_room_marker(
                idx, room_bounds, side, len(rooms_sorted))

            # ── 1. 导航到走廊门前 ──
            rospy.loginfo("[房间扫描] ① 导航到门口 (0.00, %.2f)", door_y)
            if not self._navigate_to(0.0, door_y, timeout=60.0):
                rospy.logwarn("[房间扫描] 导航到门口失败，跳过此房间")
                continue

            # ── 2. 进入房间 entry_depth 米 ──
            if side == 'left':
                entry_x = -corridor_half_width - entry_depth
            else:
                entry_x = corridor_half_width + entry_depth

            rospy.loginfo(
                "[房间扫描] ② 进入房间 %.1fm 到 (%.2f, %.2f)",
                entry_depth, entry_x, door_y)
            if not self._navigate_to(entry_x, door_y, timeout=30.0):
                rospy.logwarn("[房间扫描] 进入房间失败，跳过")
                continue

            # ── 3. 摆头扫描 ──
            rospy.loginfo("[房间扫描] ③ 摆头扫描...")
            # self._wiggle_scan()

            # ── 4. 房间内前沿探索 ──
            rospy.loginfo("[房间扫描] ④ 房间内前沿探索...")
            # self._explore_room(room_bounds)

            # ── 5. 返回当前房间门口 ──
            rospy.loginfo("[房间扫描] ⑤ 返回门口 (0.00, %.2f)", door_y)
            self._navigate_to(0.0, door_y, timeout=60.0)

        # ── 全部完成：导航至最后一扇门门口，再导航至电梯，然后乘梯上楼 ──
        if self.elevator_center is not None and last_door_y is not None:
            ex, ey = self.elevator_center
            rospy.loginfo(
                "[房间扫描] ====== 全部房间扫描完毕 ======")
            rospy.loginfo(
                "[房间扫描] 导航至最后一扇门门口 (0.00, %.2f)", last_door_y)
            self._navigate_to(0.0, last_door_y, timeout=60.0)
            rospy.loginfo(
                "[房间扫描] 导航至电梯 (0.00, %.2f)", ey)
            self._navigate_to(0.0, ey, timeout=60.0)

            # ── 乘梯到下一层 ──
            rospy.loginfo("[房间扫描] 开始乘梯流程...")
            self._take_elevator(self.elevator_center, 0, 1)
        else:
            rospy.loginfo(
                "[房间扫描] ====== 全部房间扫描完毕 "
                "(电梯位置未知，停在当前位置) ======")

        # 清除房间标记
        self._clear_room_markers()

    # ────────────── 启动前初始化扫描 ──────────────
    def _initial_scan(self):
        """
        启动前沿探索前的初始化扫描流程：
          1. 原地旋转 360°（出生点周围）
          2. 前进一段距离（推开视野）
          3. 原地旋转 360°（扫描深处）
          4. 检测走廊入口，前进到入口前 0.5m
          5. 再次旋转 360°（近距离扫描走廊入口区域）
          6. 走廊检测、房间估计、逐门扫描
          7. 结束
        各步骤的行为由 ROS 参数控制，可在运行时或 launch 文件中调节。
        """
        rospy.loginfo("[前沿探索] 等待首帧地图...")
        while not rospy.is_shutdown() and self.map_data is None:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return

        # ── 从文件加载状态 → 跳过初始扫描 ──
        if self._load_state():
            rospy.loginfo("[前沿探索] ⏩ 跳过初始扫描，恢复已保存特征")
            self._mark_forbidden_zones()
            self._publish_forbidden_zone_markers()
            if self.saved_entrance_y is not None:
                self._publish_entrance_marker(0.0, self.saved_entrance_y)
            if self.saved_corridor_end_y is not None:
                self._publish_entrance_marker(0.0, self.saved_corridor_end_y)
            if self.door_positions:
                self._publish_door_markers(self.door_positions)
            if self.stair_center:
                sx, sy = self.stair_center
                self._publish_block_marker(sx, sy, 'stairs',
                                           ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.7))
            if self.elevator_center:
                ex, ey = self.elevator_center
                self._publish_block_marker(ex, ey, 'elevator',
                                           ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.7))
            # ── 导航到走廊末1/4处 ──
            if self.saved_entrance_y is not None and self.saved_corridor_end_y is not None:
                corridor_len = self.saved_corridor_end_y - self.saved_entrance_y
                target_y = self.saved_corridor_end_y - corridor_len / 4.0
                rospy.loginfo("[前沿探索] 🚀 导航到走廊末1/4处 (0.00, %.2f) corridor_len=%.2fm",
                              target_y, corridor_len)
                self._navigate_to(0.0, target_y, timeout=60.0)
            if self.init_pause_enabled:
                rospy.loginfo("[前沿探索] ⏸ 调试暂停...")
                rx, ry, _ = self._get_robot_pose()
                if rx is not None:
                    rospy.loginfo("[位置] ★ 当前机器人位姿: "
                                  "x=%.2f y=%.2f", rx, ry)
                while not rospy.is_shutdown():
                    self.cmd_vel_pub.publish(Twist())
                    rospy.sleep(0.5)
                return
            rx, ry, _ = self._get_robot_pose()
            if rx is not None:
                rospy.loginfo("[位置] ★ 当前机器人位姿: x=%.2f y=%.2f", rx, ry)
            rospy.loginfo("[前沿探索] ✓ 特征恢复完成，开始前沿探索")
            return

        # ── 第一阶段：出生点旋转扫描 ──
        if self.init_rotate_enabled:
            rospy.loginfo("[前沿探索] 🔄 第一阶段：出生点旋转扫描...")
            self._rotate_360()

        # ── 建筑边界测量（第一阶段扫描后，y=0~2m 范围测量 x 方向自由空间）──
        # 用于调试验证建筑尺寸与生成参数是否一致
        rospy.loginfo("[前沿探索] 📐 测量建筑 x 方向边界...")
        self._measure_building_bounds()

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
        if self.init_rotate_enabled:
            rospy.loginfo("[前沿探索] 🔄 第三阶段：深入后30°旋转扫描...")
            self._wiggle_scan()

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
                            self._wiggle_scan()
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
                    # 复检尽头：从新位置再次确认尽头位置
                    re_end_y, re_def = self._detect_corridor_end(entrance_y)
                    if re_def and re_end_y is not None:
                        delta = abs(re_end_y - corridor_end_y)
                        if delta > 0.5:
                            rospy.loginfo(
                                "[尽头检测] 复检尽头偏差 %.2fm > 0.5m！"
                                "更新为 y=%.2f", delta, re_end_y)
                            corridor_end_y = re_end_y
                            self._publish_entrance_marker(0.0, corridor_end_y)
                        else:
                            rospy.loginfo(
                                "[尽头检测] 复检尽头一致 (偏差 %.2fm)", delta)
                    else:
                        rospy.loginfo("[尽头检测] 复检未确认尽头，保留原值")
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

            # ── 房间位置估计（从房门位置和建筑边界推导）──
            rospy.loginfo("[前沿探索] 📐 估计房间位置...")
            self._estimate_room_positions(entrance_y, corridor_end_y)

            # ── 逐门房间扫描 ──
            if self.room_positions:
                rospy.loginfo("[前沿探索] 🚪 开始逐门房间扫描...")
                self._scan_rooms_sequentially()

        # ── 保存状态（如果配置了路径）──
        self._save_state(
            entrance_y=entrance_y,
            corridor_end_y=corridor_end_y)

        # ── 打印当前机器人位置（方便复制到 launch 文件）──
        rx, ry, _ = self._get_robot_pose()
        if rx is not None:
            rospy.loginfo("[位置] ★ 当前机器人位姿: x=%.2f y=%.2f", rx, ry)

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
        房间扫描主流程：
          1. 初始扫描（旋转、前进、走廊检测、房间估计、逐门扫描）
          2. 完成后退出

        调试模式（debug_elevator=True）：
          1. 加载走廊状态 JSON
          2. 导航到电梯附近
          3. 执行乘梯流程
        """
        # ── 电梯调试模式 ──
        if self.debug_elevator:
            self._run_elevator_debug()
            return

        # ── 正常探索模式 ──
        rate = rospy.Rate(self.detection_freq)
        rospy.loginfo("[前沿探索] ====== 开始房间扫描 ======")

        self._initial_scan()

        rospy.loginfo("[前沿探索] ====== 房间扫描完成，节点退出 ======")

    def _run_elevator_debug(self):
        """电梯调试模式：跳过初始扫描，直接加载状态并执行乘梯往返测试"""
        rospy.loginfo("[前沿探索] ====== 电梯往返调试模式 ======")

        # ── 1. 加载状态 ──
        if not self._load_state():
            rospy.logerr("[电梯调试] 无法加载状态文件，退出")
            return

        if self.elevator_center is None:
            rospy.logerr("[电梯调试] 状态文件中无电梯中心，退出")
            return

        ex, ey = self.elevator_center
        rospy.loginfo("[电梯调试] 电梯中心: (%.2f, %.2f)", ex, ey)

        # ── 2. 等待地图 ──
        rospy.loginfo("[电梯调试] 等待首帧地图...")
        while not rospy.is_shutdown() and self.map_data is None:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return

        # ── 3. 导航到电梯附近 ──
        rospy.loginfo("[电梯调试] 导航到电梯附近 (0.00, %.2f)...", ey)
        if not self._navigate_to(0.0, ey, timeout=60.0):
            rospy.logerr("[电梯调试] 导航到电梯附近失败")
            return

        # ════════════════════════════════════════
        #  第一段：Floor 0 → Floor 1
        # ════════════════════════════════════════
        rospy.loginfo("[电梯调试] ====== 第一段: Floor 0 → Floor 1 ======")
        self._take_elevator(self.elevator_center, 0, 1)

        # ── 在 floor 1 停留观察 ──
        rospy.loginfo("[电梯调试] Floor 1 停留 10s，观察地图...")
        rospy.sleep(0.01)

        # ── 导航回电梯（使用平移后的 elevator_center）──
        ex1, ey1 = self.elevator_center
        rospy.loginfo(
            "[电梯调试] 导航回电梯 (0.00, %.2f) [floor 1 坐标]...", ey1)
        if not self._navigate_to(0.0, ey1, timeout=60.0):
            rospy.logerr("[电梯调试] 导航回电梯失败")
            return

        # ════════════════════════════════════════
        #  第二段：Floor 1 → Floor 0
        # ════════════════════════════════════════
        rospy.loginfo("[电梯调试] ====== 第二段: Floor 1 → Floor 0 ======")
        self._take_elevator(self.elevator_center, 1, 0)

        # ── 在 floor 0 停留观察 ──
        rospy.loginfo("[电梯调试] Floor 0 停留 10s，观察地图是否恢复...")
        rospy.sleep(0.01)

        rospy.loginfo("[前沿探索] ====== 电梯往返调试完成 ======")


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
