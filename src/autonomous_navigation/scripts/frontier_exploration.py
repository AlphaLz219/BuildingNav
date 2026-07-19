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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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
        # 连续无前沿次数阈值，超过则判定探索完成
        self.max_no_frontier = rospy.get_param('~max_no_frontier_count', 5)
        # 导航超时（秒），超过则取消当前目标
        self.nav_timeout = rospy.get_param('~navigation_timeout', 120.0)
        # 黑名单半径（米），导航失败的目标周围不再重复尝试
        self.blacklist_radius = rospy.get_param('~blacklist_radius', 1.5)
        # 距机器人太近的前沿不选（已在附近，无需导航）
        self.min_goal_distance = rospy.get_param('~min_goal_distance', 0.8)

    def _init_state(self):
        """初始化内部状态变量"""
        # 地图相关
        self.map_data = None          # 2D numpy 数组 (height × width)
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # 探索状态
        self.is_navigating = False     # 是否正在导航中
        self.no_frontier_count = 0     # 连续无前沿计数
        self.exploration_complete = False
        self.blacklist = []            # 导航失败的目标列表 [(x, y), ...]
        self.goal_start_time = None    # 当前导航开始时间（用于超时检测）

    def _init_ros_interface(self):
        """初始化 ROS 接口：TF、ActionClient、发布器、订阅器"""
        # TF 监听器 —— 查询机器人位姿
        self.tf_listener = tf.TransformListener()

        # move_base ActionClient —— 发送导航目标并获取结果
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[前沿探索] 等待 move_base 服务启动...")
        self.ac.wait_for_server()
        rospy.loginfo("[前沿探索] move_base 已连接")

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
        检测前沿点并聚类
        
        前沿定义：自由空间栅格，且至少有一个 4-邻域栅格是未知空间
        即：机器人可以导航到该点，且该点旁边就是未探索区域
        
        算法步骤：
          1. 创建自由空间掩码（free_mask）和未知空间掩码（unknown_mask）
          2. 对 unknown_mask 做 1 像素膨胀（dilate）
          3. 前沿 = 膨胀后的未知区域 ∩ 自由区域
          4. 用 cv2.connectedComponents 对前沿点聚类
          5. 过滤太小的簇（噪声），计算每个簇的质心
        
        返回：[(world_x, world_y, cluster_size), ...]
        """
        if self.map_data is None:
            return []

        # 二值掩码
        free_mask = (self.map_data == 0).astype(np.uint8)       # 自由空间
        unknown_mask = (self.map_data == -1).astype(np.uint8)   # 未知空间

        # 膨胀未知空间 1 像素（3×3 核）
        # 膨胀后，原来未知区域的边界会向外扩展 1 像素到自由区域
        kernel = np.ones((3, 3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel, iterations=1)

        # 前沿 = 膨胀后的未知区域 与 自由区域 的交集
        # 结果：自由空间中紧邻未知空间的栅格
        frontier_mask = (dilated_unknown & free_mask).astype(np.uint8)

        # 连通域分析 —— 将相邻前沿点聚成簇
        num_labels, labels = cv2.connectedComponents(frontier_mask)

        frontiers = []
        for label_id in range(1, num_labels):  # label 0 是背景
            # 提取当前簇的所有像素坐标
            rows, cols = np.where(labels == label_id)
            size = len(rows)

            # 过滤太小的簇（可能是噪声或无意义的碎片）
            if size < self.min_frontier_size:
                continue

            # 计算质心（栅格坐标的平均值）
            center_col = int(np.mean(cols))
            center_row = int(np.mean(rows))

            # 验证质心是否在自由空间内（防止质心落在墙上）
            if self.map_data[center_row, center_col] != 0:
                # 质心不在自由空间，找簇内距质心最近的自由空间点
                best_dist = float('inf')
                for r, c in zip(rows, cols):
                    if self.map_data[r, c] == 0:
                        d = (c - center_col)**2 + (r - center_row)**2
                        if d < best_dist:
                            best_dist = d
                            center_col, center_row = c, r

            # 转换为世界坐标
            wx, wy = self._grid_to_world(center_col, center_row)
            frontiers.append((wx, wy, size))

        return frontiers

    # ────────────── 目标选择 ──────────────
    def select_best_goal(self, frontiers):
        """
        从前沿簇中选择评分最高的导航目标
        
        评分函数：
            score = weight_size × (size / max_size)    信息增益分量
                  + weight_dist × (1 / (dist + 0.1))   距离分量
        
        过滤条件：
            - 黑名单中的前沿（之前导航失败的位置）
            - 距离机器人太近的前沿（已在附近）
        
        返回：(goal_x, goal_y, score) 或 None
        """
        if not frontiers:
            return None

        robot_x, robot_y = self._get_robot_pose()
        if robot_x is None:
            return None

        # 最大前沿簇大小（用于归一化）
        max_size = max(f[2] for f in frontiers)

        candidates = []
        for (fx, fy, fsize) in frontiers:
            # 跳过黑名单中的前沿（之前导航失败过的位置）
            if self._is_blacklisted(fx, fy):
                continue

            # 计算到机器人的距离
            dist = np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)

            # 跳过太近的前沿（机器人已经在附近了）
            if dist < self.min_goal_distance:
                continue

            # 归一化评分
            size_score = fsize / max_size           # 0~1，越大信息增益越高
            dist_score = 1.0 / (dist + 0.1)         # 越近越高，+0.1 防除零

            score = (self.weight_size * size_score +
                     self.weight_dist * dist_score)
            candidates.append((fx, fy, fsize, score))

        if not candidates:
            return None

        # 按评分降序，选最优
        candidates.sort(key=lambda c: c[3], reverse=True)
        best = candidates[0]
        return best[0], best[1], best[3]

    def _is_blacklisted(self, x, y):
        """检查 (x, y) 是否在黑名单中（之前导航失败的目标附近）"""
        for bx, by in self.blacklist:
            if (x - bx)**2 + (y - by)**2 < self.blacklist_radius**2:
                return True
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

    def cancel_goal(self):
        """取消当前导航目标"""
        self.ac.cancel_goal()
        self.is_navigating = False
        self.goal_start_time = None
        rospy.loginfo("[前沿探索] 已取消当前导航目标")

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
