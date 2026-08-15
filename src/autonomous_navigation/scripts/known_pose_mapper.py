#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Known-Pose Mapping 节点
========================

使用 Gazebo 真值位姿（来自 state_from_gazebo 的 map→odom→base TF）
将 2D 激光扫描直接投射到全局栅格地图中，不做任何扫描匹配或回环修正。

输入：
  /scan_2d (sensor_msgs/LaserScan)  — 2D 激光扫描
  TF: map → laser_frame             — 真值位姿

输出：
  /map (nav_msgs/OccupancyGrid)     — 2D 栅格地图（latch=True）

不发布任何 TF，避免与 state_from_gazebo 冲突。
"""

import rospy
import numpy as np
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse


class KnownPoseMapper:
    def __init__(self):
        rospy.init_node('known_pose_mapper')

        # 参数
        self.resolution = rospy.get_param('~resolution', 0.05)
        self.map_width = rospy.get_param('~map_width', 800)    # 栅格数
        self.map_height = rospy.get_param('~map_height', 800)  # 栅格数
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)
        self.laser_frame = rospy.get_param('~laser_frame', 'laser_livox_level')
        self.map_frame = rospy.get_param('~map_frame', 'map')

        # log-odds 参数
        self.l_free = rospy.get_param('~l_free', -0.25)    # 每次穿过的 log-odds 减量（更保守，减少对障碍物的侵蚀）
        self.l_occ = rospy.get_param('~l_occ', 1.2)        # 每次命中的 log-odds 增量（更强，单点即稳定占用）
        self.l_prior = rospy.get_param('~l_prior', 0.0)    # 先验 log-odds（未知）
        self.l_min = rospy.get_param('~l_min', -5.0)       # log-odds 下限
        self.l_max = rospy.get_param('~l_max', 5.0)        # log-odds 上限
        self.max_range = rospy.get_param('~max_range', 12.0)          # 射线最大有效距离(m)
        self.min_occ_dist = rospy.get_param('~min_occ_dist', 0.15)    # 终点过近阈值(m)，避免机器人脚下标 occ
        self.occ_block_thresh = rospy.get_param('~occ_block_thresh', 2.0)  # free 射线被占据格阻挡的 log-odds 阈值

        # 内部状态
        self.log_odds = np.full(
            (self.map_height, self.map_width), self.l_prior, dtype=np.float32)
        self.visited = np.zeros((self.map_height, self.map_width), dtype=bool)
        self.map_origin_x = -(self.map_width * self.resolution) / 2.0
        self.map_origin_y = -(self.map_height * self.resolution) / 2.0

        # TF 监听器
        self.tf_listener = tf.TransformListener()

        # 发布器
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)

        # 订阅激光扫描
        self.scan_sub = rospy.Subscriber(
            '/scan_2d', LaserScan, self._scan_callback, queue_size=10)

        # reset 服务：清空地图，用于多楼层切换
        self.reset_srv = rospy.Service(
            '/known_pose_mapper/reset', Empty, self._reset_callback)

        # 标记是否有新数据需要发布
        self._dirty = False

        rospy.loginfo(
            "[KnownPoseMapper] 初始化完成: %.1fm×%.1fm @ %.3fm/px, "
            "laser_frame=%s, publish_rate=%.1fHz",
            self.map_width * self.resolution,
            self.map_height * self.resolution,
            self.resolution,
            self.laser_frame,
            self.publish_rate)

    def _scan_callback(self, scan):
        """处理一帧激光扫描，投射到栅格地图"""
        # 查询 TF: map → laser_frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.map_frame, self.laser_frame, scan.header.stamp)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "[KnownPoseMapper] TF 查询失败: %s", e)
            return

        # 传感器位姿 (世界坐标)
        sx = trans[0]
        sy = trans[1]
        # 从四元数提取 yaw
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [rot[0], rot[1], rot[2], rot[3]])

        # 传感器在栅格中的位置
        s_col = int((sx - self.map_origin_x) / self.resolution)
        s_row = int((sy - self.map_origin_y) / self.resolution)

        # 逐条射线投射
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            angle = scan.angle_min + i * scan.angle_increment

            # 跳过无效测量 / 超远点
            if r < scan.range_min or r > min(scan.range_max, self.max_range) or not np.isfinite(r):
                continue
            # 过近的终点不标 occupied，避免把机器人脚下标成障碍
            if r < self.min_occ_dist:
                continue

            # 射线终点（世界坐标）
            ex = sx + r * np.cos(yaw + angle)
            ey = sy + r * np.sin(yaw + angle)

            # 终点在栅格中的位置
            e_col = int((ex - self.map_origin_x) / self.resolution)
            e_row = int((ey - self.map_origin_y) / self.resolution)

            # Bresenham 光线投射：穿过的格子标记为 free；被占据格挡住则提前停止
            blocked = self._bresenham_ray(s_col, s_row, e_col, e_row)

            # 终点标记为 occupied（射线未被遮挡时才标）
            if not blocked and (0 <= e_row < self.map_height and 0 <= e_col < self.map_width):
                self.log_odds[e_row, e_col] = np.clip(
                    self.log_odds[e_row, e_col] + self.l_occ,
                    self.l_min, self.l_max)
                self.visited[e_row, e_col] = True

        self._dirty = True

    def _bresenham_ray(self, x0, y0, x1, y1):
        """Bresenham 光线投射：将穿过的格子标记为 free（不含起点和终点）。

        若途中遇到已确信占据的格子，说明射线被障碍物挡住，提前终止并返回 True。
        返回 True 表示射线被遮挡（调用方不应再标终点 occupied）。
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        first = True

        while True:
            if x0 == x1 and y0 == y1:
                return False

            if not first:  # 跳过起点
                if 0 <= y0 < self.map_height and 0 <= x0 < self.map_width:
                    # 已被占据：射线被挡住，停止继续标 free
                    if self.log_odds[y0, x0] >= self.occ_block_thresh:
                        return True
                    self.log_odds[y0, x0] = np.clip(
                        self.log_odds[y0, x0] + self.l_free,
                        self.l_min, self.l_max)
                    self.visited[y0, x0] = True

            first = False
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def _log_odds_to_prob(self, l):
        """log-odds → 占用概率"""
        return 1.0 - 1.0 / (1.0 + np.exp(l))

    def _publish_map(self):
        """发布 OccupancyGrid"""
        if not self._dirty:
            return

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = rospy.Time.now()

        grid.info.resolution = self.resolution
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.origin.position.x = self.map_origin_x
        grid.info.origin.position.y = self.map_origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # log-odds → OccupancyGrid 值 (0=free, 100=occupied, -1=unknown)
        # 只对观测过的格子计算，其余保持 -1，避免每帧全图 exp 运算
        occ_values = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        if self.visited.any():
            prob = self._log_odds_to_prob(self.log_odds[self.visited])
            occ_values[self.visited] = np.clip(
                (prob * 100).astype(np.int8), 0, 100)

        grid.data = occ_values.flatten().tolist()
        self.map_pub.publish(grid)
        self._dirty = False

    def _reset_callback(self, req):
        """清空地图，重置为先验状态（多楼层切换时调用）"""
        self.log_odds[:] = self.l_prior
        self.visited[:] = False
        self._dirty = True
        rospy.loginfo("[KnownPoseMapper] ✓ 地图已清空")
        return EmptyResponse()

    def run(self):
        """主循环：定时发布地图"""
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._publish_map()
            rate.sleep()


if __name__ == '__main__':
    try:
        mapper = KnownPoseMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
