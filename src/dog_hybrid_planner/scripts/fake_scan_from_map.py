#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synthetic 2D LiDAR scan from a static OccupancyGrid + TF pose.

The ASK-3 simulator URDF has no LiDAR. The wheeled-robot DWA depends on
LaserScan callbacks for real-time obstacle awareness. To preserve the
same interface (and the same DWA collision-checking code path), we
ray-cast the static `/map` from the dog's current `map -> base` pose
and publish the result on `/scan`.

Notes:
- This sees the static walls of the indoor world but cannot see dynamic
  obstacles. That is a deliberate trade-off: the report Section 4
  documents that the dog stack therefore relies on the global plan +
  inflation for static avoidance, with the synthetic scan giving DWA
  enough coverage to slow down near walls and trigger the recovery FSM
  if the plan grazes a tight gap.
- Ranges are computed by 1-D DDA along the ray. Each ray walks at most
  `range_max / resolution` cells which for a 6 m range on a 0.05 m map
  is 120 cells -- well inside the per-cycle budget at 10 Hz.
"""
import math

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class FakeScanFromMap:
    def __init__(self):
        rospy.init_node('fake_scan_from_map', anonymous=False)

        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base')
        self.scan_frame = rospy.get_param('~scan_frame', 'base')
        self.angle_min = float(rospy.get_param('~angle_min', -math.pi))
        self.angle_max = float(rospy.get_param('~angle_max', math.pi))
        self.angle_increment = float(rospy.get_param('~angle_increment', math.pi / 180.0))
        self.range_min = float(rospy.get_param('~range_min', 0.10))
        self.range_max = float(rospy.get_param('~range_max', 6.0))
        self.publish_rate = float(rospy.get_param('~rate', 10.0))
        self.threshold = int(rospy.get_param('~obstacle_threshold', 65))

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_w = None
        self.map_h = None

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('/map', OccupancyGrid, self._map_cb, queue_size=1)
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._tick)

        # Pre-build angle table
        n = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        self.angles = self.angle_min + np.arange(n) * self.angle_increment
        rospy.loginfo("[FakeScan] %d beams, range %.1f m, %.1f Hz",
                      n, self.range_max, self.publish_rate)

    def _map_cb(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x,
                           msg.info.origin.position.y)
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        self.map_data = (np.array(msg.data, dtype=np.int16)
                         .reshape((self.map_h, self.map_w)))
        rospy.loginfo("[FakeScan] Map %dx%d res=%.3f", self.map_w,
                      self.map_h, self.map_resolution)

    def _world_to_map(self, x, y):
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def _is_obs(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.map_w or my >= self.map_h:
            return True
        v = self.map_data[my, mx]
        if v < 0:
            return False
        return v >= self.threshold

    def _raycast(self, ox, oy, theta):
        """1-D DDA along a ray (ox, oy, theta) until it hits an occupied cell."""
        steps = int(self.range_max / self.map_resolution)
        c = math.cos(theta)
        s = math.sin(theta)
        for i in range(steps):
            r = (i + 0.5) * self.map_resolution
            if r < self.range_min:
                continue
            mx, my = self._world_to_map(ox + r * c, oy + r * s)
            if self._is_obs(mx, my):
                return r
        return self.range_max

    def _tick(self, _evt):
        if self.map_data is None:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.scan_frame
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_max)
        scan.angle_increment = float(self.angle_increment)
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate
        # Cast each beam. 360 beams * 120 steps ~= 43k checks per cycle,
        # tractable in pure python at 10 Hz; if not, drop publish_rate.
        ranges = []
        for a in self.angles:
            ranges.append(self._raycast(x, y, yaw + a))
        scan.ranges = ranges
        self.pub.publish(scan)


if __name__ == '__main__':
    try:
        FakeScanFromMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
