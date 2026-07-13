#!/usr/bin/env python3
"""PointCloud → CustomMsg 转换。

Mid360 仿真插件输出旧版 PointCloud，Fast-LIO Avia 模式需要 CustomMsg。
关键修复：
  - offset_time：按点索引等间隔分配，使 Fast-LIO 能做运动补偿
  - line：按索引循环分配 0~5，匹配 Mid-360 的 6 线扫描
  - timebase：使用点云自身的 header.stamp，保证与 IMU 时间对齐
"""
import math
import rospy
from sensor_msgs.msg import PointCloud
from autonomous_navigation.msg import CustomMsg, CustomPoint

LIVOX_SCAN_LINE = 6           # Mid-360 有 6 条扫描线
SCAN_PERIOD_US  = 100000      # 10Hz → 100ms = 100000μs


class PointCloudToCustomMsg:
    def __init__(self):
        self._pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=10)
        self._sub = rospy.Subscriber("/scan", PointCloud, self._cb)
        rospy.loginfo("PointCloud → CustomMsg: /scan → /livox/lidar"
                       " | line=0~%d | offset_time=per-point", LIVOX_SCAN_LINE - 1)

    def _cb(self, cloud: PointCloud):
        n = len(cloud.points)
        if n == 0:
            return

        msg = CustomMsg()
        msg.header = cloud.header
        msg.timebase = cloud.header.stamp.to_nsec()
        msg.lidar_id = 1
        msg.rsvd = [0, 0, 0]

        dt_per_point = float(SCAN_PERIOD_US) / float(n)

        for i, pt in enumerate(cloud.points):
            cp = CustomPoint()
            cp.x = pt.x
            cp.y = pt.y
            cp.z = pt.z
            cp.reflectivity = 0
            cp.tag = 0
            cp.line = i % LIVOX_SCAN_LINE
            cp.offset_time = int(i * dt_per_point)
            msg.points.append(cp)
        msg.point_num = n
        self._pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_custommsg")
    PointCloudToCustomMsg()
    rospy.spin()
