#!/usr/bin/env python3
"""
PointCloud → CustomMsg 转换。
Mid360 仿真插件输出旧版 PointCloud，Fast-LIO Avia 模式需要 CustomMsg。
"""
import rospy
from sensor_msgs.msg import PointCloud
from autonomous_navigation.msg import CustomMsg, CustomPoint


class PointCloudToCustomMsg:
    def __init__(self):
        self._pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=10)
        self._sub = rospy.Subscriber("/scan", PointCloud, self._cb)
        rospy.loginfo("PointCloud → CustomMsg: /scan → /livox/lidar")

    def _cb(self, cloud: PointCloud):
        if len(cloud.points) == 0:
            return

        msg = CustomMsg()
        msg.header = cloud.header
        msg.timebase = rospy.Time.now().to_nsec()
        msg.lidar_id = 1
        msg.rsvd = [0, 0, 0]

        t0 = msg.timebase
        for pt in cloud.points:
            cp = CustomPoint()
            cp.x = pt.x
            cp.y = pt.y
            cp.z = pt.z
            cp.reflectivity = 0
            cp.tag = 0
            cp.line = 0
            cp.offset_time = 0  # 仿真的点没有时间偏移
            msg.points.append(cp)
        msg.point_num = len(msg.points)
        self._pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_custommsg")
    PointCloudToCustomMsg()
    rospy.spin()
