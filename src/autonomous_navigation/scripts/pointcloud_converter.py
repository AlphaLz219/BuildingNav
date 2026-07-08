#!/usr/bin/env python3
"""将 sensor_msgs/PointCloud 转换为 sensor_msgs/PointCloud2。

Mid360 仿真插件输出的是旧版 PointCloud 格式，
pointcloud_to_laserscan 和 RViz 需要 PointCloud2 格式。
"""
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct


class PointCloudConverter:
    def __init__(self):
        self._pub = rospy.Publisher("/scan2", PointCloud2, queue_size=10)
        self._sub = rospy.Subscriber("/scan", PointCloud, self._callback)
        rospy.loginfo("PointCloud → PointCloud2 converter ready: /scan → /scan2")

    def _callback(self, cloud_in: PointCloud):
        if len(cloud_in.points) == 0:
            return

        # 查找 intensity channel（可能有不同名称）
        intensity_ch = None
        for ch in cloud_in.channels:
            low = ch.name.lower()
            if "intensity" in low or ch.name == "i" or "reflect" in low:
                intensity_ch = ch
                break

        # 始终包含 intensity，Fast-LIO 需要这个字段
        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
        fmt = "<ffff"

        cloud_out = PointCloud2()
        cloud_out.header = cloud_in.header
        cloud_out.height = 1
        cloud_out.width = len(cloud_in.points)
        cloud_out.fields = fields
        cloud_out.is_bigendian = False
        cloud_out.point_step = point_step
        cloud_out.row_step = point_step * cloud_out.width
        cloud_out.is_dense = True

        buf = bytearray()
        for i, pt in enumerate(cloud_in.points):
            intensity = intensity_ch.values[i] if intensity_ch else 0.0
            buf.extend(struct.pack(fmt, pt.x, pt.y, pt.z, intensity))

        cloud_out.data = bytes(buf)
        self._pub.publish(cloud_out)


if __name__ == "__main__":
    rospy.init_node("pointcloud_converter")
    PointCloudConverter()
    rospy.spin()
