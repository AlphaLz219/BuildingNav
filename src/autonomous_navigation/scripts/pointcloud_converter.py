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

        # 构建 PointCloud2 的 fields
        fields = [
            PointField(name="x",      offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",      offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",      offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        # 如果有 intensity channel，也保留
        has_intensity = False
        point_step = 12  # x,y,z = 3 * float32 = 12 bytes
        intensity_offset = 12
        for ch in cloud_in.channels:
            if ch.name in ("intensity", "intensities"):
                fields.append(PointField(name="intensity", offset=intensity_offset,
                                         datatype=PointField.FLOAT32, count=1))
                has_intensity = True
                point_step = 16  # +1 float32

        # 打包数据
        cloud_out = PointCloud2()
        cloud_out.header = cloud_in.header
        cloud_out.height = 1
        cloud_out.width = len(cloud_in.points)
        cloud_out.fields = fields
        cloud_out.is_bigendian = False
        cloud_out.point_step = point_step
        cloud_out.row_step = point_step * cloud_out.width
        cloud_out.is_dense = True

        fmt = "<fff"
        if has_intensity:
            fmt += "f"

        buf = bytearray()
        for i, pt in enumerate(cloud_in.points):
            if has_intensity:
                intensity = cloud_in.channels[0].values[i]
                buf.extend(struct.pack(fmt, pt.x, pt.y, pt.z, intensity))
            else:
                buf.extend(struct.pack(fmt, pt.x, pt.y, pt.z))

        cloud_out.data = bytes(buf)
        self._pub.publish(cloud_out)


if __name__ == "__main__":
    rospy.init_node("pointcloud_converter")
    PointCloudConverter()
    rospy.spin()
