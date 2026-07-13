#!/usr/bin/env python3
"""Fast-LIO 定位 → base_footprint（Z 轴锁定，消除 IMU 抖动）"""
import rospy
import tf2_ros
import geometry_msgs.msg

B_TO_BF_X = -0.017   # body → base_footprint X 偏移
B_TO_BF_Y =  0.023   # body → base_footprint Y 偏移
GROUND_Z  =  0.0     # base_footprint 锁定在 Z=0


def main():
    rospy.init_node("lio_to_base")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    br = tf2_ros.TransformBroadcaster()

    rospy.loginfo("LIO→Base: camera_init→body → camera_init→base_footprint")
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform("camera_init", "body",
                                     rospy.Time(0), rospy.Duration(0.05))
            out = geometry_msgs.msg.TransformStamped()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = "camera_init"
            out.child_frame_id = "base_footprint"
            out.transform.translation.x = t.transform.translation.x + B_TO_BF_X
            out.transform.translation.y = t.transform.translation.y + B_TO_BF_Y
            out.transform.translation.z = GROUND_Z
            out.transform.rotation = t.transform.rotation
            br.sendTransform(out)
        except Exception:
            pass
        rate.sleep()


if __name__ == "__main__":
    main()