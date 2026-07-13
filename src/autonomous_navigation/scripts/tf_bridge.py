#!/usr/bin/env python3
"""TF 桥接：Fast-LIO odometry → Gazebo world。

发布 camera_init → odom，连接 Fast-LIO 定位和 Gazebo TF 树。
camera_init 作为全局世界坐标系，move_base 直接使用。
"""
import numpy as np
import rospy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import geometry_msgs.msg
import tf.transformations as tft


def tf_to_mat(t):
    trans = tft.translation_matrix([t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z])
    q = t.transform.rotation
    rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    return np.dot(trans, rot)


def main():
    rospy.init_node("tf_bridge")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    br = tf2_ros.TransformBroadcaster()

    rospy.loginfo("TF Bridge: waiting for camera_init→body & odom→livox_imu_link ...")
    buf.lookup_transform("camera_init", "body", rospy.Time(0), rospy.Duration(120))
    buf.lookup_transform("odom", "livox_imu_link", rospy.Time(0), rospy.Duration(120))
    rospy.loginfo("TF Bridge: publishing camera_init → odom")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            t_cam = buf.lookup_transform("camera_init", "body", now, rospy.Duration(0.1))
            t_odom = buf.lookup_transform("odom", "livox_imu_link", now, rospy.Duration(0.1))

            M = np.dot(tf_to_mat(t_cam), np.linalg.inv(tf_to_mat(t_odom)))

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "camera_init"
            t.child_frame_id = "odom"
            t.transform.translation.x = M[0, 3]
            t.transform.translation.y = M[1, 3]
            t.transform.translation.z = M[2, 3]
            q = tft.quaternion_from_matrix(M)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        rate.sleep()


if __name__ == "__main__":
    main()