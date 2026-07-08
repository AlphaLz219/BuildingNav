#!/usr/bin/env python3
"""TF 桥接：合并 Fast-LIO 和 Gazebo 两棵 TF 树。

T(camera_init → odom) = T(camera_init → body) * inv(T(odom → livox_imu_link))
Fast-LIO 的 body 和 Gazebo 的 livox_imu_link 是同一物理点。
"""
import rospy
import tf


def main():
    rospy.init_node("tf_bridge")
    br = tf.TransformBroadcaster()
    lis = tf.TransformListener()
    rate = rospy.Rate(30)

    rospy.loginfo("TF Bridge: waiting for transforms...")
    # 等两边 TF 都就绪
    lis.waitForTransform("camera_init", "body", rospy.Time(0), rospy.Duration(60))
    lis.waitForTransform("odom", "livox_imu_link", rospy.Time(0), rospy.Duration(60))
    rospy.loginfo("TF Bridge: connected!")

    while not rospy.is_shutdown():
        try:
            # Fast-LIO: body 在 camera_init 系下的位姿
            (cam_p, cam_q) = lis.lookupTransform(
                "camera_init", "body", rospy.Time(0))

            # Gazebo: livox_imu_link（= body）在 odom 系下的位姿
            (odom_p, odom_q) = lis.lookupTransform(
                "odom", "livox_imu_link", rospy.Time(0))

            # 计算 odom 在 camera_init 系下的位置差（仅平移，不用朝向）
            tx = cam_p[0] - odom_p[0]
            ty = cam_p[1] - odom_p[1]
            tz = cam_p[2] - odom_p[2]

            br.sendTransform((tx, ty, tz), (0, 0, 0, 1),
                             rospy.Time.now(), "odom", "camera_init")
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass
        rate.sleep()


if __name__ == "__main__":
    main()
