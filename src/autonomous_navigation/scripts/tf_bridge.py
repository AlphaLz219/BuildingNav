#!/usr/bin/env python3
"""TF 桥接：Fast-LIO 全局定位 → 标准 nav 栈 map→odom→base_footprint。

用严格 4×4 齐次变换乘法计算 map → odom，正确处理旋转。

  map → camera_init (identity)
  map → odom (T_cam_imu × inv(T_odom_imu))
  odom → base_footprint → ... → livox_imu_link (Gazebo + URDF)
"""
import numpy as np

import rospy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import geometry_msgs.msg
import tf.transformations as tft


def tf_to_mat(t):
    """TransformStamped → 4×4 齐次矩阵"""
    trans = tft.translation_matrix([
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z])
    q = t.transform.rotation
    rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    return np.dot(trans, rot)


def mat_to_tf(mat, stamp, parent, child):
    """4×4 齐次矩阵 → TransformStamped"""
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    trans = tft.translation_from_matrix(mat)
    quat = tft.quaternion_from_matrix(mat)
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


class TFBridge:
    def __init__(self):
        self._buf = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self._buf)
        self._br  = tf2_ros.TransformBroadcaster()
        rospy.loginfo("TF Bridge: waiting for camera_init→body and odom→livox_imu_link...")

        try:
            self._buf.lookup_transform(
                "camera_init", "body", rospy.Time(0), rospy.Duration(120))
            self._buf.lookup_transform(
                "odom", "livox_imu_link", rospy.Time(0), rospy.Duration(120))
        except Exception as e:
            rospy.logerr("TF Bridge init failed: %s", e)
            return

        rospy.loginfo("TF Bridge connected.")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()

                # map → camera_init 恒等
                t_map_cam = geometry_msgs.msg.TransformStamped()
                t_map_cam.header.stamp = now
                t_map_cam.header.frame_id = "map"
                t_map_cam.child_frame_id = "camera_init"
                t_map_cam.transform.rotation.w = 1.0
                self._br.sendTransform(t_map_cam)

                # Fast-LIO: camera_init → body (body ≡ livox_imu_link)
                t_cam_imu = self._buf.lookup_transform(
                    "camera_init", "body", rospy.Time(0), rospy.Duration(0.1))
                # Gazebo: odom → livox_imu_link (= body)
                t_odom_imu = self._buf.lookup_transform(
                    "odom", "livox_imu_link", rospy.Time(0), rospy.Duration(0.1))

                # 严格：map → odom = T_cam_imu × inv(T_odom_imu)
                M_cam_imu = tf_to_mat(t_cam_imu)
                M_odom_imu = tf_to_mat(t_odom_imu)
                M_map_odom = np.dot(M_cam_imu, np.linalg.inv(M_odom_imu))

                t_map_odom = mat_to_tf(M_map_odom, now, "map", "odom")
                self._br.sendTransform(t_map_odom)

            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("tf_bridge")
    TFBridge().run()