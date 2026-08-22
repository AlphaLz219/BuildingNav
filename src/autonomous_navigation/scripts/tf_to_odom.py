#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF → Odometry 转换节点 (修复版)
监听 TF odom → base，计算位姿和速度，发布 nav_msgs/Odometry 到 /odom。
"""

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def main():
    rospy.init_node('tf_to_odom')

    odom_frame = rospy.get_param('~odom_frame', 'odom')
    base_frame = rospy.get_param('~base_frame', 'base')
    publish_rate = rospy.get_param('~publish_rate', 30.0)

    listener = tf.TransformListener()
    pub = rospy.Publisher('/odom', Odometry, queue_size=1)

    rate = rospy.Rate(publish_rate)
    rospy.loginfo("[tf_to_odom] 启动: %s → %s @ %.1f Hz", odom_frame, base_frame, publish_rate)

    # 用于差分计算速度的历史状态
    last_trans = None
    last_rot = None
    last_time = None

    while not rospy.is_shutdown():
        try:
            # 获取最新可用的 TF
            now = rospy.Time.now()
            listener.waitForTransform('/' + odom_frame, '/' + base_frame, now, rospy.Duration(0.1))
            (trans, rot) = listener.lookupTransform('/' + odom_frame, '/' + base_frame, now)
            
            # 获取该 TF 的实际时间戳
            tf_time = listener.getLatestCommonTime('/' + odom_frame, '/' + base_frame)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

        msg = Odometry()
        msg.header.stamp = tf_time  # 使用 TF 的真实时间戳
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame

        # 1. 设置 Pose
        msg.pose.pose.position.x = trans[0]
        msg.pose.pose.position.y = trans[1]
        msg.pose.pose.position.z = trans[2]
        msg.pose.pose.orientation = Quaternion(*rot)

        # 2. 计算并设置 Twist (速度)
        if last_trans is not None and last_time is not None:
            dt = (tf_time - last_time).to_sec()
            if dt > 0.001:  # 防止除以 0
                # 线速度
                vx = (trans[0] - last_trans[0]) / dt
                vy = (trans[1] - last_trans[1]) / dt
                
                # 角速度 (处理 yaw 跳变)
                _, _, yaw_curr = tf.transformations.euler_from_quaternion(rot)
                _, _, yaw_last = tf.transformations.euler_from_quaternion(last_rot)
                dyaw = yaw_curr - yaw_last
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw)) # 归一化到 [-pi, pi]
                vyaw = dyaw / dt
                
                msg.twist.twist.linear.x = vx
                msg.twist.twist.linear.y = vy
                msg.twist.twist.angular.z = vyaw

        # 更新历史状态
        last_trans = trans
        last_rot = rot
        last_time = tf_time

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass