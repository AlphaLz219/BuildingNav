#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel 航位推算伪里程计节点（Dead-Reckoning Odom，修复版）
"""

import math
import threading

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class CmdVelOdom:
    def __init__(self):
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base')
        self.base_z = rospy.get_param('~base_z', 0.29)
        self.rate_hz = rospy.get_param('~rate', 100.0)
        self.max_dt = rospy.get_param('~max_dt', 0.2)

        # 关键修复：/cmd_vel 超时后强制零速度
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.3)

        # 限幅，防止异常速度命令积分爆掉
        self.max_speed = rospy.get_param('~max_speed', 1.0)

        # 初始位姿
        self.init_x = rospy.get_param('~init_x', 0.0)
        self.init_y = rospy.get_param('~init_y', 0.6)
        self.init_yaw = rospy.get_param('~init_yaw', 1.5708)

        self.lock = threading.Lock()

        self.x = self.init_x
        self.y = self.init_y
        self.yaw = self.init_yaw

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.last_cmd_time = None

        self.vx_pub = 0.0
        self.vy_pub = 0.0
        self.vyaw_pub = 0.0

        self._have_imu = False
        self._last_imu_yaw = None
        self._last_imu_stamp = None

        self._last_time = None
        self._last_yaw_for_rate = None

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.tf_bc = tf.TransformBroadcaster()

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=10)
        rospy.Subscriber('/trunk_imu', Imu, self._imu_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

        rospy.Service('~reset', Empty, self._reset_cb)

        rospy.loginfo(
            "[cmd_vel_odom] 启动: %s→%s @ %.0fHz, cmd_timeout=%.2fs",
            self.odom_frame, self.base_frame, self.rate_hz, self.cmd_timeout)

    def _reset_cb(self, req):
        with self.lock:
            self.x = self.init_x
            self.y = self.init_y
            self.yaw = self.init_yaw

            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.last_cmd_time = None

            self.vx_pub = 0.0
            self.vy_pub = 0.0
            self.vyaw_pub = 0.0

            self._have_imu = False
            self._last_imu_yaw = None
            self._last_imu_stamp = None
            self._last_time = None
            self._last_yaw_for_rate = None

        rospy.loginfo("[cmd_vel_odom] 已重置")
        return EmptyResponse()

    def _cmd_vel_cb(self, msg):
        vx = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        vy = max(-self.max_speed, min(self.max_speed, msg.linear.y))

        with self.lock:
            self.cmd_vx = vx
            self.cmd_vy = vy
            self.last_cmd_time = rospy.Time.now()

    def _imu_cb(self, msg):
        q = msg.orientation
        _, _, imu_yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        with self.lock:
            # 直接使用 IMU 绝对 yaw，不做任何 offset
            self.yaw = normalize_angle(imu_yaw)

            # 用 IMU 时间戳估计 yaw 角速度
            if (self._last_imu_yaw is not None and
                    self._last_imu_stamp is not None):
                dt_imu = (msg.header.stamp - self._last_imu_stamp).to_sec()
                if dt_imu > 1e-4:
                    dyaw = normalize_angle(self.yaw - self._last_imu_yaw)
                    self.vyaw_pub = dyaw / dt_imu

            self._last_imu_yaw = self.yaw
            self._last_imu_stamp = msg.header.stamp
            self._have_imu = True

    def _tick(self, event):
        now = rospy.Time.now()

        with self.lock:
            if not self._have_imu:
                self._last_time = now
                return

            # 关键：/cmd_vel 超时 → 零速度
            if (self.last_cmd_time is None or
                    (now - self.last_cmd_time).to_sec() > self.cmd_timeout):
                vx = 0.0
                vy = 0.0
            else:
                vx = self.cmd_vx
                vy = self.cmd_vy

            yaw = self.yaw

        if self._last_time is None:
            self._last_time = now
            with self.lock:
                self._last_yaw_for_rate = yaw
            return

        dt = (now - self._last_time).to_sec()
        self._last_time = now

        if not (0.0 < dt < self.max_dt):
            with self.lock:
                self.vx_pub = 0.0
                self.vy_pub = 0.0
                self._last_yaw_for_rate = yaw
            return

        c = math.cos(yaw)
        s = math.sin(yaw)

        with self.lock:
            self.x += (vx * c - vy * s) * dt
            self.y += (vx * s + vy * c) * dt

            # 如果 IMU 回调没有给出角速度，用定时器差分兜底
            if self.vyaw_pub == 0.0 and self._last_yaw_for_rate is not None:
                dyaw = normalize_angle(yaw - self._last_yaw_for_rate)
                if abs(dyaw) > 1e-6:
                    self.vyaw_pub = dyaw / dt

            self._last_yaw_for_rate = yaw

            self.vx_pub = vx
            self.vy_pub = vy

            pub_x = self.x
            pub_y = self.y
            pub_yaw = self.yaw
            pub_vx = self.vx_pub
            pub_vy = self.vy_pub
            pub_vyaw = self.vyaw_pub

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, pub_yaw)

        # odom -> base TF
        self.tf_bc.sendTransform(
            (pub_x, pub_y, self.base_z), q,
            now, self.base_frame, self.odom_frame)

        # /odom topic
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = pub_x
        odom.pose.pose.position.y = pub_y
        odom.pose.pose.position.z = self.base_z
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = pub_vx
        odom.twist.twist.linear.y = pub_vy
        odom.twist.twist.angular.z = pub_vyaw

        self.odom_pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_odom')
    CmdVelOdom()
    rospy.spin()