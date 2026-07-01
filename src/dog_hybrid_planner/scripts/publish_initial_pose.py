#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish an initial pose to AMCL after map/scan are alive."""
import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('dog_initial_pose_publisher', anonymous=False)
    x = float(rospy.get_param('~x', 0.0))
    y = float(rospy.get_param('~y', 0.0))
    yaw = float(rospy.get_param('~yaw', 0.0))
    delay = float(rospy.get_param('~delay', 1.0))
    repeats = int(rospy.get_param('~repeats', 8))
    rate_hz = float(rospy.get_param('~rate', 2.0))

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    rospy.wait_for_message('/map', OccupancyGrid)
    rospy.wait_for_message('/scan', LaserScan)
    rospy.sleep(delay)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance[0] = 0.02
    msg.pose.covariance[7] = 0.02
    msg.pose.covariance[35] = 0.01

    rate = rospy.Rate(rate_hz)
    for _ in range(repeats):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo('[DogInitPose] Published AMCL initial pose (%.2f, %.2f, %.2f)',
                  x, y, yaw)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
