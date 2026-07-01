#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optional helper: convert geometry_msgs/Twist on /cmd_vel into the dog's
three-channel Float32 interface. Useful for keyboard tele-op tools that
already speak Twist (e.g. teleop_twist_keyboard).

The hybrid navigator (dog_navigation.py) does NOT need this helper - it
publishes the Float32 channels directly. This bridge is only for manual
debugging.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class CmdBridge:
    def __init__(self):
        rospy.init_node('dog_command_bridge', anonymous=False)
        self.fb_pub = rospy.Publisher('/ask/dog/forward_back', Float32, queue_size=1)
        self.lr_pub = rospy.Publisher('/ask/dog/left_right',   Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher('/ask/dog/yaw',         Float32, queue_size=1)
        self.start_pub = rospy.Publisher('/ask/dog/start', Bool, queue_size=1, latch=True)
        self.walk_pub  = rospy.Publisher('/ask/dog/walk',  Bool, queue_size=1, latch=True)
        # Latch walking on at startup
        b = Bool()
        b.data = True
        rospy.sleep(0.5)
        self.start_pub.publish(b)
        self.walk_pub.publish(b)
        rospy.Subscriber('/cmd_vel', Twist, self._cb, queue_size=1)
        rospy.loginfo("[CmdBridge] Twist /cmd_vel -> dog Float32 channels")

    def _cb(self, msg):
        self.fb_pub.publish(Float32(data=float(msg.linear.x)))
        self.lr_pub.publish(Float32(data=float(msg.linear.y)))
        self.yaw_pub.publish(Float32(data=float(msg.angular.z)))


if __name__ == '__main__':
    try:
        CmdBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
