#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground-truth pose bridge for the ASK-3 simulator.

The wheeled-robot stack used AMCL on top of the laser scan to align
`map -> odom`. The dog has no laser and no AMCL, so for the simulation
we shortcut by reading /gazebo/model_states (true world pose of the
robot) and broadcasting `map -> odom` such that the existing
mydog_state_estimator's `odom -> base` chain ends up with `map -> base`
equal to ground truth.

This is a SIMULATION-ONLY localisation source. On a real ASK-3 it would
be replaced by either AMCL with an added 2-D LiDAR or a VIO front-end.

Implementation note: we publish the transform at a steady 30 Hz off a
timer rather than once per /gazebo/model_states message. /model_states
arrives at 1 kHz and dropping its callback rate in TF would make
TransformListener buffer fills very noisy.
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class GroundTruthPoseBridge:
    def __init__(self):
        rospy.init_node('gt_pose_bridge', anonymous=False)
        self.model_name = rospy.get_param('~model_name', 'mydog')
        self.parent_frame = rospy.get_param('~parent_frame', 'map')
        self.child_frame = rospy.get_param('~child_frame', 'odom')
        self.publish_rate = float(rospy.get_param('~publish_rate', 30.0))
        # If the state estimator publishes odom -> base with the dog's
        # ground-truth pose, then we want map -> odom == identity.
        # If it publishes odom -> base relative to the spawn pose (i.e.
        # initial pose is origin), we'd need the spawn offset.
        self.identity_only = bool(rospy.get_param('~identity_only', True))

        self.last_pose = None
        self.broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._cb, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._tick)
        rospy.loginfo("[GTPose] Broadcasting %s -> %s (model=%s, identity_only=%s)",
                      self.parent_frame, self.child_frame,
                      self.model_name, self.identity_only)

    def _cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        self.last_pose = msg.pose[idx]

    def _tick(self, _evt):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        if self.identity_only or self.last_pose is None:
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
        else:
            t.transform.translation.x = self.last_pose.position.x
            t.transform.translation.y = self.last_pose.position.y
            t.transform.translation.z = self.last_pose.position.z
            t.transform.rotation = self.last_pose.orientation
        self.broadcaster.sendTransform(t)


if __name__ == '__main__':
    try:
        GroundTruthPoseBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
