#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulation-only kinematic base driver for the ASK-3 Gazebo model.

The bundled MNN gait policy in dog_sim can hold a standing pose in this
workspace, but in the ASK-3 setup it produces near-zero walking actions.
For path-planning experiments we still need the Gazebo model pose to
follow the planner's body-frame velocity commands. This node integrates
the same /ask/dog/* command interface and writes the resulting pose back
to Gazebo with /gazebo/set_model_state.
"""
import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class DogKinematicDriver:
    def __init__(self):
        rospy.init_node('dog_kinematic_driver', anonymous=False)

        self.model_name = rospy.get_param('~model_name', 'mydog')
        self.reference_frame = rospy.get_param('~reference_frame', 'world')
        self.rate_hz = float(rospy.get_param('~rate', 30.0))
        self.require_start = bool(rospy.get_param('~require_start', False))
        self.enabled = not self.require_start

        self.max_vx = abs(float(rospy.get_param('~max_vx', 0.30)))
        self.max_vy = abs(float(rospy.get_param('~max_vy', 0.20)))
        self.max_w = abs(float(rospy.get_param('~max_w', 0.90)))
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.fixed_z = float(rospy.get_param('~fixed_z', -1.0))
        self.collision_radius = abs(float(rospy.get_param('~collision_radius', 0.24)))

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.last_cmd_time = rospy.Time.now()
        self.pose = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        rospy.Subscriber('/ask/dog/forward_back', Float32, self._vx_cb, queue_size=1)
        rospy.Subscriber('/ask/dog/left_right', Float32, self._vy_cb, queue_size=1)
        rospy.Subscriber('/ask/dog/yaw', Float32, self._w_cb, queue_size=1)
        rospy.Subscriber('/ask/dog/start', Bool, self._start_cb, queue_size=1)
        rospy.Subscriber('/ask/dog/stand', Bool, self._stand_cb, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self._map_cb, queue_size=1)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self._read_initial_pose()
        self.last_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.loginfo('[DogKin] Driving Gazebo model %s kinematically (enabled=%s)',
                      self.model_name, self.enabled)

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, float(v)))

    def _touch_cmd(self):
        self.last_cmd_time = rospy.Time.now()

    def _vx_cb(self, msg):
        self.vx = self._clamp(msg.data, self.max_vx)
        self._touch_cmd()

    def _vy_cb(self, msg):
        self.vy = self._clamp(msg.data, self.max_vy)
        self._touch_cmd()

    def _w_cb(self, msg):
        self.w = self._clamp(msg.data, self.max_w)
        self._touch_cmd()

    def _start_cb(self, msg):
        if msg.data:
            self.enabled = True

    def _stand_cb(self, msg):
        if msg.data and self.require_start:
            self.enabled = False
        if msg.data:
            self.vx = self.vy = self.w = 0.0

    def _map_cb(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = tuple(msg.data)

    def _pose_free(self, x, y):
        if self.map_data is None or self.map_resolution <= 0.0:
            return True
        mx = int((x - self.map_origin_x) / self.map_resolution)
        my = int((y - self.map_origin_y) / self.map_resolution)
        r = int(math.ceil(self.collision_radius / self.map_resolution))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if math.hypot(dx, dy) * self.map_resolution > self.collision_radius:
                    continue
                cx = mx + dx
                cy = my + dy
                if cx < 0 or cy < 0 or cx >= self.map_width or cy >= self.map_height:
                    return False
                occ = self.map_data[cy * self.map_width + cx]
                if occ < 0 or occ >= 65:
                    return False
        return True

    def _read_initial_pose(self):
        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(
                    '/gazebo/model_states', ModelStates, timeout=1.0)
            except rospy.ROSException:
                rospy.logwarn_throttle(
                    2.0, '[DogKin] Waiting for /gazebo/model_states')
                continue
            try:
                idx = msg.name.index(self.model_name)
            except ValueError:
                rospy.logwarn_throttle(
                    2.0, '[DogKin] Waiting for Gazebo model %s',
                    self.model_name)
                continue
            self.pose = msg.pose[idx]
            if self.fixed_z >= 0.0:
                self.pose.position.z = self.fixed_z
            q = self.pose.orientation
            self.roll, self.pitch, self.yaw = euler_from_quaternion(
                [q.x, q.y, q.z, q.w])
            return

    def _tick(self, _evt):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if self.pose is None or dt <= 0.0 or dt > 0.2:
            return

        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.vx = self.vy = self.w = 0.0

        if not self.enabled:
            return

        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        dx = (self.vx * cy - self.vy * sy) * dt
        dy = (self.vx * sy + self.vy * cy) * dt
        old_x = self.pose.position.x
        old_y = self.pose.position.y
        new_x = old_x + dx
        new_y = old_y + dy
        blocked = False
        if self._pose_free(new_x, new_y):
            self.pose.position.x = new_x
            self.pose.position.y = new_y
        elif self._pose_free(old_x + dx, old_y):
            self.pose.position.x = old_x + dx
            self.vy = 0.0
            blocked = True
        elif self._pose_free(old_x, old_y + dy):
            self.pose.position.y = old_y + dy
            self.vx = 0.0
            blocked = True
        else:
            self.vx = self.vy = 0.0
            blocked = True
        if blocked:
            rospy.logwarn_throttle(
                1.0, '[DogKin] Map collision guard blocked translation at (%.2f, %.2f)',
                new_x, new_y)
        if self.fixed_z >= 0.0:
            self.pose.position.z = self.fixed_z
        self.yaw += self.w * dt
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.reference_frame
        state.pose = self.pose
        state.twist.linear.x = self.vx
        state.twist.linear.y = self.vy
        state.twist.angular.z = self.w
        try:
            self.set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, '[DogKin] set_model_state failed: %s', exc)


if __name__ == '__main__':
    try:
        DogKinematicDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
