#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid navigator: improved A* + improved DWA + anti-spin recovery.

Fixes carried forward:
- Pose is taken ONLY from TF (map -> base_footprint).
- RecoveryFSM handles stuck -> rotate -> back-off -> replan.
- Emergency stop if front laser < threshold.
- Periodic replanning.

NEW in this revision (corner-spinning fix):
- ProgressMonitor tracks min distance to goal over a sliding window. If
  the robot has not made >=0.15 m of net progress for 6 s, recovery is
  triggered regardless of DWA's instantaneous stuck flag. This catches
  the failure mode where DWA cycles through in-place rotations that
  reset `stuck_since` every other tick.
- AngularMotionMonitor detects pure-rotation oscillation: if cmd.angular
  magnitude has sign changes > 3 times in 3 s while |v| < 0.02, we also
  treat the robot as stuck.
- RecoveryFSM is more aggressive: 2 s wider rotation (1.5 rad/s) + 1.5 s
  back-off (-0.12 m/s). After two consecutive recovery cycles without
  progress, the FSM flips rotation direction and backs off further.
"""
import math
import numpy as np
from collections import deque
from threading import Lock

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from improved_astar import ImprovedAStar
from improved_dwa import ImprovedDWA


class ProgressMonitor:
    """Watch robot progress toward current goal.

    Keep a sliding window of (time, x, y, dist_to_goal) samples. Report
    'no_progress' ONLY when both:
      - robot's position span (max-min of x, y) over the window is below
        `min_position_span` (robot essentially not moving); AND
      - minimum dist_to_goal in the window has not dropped by `min_drop`
        (robot not getting closer to goal).
    This prevents normal exploration/oscillation from being flagged.
    """
    def __init__(self, window=8.0, min_drop=0.15, min_position_span=0.15):
        self.window = window
        self.min_drop = min_drop
        self.min_position_span = min_position_span
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, now, x, y, dist):
        self.samples.append((now, x, y, dist))
        while self.samples and (now - self.samples[0][0]) > self.window:
            self.samples.popleft()

    def no_progress(self, now):
        if len(self.samples) < 10:
            return False
        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        dists = [s[3] for s in self.samples]
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        drop = dists[0] - min(dists)
        # robot needs to be BOTH immobile AND not closing on goal
        return (span < self.min_position_span) and (drop < self.min_drop)


class OscillationMonitor:
    """Detect in-place rotation oscillation: linear ~0 and ω changes sign
    too often."""
    def __init__(self, window=3.0, min_sign_changes=3, v_thresh=0.02):
        self.window = window
        self.min_sign_changes = min_sign_changes
        self.v_thresh = v_thresh
        self.samples = deque()

    def update(self, now, v, w):
        self.samples.append((now, v, w))
        while self.samples and (now - self.samples[0][0]) > self.window:
            self.samples.popleft()

    def oscillating(self):
        if len(self.samples) < 8:
            return False
        if max(abs(v) for _, v, _ in self.samples) > self.v_thresh:
            return False
        # count zero-crossings of ω (ignore tiny w)
        signs = [math.copysign(1.0, w) for _, _, w in self.samples if abs(w) > 0.1]
        if len(signs) < 4:
            return False
        changes = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        return changes >= self.min_sign_changes


class RecoveryFSM:
    IDLE = 0
    ROTATE = 1
    BACK_OFF = 2

    def __init__(self):
        self.state = self.IDLE
        self.state_start = 0.0
        self.stuck_since = None
        self.rotate_dir = 1.0
        self.attempts = 0                 # consecutive recovery cycles
        self.rotate_duration = 1.2        # shorter rotation so we resume quickly
        self.backoff_duration = 0.6       # brief back-off, avoid running far backward
        self.rotate_speed = 1.0           # moderate rotation
        self.backoff_speed = -0.08

    def notify_stuck(self, now):
        if self.stuck_since is None:
            self.stuck_since = now

    def notify_moving(self):
        # called ONLY when robot has made progress
        self.stuck_since = None
        self.attempts = 0
        if self.state != self.IDLE:
            self.state = self.IDLE

    def should_trigger(self, now, threshold=1.5):
        return (self.stuck_since is not None
                and (now - self.stuck_since) > threshold
                and self.state == self.IDLE)

    def start(self, now, away_bearing):
        self.state = self.ROTATE
        self.state_start = now
        # alternate direction if we're retrying to escape different way
        if self.attempts % 2 == 0:
            self.rotate_dir = -1.0 if away_bearing > 0 else 1.0
        else:
            self.rotate_dir = 1.0 if away_bearing > 0 else -1.0
        self.attempts += 1

    def step(self, now):
        t = now - self.state_start
        # scale rotation/backoff with consecutive attempts (larger escapes later)
        rot_t = self.rotate_duration * (1.0 + 0.5 * (self.attempts - 1))
        bk_t  = self.backoff_duration * (1.0 + 0.5 * (self.attempts - 1))
        if self.state == self.ROTATE:
            if t < rot_t:
                return 0.0, self.rotate_speed * self.rotate_dir, False
            self.state = self.BACK_OFF
            self.state_start = now
            return 0.0, 0.0, False
        if self.state == self.BACK_OFF:
            if t < bk_t:
                return self.backoff_speed, 0.0, False
            self.state = self.IDLE
            self.stuck_since = None   # new window of grace after recovery
            return 0.0, 0.0, True
        return 0.0, 0.0, True


class HybridNavigator:
    def __init__(self):
        rospy.init_node('hybrid_navigator', anonymous=False)

        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.25)
        self.local_target_lookahead = rospy.get_param("~lookahead", 0.4)
        self.replan_period = rospy.get_param("~replan_period", 4.0)
        self.emergency_front_dist = rospy.get_param("~emergency_front_dist", 0.15)
        self.progress_window = rospy.get_param("~progress_window", 8.0)
        self.progress_min_drop = rospy.get_param("~progress_min_drop", 0.20)

        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None
        self.robot_v = 0.0
        self.robot_w = 0.0
        self.prev_theta = 0.0
        self.laser_points = []
        self.front_min_range = float('inf')
        self.laser_lock = Lock()

        self.global_smooth_path = []
        self.global_key_points = []
        self.current_key_idx = 0
        self.current_goal = None
        self.navigating = False
        self.last_plan_time = 0.0

        self.map_ready = False
        self.costmap = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None

        self.dwa = ImprovedDWA()
        self.recovery = RecoveryFSM()
        self.progress = ProgressMonitor(
            window=self.progress_window,
            min_drop=self.progress_min_drop,
            min_position_span=rospy.get_param('~progress_min_span', 0.15))
        self.osc = OscillationMonitor()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.global_path_pub = rospy.Publisher("/hybrid_global_path", Path, queue_size=1, latch=True)
        self.key_pts_pub = rospy.Publisher("/hybrid_key_points", MarkerArray, queue_size=1, latch=True)
        self.local_target_pub = rospy.Publisher("/hybrid_local_target", Marker, queue_size=1)

        rospy.loginfo("[HybridNav] Initialized, waiting for map and TF...")

    def map_cb(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.costmap = data.copy()
        self.map_ready = True
        rospy.loginfo("[HybridNav] Map received %dx%d res=%.3f origin=(%.2f,%.2f)",
                      self.map_width, self.map_height, self.map_resolution,
                      self.map_origin[0], self.map_origin[1])

    def _update_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return False
        self.robot_x = trans.transform.translation.x
        self.robot_y = trans.transform.translation.y
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        return True

    def odom_cb(self, msg):
        self.robot_v = msg.twist.twist.linear.x
        self.robot_w = msg.twist.twist.angular.z
        self._update_pose_from_tf()

    def scan_cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        points = []
        angle = msg.angle_min
        front_min = float('inf')
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                px = tx + lx * cos_yaw - ly * sin_yaw
                py = ty + lx * sin_yaw + ly * cos_yaw
                points.append((px, py))
                if abs(angle) < math.radians(30) and r < front_min:
                    front_min = r
            angle += msg.angle_increment

        with self.laser_lock:
            self.laser_points = points
            self.front_min_range = front_min

    def goal_cb(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("[HybridNav] Goal received (%.2f, %.2f)", *self.current_goal)
        self.progress.reset()
        self.recovery = RecoveryFSM()
        self.plan_global()

    def plan_global(self):
        if not self.map_ready:
            rospy.logwarn_throttle(2.0, "[HybridNav] Map not ready")
            return False
        if self.robot_x is None:
            rospy.logwarn_throttle(2.0, "[HybridNav] Pose not ready")
            return False
        start = (self.robot_x, self.robot_y)
        goal = self.current_goal
        if goal is None:
            return False
        rospy.loginfo("[HybridNav] Planning (%.2f,%.2f) -> (%.2f,%.2f)",
                      start[0], start[1], goal[0], goal[1])
        astar = ImprovedAStar(self.costmap, self.map_resolution,
                              self.map_origin, self.map_width, self.map_height)
        smooth_path, key_points = astar.plan(start, goal)
        if not smooth_path:
            rospy.logerr("[HybridNav] Global planning FAILED")
            return False
        self.global_smooth_path = smooth_path
        self.global_key_points = key_points
        self.current_key_idx = 0
        self.navigating = True
        self.prev_theta = self.robot_theta or 0.0
        self.last_plan_time = rospy.get_time()
        self.publish_global_path()
        self.publish_key_points()
        rospy.loginfo("[HybridNav] Global path OK, %d key points", len(key_points))
        return True

    def publish_global_path(self):
        p = Path()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        for (x, y) in self.global_smooth_path:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            p.poses.append(ps)
        self.global_path_pub.publish(p)

    def publish_key_points(self):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, (x, y) in enumerate(self.global_key_points):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = rospy.Time.now()
            m.ns = "key_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 1.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.key_pts_pub.publish(ma)

    def publish_local_target(self, x, y):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "local_target"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        m.scale.x = 0.25
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.color.g = 1.0
        m.color.a = 1.0
        self.local_target_pub.publish(m)

    def get_local_target(self):
        if not self.global_key_points:
            return None
        while self.current_key_idx < len(self.global_key_points) - 1:
            kx, ky = self.global_key_points[self.current_key_idx]
            if math.hypot(kx - self.robot_x, ky - self.robot_y) < self.local_target_lookahead:
                self.current_key_idx += 1
            else:
                break
        return self.global_key_points[self.current_key_idx]

    def control_loop(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            self._update_pose_from_tf()

            if not self.navigating or self.robot_x is None or self.current_goal is None:
                self.cmd_pub.publish(Twist())
                rate.sleep()
                continue

            # distance to goal + progress monitoring
            dg = math.hypot(self.robot_x - self.current_goal[0],
                            self.robot_y - self.current_goal[1])
            self.progress.update(now, self.robot_x, self.robot_y, dg)

            if dg < self.goal_tolerance:
                rospy.loginfo("[HybridNav] *** GOAL REACHED ***")
                self.cmd_pub.publish(Twist())
                self.navigating = False
                self.recovery = RecoveryFSM()
                self.progress.reset()
                rate.sleep()
                continue

            # recovery in progress?
            if self.recovery.state != RecoveryFSM.IDLE:
                v, w, finished = self.recovery.step(now)
                cmd = Twist()
                cmd.linear.x = v
                cmd.angular.z = w
                self.cmd_pub.publish(cmd)
                self.osc.update(now, v, w)
                if finished:
                    rospy.logwarn("[HybridNav] Recovery finished (attempt %d) - replanning",
                                  self.recovery.attempts)
                    self.plan_global()
                rate.sleep()
                continue

            # laser snapshot
            with self.laser_lock:
                front = self.front_min_range
                obs = list(self.laser_points)

            # emergency stop
            if front < self.emergency_front_dist:
                rospy.logwarn_throttle(1.0,
                    "[HybridNav] Emergency stop: front=%.2fm", front)
                self.cmd_pub.publish(Twist())
                self.recovery.notify_stuck(now)
                if self.recovery.should_trigger(now, threshold=0.5):
                    self.recovery.start(now, 0.0)
                rate.sleep()
                continue

            # pick local target
            target = self.get_local_target()
            if target is None:
                self.cmd_pub.publish(Twist())
                rate.sleep()
                continue
            self.publish_local_target(*target)

            v, w, info = self.dwa.plan(
                current_x=self.robot_x,
                current_y=self.robot_y,
                current_theta=self.robot_theta,
                v_curr=self.robot_v,
                w_curr=self.robot_w,
                goal_x=target[0],
                goal_y=target[1],
                obstacle_points=obs,
                prev_theta=self.prev_theta,
            )

            # oscillation + progress detection
            dwa_stuck  = info.get('stuck', False)
            no_prog    = self.progress.no_progress(now)
            oscillate  = self.osc.oscillating()

            if dwa_stuck or no_prog or oscillate:
                self.recovery.notify_stuck(now)
                # lower threshold when we have compelling evidence
                trig = 1.5 if (no_prog or oscillate) else 2.5
                if self.recovery.should_trigger(now, threshold=trig):
                    bearing = 0.0
                    if info.get('nearest_obs') is not None:
                        ox, oy = info['nearest_obs']
                        bearing = math.atan2(oy - self.robot_y, ox - self.robot_x) \
                                  - (self.robot_theta or 0.0)
                        while bearing > math.pi: bearing -= 2*math.pi
                        while bearing < -math.pi: bearing += 2*math.pi
                    self.recovery.start(now, bearing)
                    rospy.logwarn("[HybridNav] Recovery (dwa=%s no_prog=%s osc=%s) attempt=%d",
                                  dwa_stuck, no_prog, oscillate, self.recovery.attempts)
            else:
                # only clear stuck_since if there is actual progress
                if not no_prog:
                    self.recovery.notify_moving()

            # periodic replan
            if (now - self.last_plan_time) > self.replan_period:
                self.plan_global()

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self.osc.update(now, v, w)
            self.prev_theta = self.robot_theta or self.prev_theta
            rate.sleep()


if __name__ == '__main__':
    try:
        HybridNavigator().control_loop()
    except rospy.ROSInterruptException:
        pass
