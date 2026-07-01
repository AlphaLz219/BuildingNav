#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid navigator for the ASK-3 quadruped.

Pipeline mirrors py_hybrid_planner/hybrid_navigation.py:
  1. ImprovedAStar plans a smoothed global path on the static map.
  2. The global path is used as a reference corridor, not as the command
     trajectory.
  3. OmniDWA picks (vx, vy, w) every cycle and publishes its realtime
     local trajectory for RViz.
  4. Recovery FSM kicks in when DWA reports `stuck` or no progress.

The big interface change vs. the wheeled baseline is the controller
output: the dog firmware does NOT accept geometry_msgs/Twist on
/cmd_vel. Instead it expects three std_msgs/Float32 channels:
  /ask/dog/forward_back  ->  body-frame vx
  /ask/dog/left_right    ->  body-frame vy
  /ask/dog/yaw           ->  body-frame yaw rate
plus two std_msgs/Bool latches (`/ask/dog/start`, `/ask/dog/walk`) that
tell the low-level RL controller to enable walking. We send those
latches once at goal acceptance.

Localisation: by default the ASK-3 Gazebo lidar publishes /scan and AMCL
broadcasts `map -> odom`. The mydog_state_estimator broadcasts
`odom -> base`, so the navigation node consumes the standard
`map -> odom -> base` chain. A ground-truth bridge remains available in
launch files only as a simulation fallback.
"""
import math
import numpy as np
from collections import deque
from threading import Lock

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.realpath(__file__)))
from improved_astar import ImprovedAStar
from omni_dwa import OmniDWA


class ProgressMonitor:
    def __init__(self, window=8.0, min_drop=0.20, min_position_span=0.15):
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
        return (span < self.min_position_span) and (drop < self.min_drop)


class OmniRecoveryFSM:
    """Recovery FSM tuned for an omnidirectional quadruped.

    Sequence: BACK_OFF -> ROTATE -> SIDE_STEP. Backing out first is more
    stable at wall corners because it creates space before yawing the long
    body, and keeps side-stepping as the last backup move.
    """
    IDLE = 0
    SIDE_STEP = 1
    ROTATE = 2
    BACK_OFF = 3

    def __init__(self):
        self.state = self.IDLE
        self.state_start = 0.0
        self.stuck_since = None
        self.attempts = 0
        self.side_dir = 1.0
        self.rot_dir = 1.0
        self.side_duration = 0.4
        self.side_speed = 0.10
        self.rot_duration = 0.6
        self.back_duration = 0.7

    def notify_stuck(self, now):
        if self.stuck_since is None:
            self.stuck_since = now

    def notify_moving(self):
        self.stuck_since = None
        self.attempts = 0
        if self.state != self.IDLE:
            self.state = self.IDLE

    def should_trigger(self, now, threshold=1.5):
        return (self.stuck_since is not None
                and (now - self.stuck_since) > threshold
                and self.state == self.IDLE)

    def start(self, now, away_bearing):
        self.state = self.BACK_OFF
        self.state_start = now
        # alternate dirs across attempts
        if self.attempts % 2 == 0:
            self.side_dir = -1.0 if away_bearing > 0 else 1.0
            self.rot_dir = -1.0 if away_bearing > 0 else 1.0
        else:
            self.side_dir = 1.0 if away_bearing > 0 else -1.0
            self.rot_dir = 1.0 if away_bearing > 0 else -1.0
        self.attempts += 1

    def step(self, now):
        t = now - self.state_start
        sd = self.side_duration * (1.0 + 0.3 * (self.attempts - 1))
        rd = self.rot_duration * (1.0 + 0.3 * (self.attempts - 1))
        bd = self.back_duration * (1.0 + 0.3 * (self.attempts - 1))
        if self.state == self.BACK_OFF:
            if t < bd:
                return -0.10, 0.0, 0.0, False
            self.state = self.ROTATE
            self.state_start = now
            return 0.0, 0.0, 0.0, False
        if self.state == self.ROTATE:
            if t < rd:
                return 0.0, 0.0, 0.55 * self.rot_dir, False
            self.state = self.SIDE_STEP
            self.state_start = now
            return 0.0, 0.0, 0.0, False
        if self.state == self.SIDE_STEP:
            if t < sd:
                return 0.0, self.side_speed * self.side_dir, 0.0, False
            self.state = self.IDLE
            self.stuck_since = None
            return 0.0, 0.0, 0.0, True
        return 0.0, 0.0, 0.0, True


class DogHybridNavigator:
    def __init__(self):
        rospy.init_node('dog_hybrid_navigator', anonymous=False)

        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.25)
        self.local_target_lookahead = rospy.get_param("~lookahead", 0.50)
        self.replan_period = rospy.get_param("~replan_period", 5.0)
        self.periodic_replan = rospy.get_param("~periodic_replan", True)
        self.path_switch_lock_time = rospy.get_param("~path_switch_lock_time", 0.0)
        self.replan_min_translation = rospy.get_param("~replan_min_translation", 0.12)
        self.replan_path_deviation = rospy.get_param("~replan_path_deviation", 0.60)
        self.replan_max_yaw_rate = rospy.get_param("~replan_max_yaw_rate", 0.18)
        self.path_switch_max_heading = math.radians(
            rospy.get_param("~path_switch_max_heading_deg", 70.0))
        self.path_switch_max_lateral = rospy.get_param("~path_switch_max_lateral", 0.30)
        self.path_switch_max_target_jump = rospy.get_param(
            "~path_switch_max_target_jump", 0.55)
        self.emergency_front_dist = rospy.get_param("~emergency_front_dist", 0.16)
        self.emergency_front_half_angle = math.radians(
            rospy.get_param("~emergency_front_half_angle_deg", 22.0))
        self.emergency_front_half_width = rospy.get_param(
            "~emergency_front_half_width", 0.09)
        self.self_filter_enabled = rospy.get_param("~self_filter_enabled", True)
        self.self_filter_x_min = rospy.get_param("~self_filter_x_min", -0.25)
        self.self_filter_x_max = rospy.get_param("~self_filter_x_max", 0.27)
        self.self_filter_half_width = rospy.get_param("~self_filter_half_width", 0.14)
        self.align_to_path = rospy.get_param("~align_to_path", False)
        self.align_start_angle = math.radians(
            rospy.get_param("~align_start_angle_deg", 22.0))
        self.align_finish_angle = math.radians(
            rospy.get_param("~align_finish_angle_deg", 8.0))
        self.align_path_lookahead = rospy.get_param("~align_path_lookahead", 0.60)
        self.align_max_duration = rospy.get_param("~align_max_duration", 3.0)
        self.align_cooldown = rospy.get_param("~align_cooldown", 2.0)
        self.align_turn_clearance = rospy.get_param("~align_turn_clearance", 0.24)
        self.align_rear_clearance = rospy.get_param("~align_rear_clearance", 0.20)
        self.align_front_space = rospy.get_param("~align_front_space", 0.28)
        self.align_side_clearance = rospy.get_param("~align_side_clearance", 0.18)
        self.align_back_speed = rospy.get_param("~align_back_speed", 0.10)
        self.align_side_speed = rospy.get_param("~align_side_speed", 0.08)
        self.align_yaw_speed = rospy.get_param("~align_yaw_speed", 0.65)
        self.progress_window = rospy.get_param("~progress_window", 8.0)
        self.progress_min_drop = rospy.get_param("~progress_min_drop", 0.20)
        self.no_progress_recovery_clearance = rospy.get_param(
            "~no_progress_recovery_clearance", 0.45)
        self.no_progress_recovery_delay = rospy.get_param(
            "~no_progress_recovery_delay", 3.0)
        self.post_recovery_dwa_grace = rospy.get_param(
            "~post_recovery_dwa_grace", 2.5)
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.global_frame = rospy.get_param("~global_frame", "map")
        # publishing rate of velocity setpoints (Hz)
        self.cmd_rate = rospy.get_param("~cmd_rate", 25.0)
        # cap of body-frame speed for safety
        self.max_speed = rospy.get_param("~max_speed", 0.75)
        self.auto_enable_idle = rospy.get_param("~auto_enable_idle", False)
        self.enable_low_level_gait = rospy.get_param("~enable_low_level_gait", True)
        self.max_laser_points = int(rospy.get_param("~max_laser_points", 240))
        self.astar_clearance_radius = rospy.get_param(
            "~astar_clearance_radius", 0.50)
        self.astar_clearance_weight = rospy.get_param(
            "~astar_clearance_weight", 1.20)
        self.astar_turn_weight = rospy.get_param(
            "~astar_turn_weight", 0.08)
        self.astar_shortcut_clearance = rospy.get_param(
            "~astar_shortcut_clearance", 0.30)
        self.astar_resample_spacing = rospy.get_param(
            "~astar_resample_spacing", 0.10)
        self.astar_chaikin_iterations = int(rospy.get_param(
            "~astar_chaikin_iterations", 2))
        self.dwa_heading_align = rospy.get_param("~dwa_heading_align", True)
        self.dwa_align_start_angle = math.radians(
            rospy.get_param("~dwa_align_start_angle_deg", 42.0))
        self.dwa_align_lookahead = rospy.get_param(
            "~dwa_align_lookahead", 0.45)
        self.dwa_align_turn_clearance = rospy.get_param(
            "~dwa_align_turn_clearance", 0.34)
        self.dwa_align_rear_clearance = rospy.get_param(
            "~dwa_align_rear_clearance", 0.22)
        self.dwa_align_front_space = rospy.get_param(
            "~dwa_align_front_space", 0.26)
        self.dwa_align_side_scale = rospy.get_param(
            "~dwa_align_side_scale", 0.18)
        self.dwa_align_creep_speed = rospy.get_param(
            "~dwa_align_creep_speed", 0.24)
        self.dwa_align_yaw_speed = rospy.get_param(
            "~dwa_align_yaw_speed", 1.15)
        self.min_cruise_vx = rospy.get_param("~min_cruise_vx", 0.34)
        self.replan_cruise_vx = rospy.get_param("~replan_cruise_vx", 0.38)
        self.post_replan_fast_grace = rospy.get_param(
            "~post_replan_fast_grace", 1.3)
        self.cruise_front_clearance = rospy.get_param(
            "~cruise_front_clearance", 0.60)
        self.cruise_max_yaw_rate = rospy.get_param(
            "~cruise_max_yaw_rate", 0.45)
        self.cruise_lateral_limit = rospy.get_param(
            "~cruise_lateral_limit", 0.16)
        self.use_cmd_velocity_state = rospy.get_param(
            "~use_cmd_velocity_state", True)
        self.cmd_velocity_blend = rospy.get_param(
            "~cmd_velocity_blend", 0.75)

        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None
        self.body_vx = 0.0
        self.body_vy = 0.0
        self.body_w = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_w = 0.0
        self.prev_theta = 0.0

        self.laser_points = []
        self.front_min_range = float('inf')
        self.laser_lock = Lock()

        self.global_smooth_path = []
        self.global_key_points = []
        self.current_path_idx = 0
        self.current_key_idx = 0
        self.current_goal = None
        self.navigating = False
        self.last_plan_time = 0.0
        self.path_lock_until = 0.0
        self.last_replan_pose = None
        self.aligning_to_path = False
        self.aligning_since = None
        self.align_cooldown_until = 0.0
        self.post_recovery_dwa_until = 0.0
        self.post_replan_fast_until = 0.0

        self.map_ready = False
        self.costmap = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None

        self.dwa = OmniDWA(
            robot_radius=rospy.get_param("~robot_radius", 0.30),
            hard_radius=rospy.get_param("~hard_radius", 0.13),
            max_vx=rospy.get_param("~max_vx", 0.75),
            min_vx=rospy.get_param("~min_vx", -0.12),
            max_vy=rospy.get_param("~max_vy", 0.26),
            min_vy=rospy.get_param("~min_vy", -0.26),
            max_omega=rospy.get_param("~max_omega", 1.35),
            min_omega=-rospy.get_param("~max_omega", 1.35),
            max_accel_xy=rospy.get_param("~max_accel_xy", 2.8),
            max_domega=rospy.get_param("~max_domega", 3.4),
            predict_time=rospy.get_param("~predict_time", 1.35),
            max_decel_xy=rospy.get_param("~max_decel_xy", 2.0),
            brake_margin=rospy.get_param("~brake_margin", 0.03),
            path_track_weight=rospy.get_param("~dwa_path_track_weight", 0.16),
            path_corridor_width=rospy.get_param("~dwa_path_corridor_width", 0.55),
        )
        self.recovery = OmniRecoveryFSM()
        self.progress = ProgressMonitor(
            window=self.progress_window,
            min_drop=self.progress_min_drop,
            min_position_span=rospy.get_param('~progress_min_span', 0.15))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         self.goal_cb, queue_size=1)

        # Quadruped command interface
        self.fb_pub = rospy.Publisher("/ask/dog/forward_back", Float32, queue_size=1)
        self.lr_pub = rospy.Publisher("/ask/dog/left_right",   Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher("/ask/dog/yaw",         Float32, queue_size=1)
        self.start_pub = rospy.Publisher("/ask/dog/start", Bool, queue_size=1, latch=True)
        self.walk_pub  = rospy.Publisher("/ask/dog/walk",  Bool, queue_size=1, latch=True)
        self.stand_pub = rospy.Publisher("/ask/dog/stand", Bool, queue_size=1, latch=True)

        # Visualization
        self.global_path_pub = rospy.Publisher(
            "/dog_global_path", Path, queue_size=1, latch=True)
        self.key_pts_pub = rospy.Publisher(
            "/dog_key_points", MarkerArray, queue_size=1, latch=True)
        self.local_target_pub = rospy.Publisher(
            "/dog_local_target", Marker, queue_size=1)
        self.dwa_path_pub = rospy.Publisher(
            "/dog_dwa_path", Path, queue_size=1)

        rospy.loginfo("[DogNav] Initialized, waiting for map and TF (base_frame=%s)",
                      self.base_frame)

    # ------------------------------------------------------------------
    def _is_self_point(self, px, py):
        if (not self.self_filter_enabled or self.robot_x is None
                or self.robot_y is None or self.robot_theta is None):
            return False
        dx = px - self.robot_x
        dy = py - self.robot_y
        ct, st = math.cos(self.robot_theta), math.sin(self.robot_theta)
        bx = dx * ct + dy * st
        by = -dx * st + dy * ct
        return (self.self_filter_x_min <= bx <= self.self_filter_x_max
                and abs(by) <= self.self_filter_half_width)

    @staticmethod
    def _wrap_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _point_to_body(self, x, y):
        dx = x - self.robot_x
        dy = y - self.robot_y
        ct, st = math.cos(self.robot_theta), math.sin(self.robot_theta)
        return dx * ct + dy * st, -dx * st + dy * ct

    def _body_clearances(self, obstacle_points):
        clear = {
            'front': 2.0,
            'rear': 2.0,
            'left': 2.0,
            'right': 2.0,
            'turn': 2.0,
            'nearest': None,
        }
        if self.robot_x is None or self.robot_theta is None:
            return clear
        corridor_half_width = 0.14
        body_half_length = 0.25
        nearest_d = float('inf')
        for ox, oy in obstacle_points:
            bx, by = self._point_to_body(ox, oy)
            d = math.hypot(bx, by)
            if d < nearest_d:
                nearest_d = d
                clear['nearest'] = (ox, oy)
            if abs(by) < corridor_half_width:
                if bx > 0.0:
                    clear['front'] = min(clear['front'], bx)
                else:
                    clear['rear'] = min(clear['rear'], -bx)
            if abs(bx) < body_half_length:
                if by > 0.0:
                    clear['left'] = min(clear['left'], by)
                else:
                    clear['right'] = min(clear['right'], -by)
        if nearest_d < float('inf'):
            clear['turn'] = min(nearest_d, 2.0)
        return clear

    def _path_heading(self):
        if len(self.global_smooth_path) >= 2 and self.robot_x is not None:
            nearest = self._tracked_path_index(self.global_smooth_path,
                                               update=False)
            ahead = nearest
            travelled = 0.0
            while ahead + 1 < len(self.global_smooth_path) and travelled < self.align_path_lookahead:
                x0, y0 = self.global_smooth_path[ahead]
                x1, y1 = self.global_smooth_path[ahead + 1]
                travelled += math.hypot(x1 - x0, y1 - y0)
                ahead += 1
            if ahead != nearest:
                x0, y0 = self.global_smooth_path[nearest]
                x1, y1 = self.global_smooth_path[ahead]
                if math.hypot(x1 - x0, y1 - y0) > 1e-3:
                    return math.atan2(y1 - y0, x1 - x0)
        target = self.get_local_target()
        if target is None or self.robot_x is None:
            return None
        return math.atan2(target[1] - self.robot_y, target[0] - self.robot_x)

    def _nearest_obstacle_bearing(self, obstacle_points):
        if not obstacle_points or self.robot_x is None:
            return 0.0
        nearest = min(obstacle_points,
                      key=lambda p: math.hypot(p[0] - self.robot_x,
                                               p[1] - self.robot_y))
        return self._wrap_angle(
            math.atan2(nearest[1] - self.robot_y, nearest[0] - self.robot_x)
            - self.robot_theta)

    def _path_alignment_cmd(self, target, obstacle_points):
        now = rospy.get_time()
        if now < self.align_cooldown_until:
            self.aligning_to_path = False
            self.aligning_since = None
            return None, False

        if (not self.align_to_path or self.robot_x is None
                or self.robot_theta is None or target is None):
            self.aligning_to_path = False
            self.aligning_since = None
            return None, False

        heading = self._path_heading()
        if heading is None:
            self.aligning_to_path = False
            self.aligning_since = None
            return None, False

        err = self._wrap_angle(heading - self.robot_theta)
        if abs(err) < self.align_finish_angle:
            self.aligning_to_path = False
            self.aligning_since = None
            return None, False
        if not self.aligning_to_path and abs(err) < self.align_start_angle:
            self.aligning_since = None
            return None, False

        target_bx, target_by = self._point_to_body(target[0], target[1])

        clear = self._body_clearances(obstacle_points)
        enough_turn_space = (clear['turn'] > self.align_turn_clearance and
                             clear['rear'] > self.align_rear_clearance)
        if not enough_turn_space:
            self.aligning_to_path = False
            self.aligning_since = None
            front_blocked = clear['front'] < self.align_front_space
            rear_blocked = clear['rear'] < self.align_rear_clearance
            side_blocked = min(clear['left'], clear['right']) < self.align_side_clearance
            boxed_in = front_blocked and (rear_blocked or side_blocked)
            if boxed_in and abs(err) > self.align_start_angle:
                return None, True
            return None, False

        if not self.aligning_to_path:
            self.aligning_since = now
        elif (self.aligning_since is not None and
                now - self.aligning_since > self.align_max_duration):
            rospy.logwarn_throttle(
                1.0,
                "[DogNav] Path alignment timeout, hand control back to DWA")
            self.aligning_to_path = False
            self.aligning_since = None
            self.align_cooldown_until = now + self.align_cooldown
            return None, False
        self.aligning_to_path = True
        side_dir = 1.0 if target_by >= 0.0 else -1.0
        side_clearance = clear['left'] if side_dir > 0.0 else clear['right']

        w = max(-self.align_yaw_speed,
                min(self.align_yaw_speed, 1.5 * err))
        vx = 0.0
        if (abs(err) > math.radians(45.0) or
                clear['front'] < self.align_front_space):
            vx = -self.align_back_speed
        vy = 0.0
        if abs(target_by) > 0.12 and side_clearance > self.align_side_clearance:
            vy = side_dir * self.align_side_speed

        return (vx, vy, w), False

    def map_cb(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.costmap = data.copy()
        self.map_ready = True
        rospy.loginfo("[DogNav] Map %dx%d res=%.3f origin=(%.2f,%.2f)",
                      self.map_width, self.map_height, self.map_resolution,
                      self.map_origin[0], self.map_origin[1])

    def _update_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.1))
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
        # body-frame twist: linear.x = forward, linear.y = lateral
        self.body_vx = msg.twist.twist.linear.x
        self.body_vy = msg.twist.twist.linear.y
        self.body_w = msg.twist.twist.angular.z
        self._update_pose_from_tf()

    def scan_cb(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id,
                rospy.Time(0), rospy.Duration(0.1))
        except Exception:
            return
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        cy, sy = math.cos(yaw), math.sin(yaw)

        points = []
        front_min = float('inf')
        step = 1
        if self.max_laser_points > 0:
            step = max(1, len(msg.ranges) // self.max_laser_points)
        for idx, r in enumerate(msg.ranges):
            angle = msg.angle_min + idx * msg.angle_increment
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                px = tx + lx * cy - ly * sy
                py = ty + lx * sy + ly * cy
                if self._is_self_point(px, py):
                    continue
                if (lx > 0.0
                        and abs(angle) < self.emergency_front_half_angle
                        and abs(ly) < self.emergency_front_half_width
                        and r < front_min):
                    front_min = r
                if idx % step != 0:
                    continue
                points.append((px, py))

        with self.laser_lock:
            self.laser_points = points
            self.front_min_range = front_min

    def goal_cb(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("[DogNav] Goal received (%.2f, %.2f)", *self.current_goal)
        self.progress.reset()
        self.recovery = OmniRecoveryFSM()
        self.aligning_to_path = False
        self.aligning_since = None
        self.align_cooldown_until = 0.0
        self.post_recovery_dwa_until = 0.0
        # Make sure walking gait is enabled before sending velocities.
        if self.enable_low_level_gait:
            self._enable_walking()
        self.plan_global(force=True, reason="goal")

    def _enable_walking(self):
        m = Bool()
        m.data = True
        self.start_pub.publish(m)
        self.walk_pub.publish(m)

    def _stand(self):
        m = Bool()
        m.data = True
        self.stand_pub.publish(m)

    def _send_cmd(self, vx, vy, w):
        # Clamp body speed for safety
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale
        self.fb_pub.publish(Float32(data=float(vx)))
        self.lr_pub.publish(Float32(data=float(vy)))
        self.yaw_pub.publish(Float32(data=float(w)))
        self.cmd_vx = float(vx)
        self.cmd_vy = float(vy)
        self.cmd_w = float(w)

    # ------------------------------------------------------------------
    def _dwa_velocity_state(self, front_range):
        if not self.use_cmd_velocity_state:
            return self.body_vx, self.body_vy, self.body_w
        if front_range < max(self.emergency_front_dist + 0.12, 0.30):
            return self.body_vx, self.body_vy, self.body_w

        vx = self.body_vx
        if self.cmd_vx > 0.0:
            vx = max(vx, self.cmd_velocity_blend * self.cmd_vx)

        vy = self.body_vy
        cmd_vy_blend = self.cmd_velocity_blend * self.cmd_vy
        if abs(cmd_vy_blend) > abs(vy):
            vy = cmd_vy_blend

        w = self.body_w
        cmd_w_blend = self.cmd_velocity_blend * self.cmd_w
        if abs(cmd_w_blend) > abs(w):
            w = cmd_w_blend
        return vx, vy, w

    def _should_defer_replan(self, now, reason):
        return (reason == "periodic" and self.path_lock_until > 0.0
                and now < self.path_lock_until)

    def _nearest_path_distance(self, path):
        if self.robot_x is None or not path:
            return float('inf')
        return min(math.hypot(px - self.robot_x, py - self.robot_y)
                   for px, py in path)

    @staticmethod
    def _path_window_bounds(path, center_idx, back_dist=0.25, forward_dist=1.20):
        if not path:
            return 0, 0
        center_idx = max(0, min(center_idx, len(path) - 1))
        lo = center_idx
        dist = 0.0
        while lo > 0 and dist < back_dist:
            x0, y0 = path[lo]
            x1, y1 = path[lo - 1]
            dist += math.hypot(x1 - x0, y1 - y0)
            lo -= 1
        hi = center_idx
        dist = 0.0
        while hi + 1 < len(path) and dist < forward_dist:
            x0, y0 = path[hi]
            x1, y1 = path[hi + 1]
            dist += math.hypot(x1 - x0, y1 - y0)
            hi += 1
        return lo, hi

    def _tracked_path_index(self, path, update=True):
        if self.robot_x is None or not path:
            return 0
        if not self.global_smooth_path or path is not self.global_smooth_path:
            return min(range(len(path)),
                       key=lambda i: math.hypot(path[i][0] - self.robot_x,
                                                path[i][1] - self.robot_y))

        center = max(0, min(self.current_path_idx, len(path) - 1))
        lo, hi = self._path_window_bounds(path, center)
        nearest = min(range(lo, hi + 1),
                      key=lambda i: math.hypot(path[i][0] - self.robot_x,
                                               path[i][1] - self.robot_y))
        if update:
            self.current_path_idx = max(self.current_path_idx, nearest)
            return self.current_path_idx
        return nearest

    def _target_on_path(self, path, lookahead=None, track_progress=False):
        if self.robot_x is None or not path:
            return None
        if lookahead is None:
            lookahead = self.local_target_lookahead

        if track_progress:
            nearest = self._tracked_path_index(path, update=True)
        else:
            nearest = min(range(len(path)),
                          key=lambda i: math.hypot(path[i][0] - self.robot_x,
                                                   path[i][1] - self.robot_y))

        target_idx = nearest
        travelled = 0.0
        while target_idx + 1 < len(path) and travelled < lookahead:
            x0, y0 = path[target_idx]
            x1, y1 = path[target_idx + 1]
            travelled += math.hypot(x1 - x0, y1 - y0)
            target_idx += 1
        return path[target_idx]

    @staticmethod
    def _target_from_index(path, start_idx, lookahead):
        if not path:
            return None
        target_idx = max(0, min(start_idx, len(path) - 1))
        travelled = 0.0
        while target_idx + 1 < len(path) and travelled < lookahead:
            x0, y0 = path[target_idx]
            x1, y1 = path[target_idx + 1]
            travelled += math.hypot(x1 - x0, y1 - y0)
            target_idx += 1
        return path[target_idx]

    @staticmethod
    def _heading_from_index(path, start_idx, lookahead):
        if not path:
            return None
        start_idx = max(0, min(start_idx, len(path) - 1))
        ahead = start_idx
        travelled = 0.0
        while ahead + 1 < len(path) and travelled < lookahead:
            x0, y0 = path[ahead]
            x1, y1 = path[ahead + 1]
            travelled += math.hypot(x1 - x0, y1 - y0)
            ahead += 1
        if ahead == start_idx:
            return None
        x0, y0 = path[start_idx]
        x1, y1 = path[ahead]
        if math.hypot(x1 - x0, y1 - y0) < 1e-3:
            return None
        return math.atan2(y1 - y0, x1 - x0)

    def _path_heading_for_path(self, path, lookahead=None):
        if self.robot_x is None or not path:
            return None
        if lookahead is None:
            lookahead = self.align_path_lookahead

        nearest = 0
        nearest_d = float('inf')
        for i, (px, py) in enumerate(path):
            d = math.hypot(px - self.robot_x, py - self.robot_y)
            if d < nearest_d:
                nearest_d = d
                nearest = i

        ahead = nearest
        travelled = 0.0
        while ahead + 1 < len(path) and travelled < lookahead:
            x0, y0 = path[ahead]
            x1, y1 = path[ahead + 1]
            travelled += math.hypot(x1 - x0, y1 - y0)
            ahead += 1
        if ahead == nearest:
            return None
        x0, y0 = path[nearest]
        x1, y1 = path[ahead]
        if math.hypot(x1 - x0, y1 - y0) < 1e-3:
            return None
        return math.atan2(y1 - y0, x1 - x0)

    def _candidate_path_is_unstable_switch(self, new_path):
        if (not self.global_smooth_path or self.robot_x is None
                or self.robot_theta is None):
            return False

        old_idx = max(0, min(self.current_path_idx,
                             len(self.global_smooth_path) - 1))
        old_heading = self._heading_from_index(
            self.global_smooth_path, old_idx, self.align_path_lookahead)
        new_heading = self._path_heading_for_path(
            new_path, self.align_path_lookahead)
        heading_jump = 0.0
        if old_heading is not None and new_heading is not None:
            heading_jump = abs(self._wrap_angle(new_heading - old_heading))

        old_target = self._target_from_index(
            self.global_smooth_path, old_idx, self.local_target_lookahead)
        new_target = self._target_on_path(
            new_path, self.local_target_lookahead)
        lateral_jump = 0.0
        side_flip = False
        target_jump = 0.0
        if old_target is not None and new_target is not None:
            old_y = self._point_to_body(old_target[0], old_target[1])[1]
            new_y = self._point_to_body(new_target[0], new_target[1])[1]
            lateral_jump = abs(new_y - old_y)
            side_flip = old_y * new_y < -0.01
            target_jump = math.hypot(new_target[0] - old_target[0],
                                     new_target[1] - old_target[1])

        lateral_jump_large = (lateral_jump > self.path_switch_max_lateral
                              and target_jump > 0.25
                              and heading_jump > math.radians(20.0))
        side_jump = side_flip and lateral_jump > self.path_switch_max_lateral
        heading_jump_large = heading_jump > self.path_switch_max_heading
        target_jump_large = target_jump > self.path_switch_max_target_jump
        return (side_jump or lateral_jump_large or
                (heading_jump_large and target_jump > 0.25) or
                (target_jump_large and heading_jump > math.radians(20.0)))

    def plan_global(self, force=False, reason="manual"):
        now = rospy.get_time()
        if not self.map_ready:
            rospy.logwarn_throttle(2.0, "[DogNav] Map not ready")
            return False
        if self.robot_x is None:
            rospy.logwarn_throttle(2.0, "[DogNav] Pose not ready")
            return False
        if not force and self._should_defer_replan(now, reason):
            self.last_plan_time = now
            rospy.loginfo_throttle(
                2.0,
                "[DogNav] Defer %s replan while current path is stabilizing",
                reason)
            return False
        start = (self.robot_x, self.robot_y)
        goal = self.current_goal
        if goal is None:
            return False
        rospy.loginfo("[DogNav] Planning[%s] (%.2f,%.2f) -> (%.2f,%.2f)",
                      reason, start[0], start[1], goal[0], goal[1])
        astar = ImprovedAStar(self.costmap, self.map_resolution,
                              self.map_origin, self.map_width, self.map_height,
                              inflate_radius=rospy.get_param('~inflate_radius', 0.22),
                              clearance_radius=self.astar_clearance_radius,
                              clearance_weight=self.astar_clearance_weight,
                              turn_weight=self.astar_turn_weight,
                              shortcut_clearance=self.astar_shortcut_clearance,
                              resample_spacing=self.astar_resample_spacing,
                              chaikin_iterations=self.astar_chaikin_iterations)
        smooth_path, key_points = astar.plan(start, goal)
        if not smooth_path:
            rospy.logerr("[DogNav] Global planning FAILED")
            return False
        if (not force and reason == "periodic"
                and self._candidate_path_is_unstable_switch(smooth_path)):
            self.last_plan_time = now
            rospy.logwarn_throttle(
                2.0,
                "[DogNav] Reject large periodic path switch; keep tracking current path")
            return False
        self.global_smooth_path = smooth_path
        self.global_key_points = key_points
        self.current_path_idx = 0
        self.current_key_idx = 0
        self.navigating = True
        self.aligning_to_path = False
        self.aligning_since = None
        self.align_cooldown_until = 0.0
        self.prev_theta = self.robot_theta or 0.0
        self.last_plan_time = now
        self.path_lock_until = now + self.path_switch_lock_time
        self.post_replan_fast_until = now + self.post_replan_fast_grace
        self.progress.reset()
        self.last_replan_pose = (self.robot_x, self.robot_y, self.robot_theta or 0.0)
        self.publish_global_path()
        self.publish_key_points()
        rospy.loginfo("[DogNav] Global path OK, %d key points",
                      len(key_points))
        return True

    def publish_global_path(self):
        p = Path()
        p.header.frame_id = self.global_frame
        p.header.stamp = rospy.Time.now()
        for (x, y) in self.global_smooth_path:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.global_path_pub.publish(p)

    def publish_key_points(self):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, (x, y) in enumerate(self.global_key_points):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "key_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 1.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.key_pts_pub.publish(ma)

    def publish_local_target(self, x, y):
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "local_target"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = 0.25
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.color.g = 1.0
        m.color.a = 1.0
        self.local_target_pub.publish(m)

    def publish_dwa_path(self, trajectory):
        p = Path()
        p.header.frame_id = self.global_frame
        p.header.stamp = rospy.Time.now()
        for x, y, theta in trajectory:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(theta * 0.5)
            ps.pose.orientation.w = math.cos(theta * 0.5)
            p.poses.append(ps)
        self.dwa_path_pub.publish(p)

    def _dwa_trajectory_heading(self, trajectory):
        if trajectory is None or len(trajectory) < 2:
            return None
        x0, y0, _ = trajectory[0]
        travelled = 0.0
        px, py = x0, y0
        best = None
        for x, y, _ in trajectory[1:]:
            travelled += math.hypot(x - px, y - py)
            px, py = x, y
            if math.hypot(x - x0, y - y0) > 0.06:
                best = (x, y)
            if travelled >= self.dwa_align_lookahead and best is not None:
                break
        if best is None:
            return None
        return math.atan2(best[1] - y0, best[0] - x0)

    def _align_head_to_dwa_path(self, vx, vy, w, trajectory, obstacle_points):
        if (not self.dwa_heading_align or self.robot_theta is None
                or self.robot_x is None or self.robot_y is None):
            return vx, vy, w, False

        heading = self._dwa_trajectory_heading(trajectory)
        if heading is None:
            return vx, vy, w, False
        err = self._wrap_angle(heading - self.robot_theta)
        if abs(err) < self.dwa_align_start_angle:
            return vx, vy, w, False

        clear = self._body_clearances(obstacle_points)
        enough_space = (
            clear['turn'] > self.dwa_align_turn_clearance and
            clear['rear'] > self.dwa_align_rear_clearance and
            clear['front'] > self.dwa_align_front_space)
        if not enough_space:
            return vx, vy, w, False

        w_align = max(-self.dwa_align_yaw_speed,
                      min(self.dwa_align_yaw_speed, 1.8 * err))
        if abs(err) > math.radians(45.0):
            vx = self.dwa_align_creep_speed
        else:
            vx = min(max(vx, self.dwa_align_creep_speed),
                     max(0.24, self.dwa_align_creep_speed))
        vy *= self.dwa_align_side_scale
        return vx, vy, w_align, True

    def _apply_cruise_speed_floor(self, vx, vy, w, target, obstacle_points, now):
        if (target is None or self.robot_x is None or self.robot_theta is None
                or vx < -0.02 or abs(w) > self.cruise_max_yaw_rate
                or abs(vy) > self.cruise_lateral_limit):
            return vx

        target_bx, target_by = self._point_to_body(target[0], target[1])
        if target_bx < 0.25 or abs(target_by) > 0.35:
            return vx

        clear = self._body_clearances(obstacle_points)
        front_clear = min(clear['front'], self.front_min_range)
        if front_clear < self.cruise_front_clearance:
            return vx
        floor = self.replan_cruise_vx if now < self.post_replan_fast_until else self.min_cruise_vx
        return max(vx, floor)

    def get_local_target(self):
        path = self.global_smooth_path if self.global_smooth_path else self.global_key_points
        if not path:
            return None
        target = self._target_on_path(
            path, self.local_target_lookahead,
            track_progress=(path is self.global_smooth_path))
        if target is None:
            return None
        if path is self.global_key_points:
            self.current_key_idx = min(
                range(len(path)),
                key=lambda i: math.hypot(path[i][0] - self.robot_x,
                                         path[i][1] - self.robot_y))
        return target

    # ------------------------------------------------------------------
    def control_loop(self):
        rate = rospy.Rate(self.cmd_rate)
        # Keep walking disabled while idle by default. Enabling the RL gait
        # before the body has settled can make ASK-3 step away from its spawn
        # pose or tip over near walls.
        last_enable = 0.0
        while not rospy.is_shutdown():
            now = rospy.get_time()
            self._update_pose_from_tf()

            if not self.navigating or self.robot_x is None or self.current_goal is None:
                self.publish_dwa_path([])
                self._send_cmd(0.0, 0.0, 0.0)
                if (self.enable_low_level_gait and self.auto_enable_idle
                        and now - last_enable > 2.0):
                    self._enable_walking()
                    last_enable = now
                rate.sleep()
                continue

            dg = math.hypot(self.robot_x - self.current_goal[0],
                            self.robot_y - self.current_goal[1])
            self.progress.update(now, self.robot_x, self.robot_y, dg)

            if dg < self.goal_tolerance:
                rospy.loginfo("[DogNav] *** GOAL REACHED ***")
                self._send_cmd(0.0, 0.0, 0.0)
                self.publish_dwa_path([])
                self.navigating = False
                self.aligning_to_path = False
                self.aligning_since = None
                self.align_cooldown_until = 0.0
                self.post_recovery_dwa_until = 0.0
                self.recovery = OmniRecoveryFSM()
                self.progress.reset()
                rate.sleep()
                continue

            # recovery first
            if self.recovery.state != OmniRecoveryFSM.IDLE:
                vx, vy, w, finished = self.recovery.step(now)
                self.publish_dwa_path([])
                self._send_cmd(vx, vy, w)
                if finished:
                    rospy.logwarn("[DogNav] Recovery finished (attempt %d) - replanning",
                                  self.recovery.attempts)
                    self.plan_global(force=True, reason="recovery")
                    self.progress.reset()
                    self.recovery.notify_moving()
                    self.post_recovery_dwa_until = now + self.post_recovery_dwa_grace
                    self.align_cooldown_until = max(
                        self.align_cooldown_until,
                        self.post_recovery_dwa_until)
                    rospy.loginfo(
                        "[DogNav] Recovery cleared; DWA has %.1fs grace window",
                        self.post_recovery_dwa_grace)
                rate.sleep()
                continue

            with self.laser_lock:
                front = self.front_min_range
                obs = list(self.laser_points)

            front_blocked_hard = front < self.emergency_front_dist
            if front_blocked_hard:
                rospy.logwarn_throttle(1.0,
                    "[DogNav] Front close %.2fm; blocking forward vx only", front)

            path_dev = self._nearest_path_distance(self.global_smooth_path)
            if (self.periodic_replan and
                    path_dev > self.replan_path_deviation and
                    (now - self.last_plan_time) > min(self.replan_period, 1.5)):
                rospy.logwarn_throttle(
                    1.0,
                    "[DogNav] Robot %.2fm away from active path, replanning",
                    path_dev)
                self.plan_global(force=True, reason="path_deviation")

            target = self.get_local_target()
            if target is None:
                self.publish_dwa_path([])
                self._send_cmd(0.0, 0.0, 0.0)
                rate.sleep()
                continue
            self.publish_local_target(*target)

            align_cmd, needs_escape = self._path_alignment_cmd(target, obs)
            if needs_escape:
                bearing = self._nearest_obstacle_bearing(obs)
                self.recovery.start(now, bearing)
                vx, vy, w, _ = self.recovery.step(now)
                self._send_cmd(vx, vy, w)
                rospy.logwarn_throttle(
                    1.0,
                    "[DogNav] Alignment space blocked, escaping before yaw alignment")
                rate.sleep()
                continue
            if align_cmd is not None:
                vx, vy, w = align_cmd
                if front_blocked_hard and vx > 0.0:
                    vx = 0.0
                self._send_cmd(vx, vy, w)
                rospy.loginfo_throttle(
                    1.0,
                    "[DogNav] Aligning to planned path: vx=%.2f vy=%.2f w=%.2f",
                    vx, vy, w)
                rate.sleep()
                continue

            vx_state, vy_state, w_state = self._dwa_velocity_state(front)
            vx, vy, w, info = self.dwa.plan(
                current_x=self.robot_x,
                current_y=self.robot_y,
                current_theta=self.robot_theta,
                vx_curr=vx_state,
                vy_curr=vy_state,
                w_curr=w_state,
                goal_x=target[0],
                goal_y=target[1],
                obstacle_points=obs,
                prev_theta=self.prev_theta,
                global_path=self.global_smooth_path,
                path_index=self.current_path_idx,
            )
            dwa_traj = info.get('trajectory', [])
            self.publish_dwa_path(dwa_traj)
            dwa_path_aligning = False
            fast_replan_window = now < self.post_replan_fast_until
            if not info.get('stuck', False) and not fast_replan_window:
                vx, vy, w, dwa_path_aligning = self._align_head_to_dwa_path(
                    vx, vy, w, dwa_traj, obs)
                if dwa_path_aligning:
                    rospy.loginfo_throttle(
                        1.0,
                        "[DogNav] Align head to DWA path: vx=%.2f vy=%.2f w=%.2f",
                        vx, vy, w)
            if not dwa_path_aligning:
                vx = self._apply_cruise_speed_floor(vx, vy, w, target, obs, now)

            if front_blocked_hard and vx > 0.0:
                vx = 0.0

            dwa_stuck = info.get('stuck', False)
            raw_no_prog = self.progress.no_progress(now)
            in_post_recovery_grace = now < self.post_recovery_dwa_until
            no_prog = (raw_no_prog and not in_post_recovery_grace
                       and not dwa_path_aligning)
            if raw_no_prog and in_post_recovery_grace:
                self.recovery.notify_moving()
                rospy.logwarn_throttle(
                    2.0,
                    "[DogNav] Suppress no_progress after recovery; keep DWA control")
            if raw_no_prog and dwa_path_aligning:
                self.recovery.notify_moving()
            front_blocked_stop = (front_blocked_hard and abs(vx) < 0.03
                                  and abs(vy) < 0.03 and abs(w) < 0.03)
            recovery_clear = self._body_clearances(obs)
            near_recovery_obstacle = (
                recovery_clear['turn'] < self.no_progress_recovery_clearance or
                recovery_clear['front'] < self.align_front_space or
                min(recovery_clear['left'], recovery_clear['right']) <
                self.align_side_clearance)
            no_prog_recovery = no_prog and near_recovery_obstacle

            if no_prog and not near_recovery_obstacle:
                self.recovery.notify_moving()
                rospy.logwarn_throttle(
                    2.0,
                    "[DogNav] No progress but clearance is open; keep DWA control")

            if dwa_stuck or no_prog_recovery or front_blocked_stop:
                self.recovery.notify_stuck(now)
                trig = 0.8 if front_blocked_stop else (
                    self.no_progress_recovery_delay if no_prog_recovery else 2.5)
                if self.recovery.should_trigger(now, threshold=trig):
                    bearing = 0.0
                    if info.get('nearest_obs') is not None:
                        ox, oy = info['nearest_obs']
                        bearing = math.atan2(oy - self.robot_y,
                                             ox - self.robot_x) - (self.robot_theta or 0.0)
                        while bearing > math.pi:
                            bearing -= 2 * math.pi
                        while bearing < -math.pi:
                            bearing += 2 * math.pi
                    self.recovery.start(now, bearing)
                    rospy.logwarn("[DogNav] Recovery (dwa=%s no_prog=%s near_obs=%s front_stop=%s) attempt=%d",
                                  dwa_stuck, no_prog_recovery,
                                  near_recovery_obstacle, front_blocked_stop,
                                  self.recovery.attempts)
            else:
                if not no_prog_recovery:
                    self.recovery.notify_moving()

            if self.periodic_replan and (now - self.last_plan_time) > self.replan_period:
                if abs(w) > self.replan_max_yaw_rate:
                    self.last_plan_time = now
                    rospy.loginfo_throttle(
                        2.0,
                        "[DogNav] Skip periodic replan while yaw command is active")
                else:
                    self.plan_global(force=False, reason="periodic")

            self._send_cmd(vx, vy, w)
            self.prev_theta = self.robot_theta or self.prev_theta
            rate.sleep()


if __name__ == '__main__':
    try:
        DogHybridNavigator().control_loop()
    except rospy.ROSInterruptException:
        pass
