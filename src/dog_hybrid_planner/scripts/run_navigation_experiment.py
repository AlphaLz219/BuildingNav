#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drives a sequence of /move_base_simple/goal poses against a running
dog_hybrid_navigator and records the resulting (x, y, t) trajectory of
the ASK-3 from /gazebo/model_states.

Run AFTER `roslaunch dog_hybrid_planner dog_all.launch` is up.

Goals were picked to remain inside corridors that survive the dog's
0.22 m inflation; sending the dog through the tight gap in the middle
of the wall layout would block A*.

Output:
  experiments/results/dog_integration_trajectory.log
  experiments/results/dog_integration.json
"""
import os
import sys
import math
import time
import json
import threading

import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.realpath(__file__)))

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "experiments", "results")

# Multi-leg route for ask3_lab.world. Goals are selected to remain reachable
# under the 0.30-0.32 m ASK-3 safety inflation.
GOALS = [
    (10.8, 7.3, 0.0),
    (10.8, 1.1, -1.5708),
    (1.3, 7.2, 3.1416),
]
GOAL_TIMEOUT = 220.0
ARRIVAL_TOL = 0.45


class Recorder:
    def __init__(self):
        rospy.init_node('dog_navigation_experiment', anonymous=True)
        self.model_name = rospy.get_param('~model_name', 'mydog')
        self.last_pose = None
        self.last_path_stamp = None
        self.last_path = []
        self.path_lock = threading.Lock()
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._model_cb, queue_size=1)
        rospy.Subscriber('/dog_global_path', Path, self._path_cb, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=1)
        rospy.loginfo("[Exp] Waiting for first model_states + path...")
        rospy.sleep(1.0)

    def _model_cb(self, msg):
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            return
        self.last_pose = (msg.pose[i].position.x, msg.pose[i].position.y)

    def _path_cb(self, msg):
        self.last_path_stamp = msg.header.stamp.to_sec()
        with self.path_lock:
            self.last_path = [
                (ps.pose.position.x, ps.pose.position.y)
                for ps in msg.poses
            ]

    def send_goal(self, x, y, yaw):
        gp = PoseStamped()
        gp.header.frame_id = 'map'
        gp.header.stamp = rospy.Time.now()
        gp.pose.position.x = x
        gp.pose.position.y = y
        gp.pose.orientation.z = math.sin(yaw / 2.0)
        gp.pose.orientation.w = math.cos(yaw / 2.0)
        prev_path_stamp = self.last_path_stamp
        for _ in range(3):
            self.goal_pub.publish(gp)
            rospy.sleep(0.1)
        t0 = rospy.get_time()
        while (rospy.get_time() - t0 < 5.0 and
               (self.last_path_stamp is None or
                self.last_path_stamp == prev_path_stamp)):
            rospy.sleep(0.1)
        rospy.loginfo("[Exp] sent goal (%.2f, %.2f), path stamp %s",
                      x, y, self.last_path_stamp)
        with self.path_lock:
            return list(self.last_path)

    def wait_arrival(self, x, y, timeout):
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            if self.last_pose is None:
                rospy.sleep(0.1)
                continue
            d = math.hypot(self.last_pose[0] - x, self.last_pose[1] - y)
            if d < ARRIVAL_TOL:
                return True, rospy.get_time() - t0
            if rospy.get_time() - t0 > timeout:
                return False, rospy.get_time() - t0
            rospy.sleep(0.2)
        return False, 0.0

    def run(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        log_path = os.path.join(OUT_DIR, "dog_integration_trajectory.log")
        json_path = os.path.join(OUT_DIR, "dog_integration.json")
        traj = []
        per_goal = []

        recording = [True]

        def rec_loop():
            while recording[0] and not rospy.is_shutdown():
                if self.last_pose is not None:
                    traj.append((rospy.get_time(),
                                 self.last_pose[0], self.last_pose[1]))
                rospy.sleep(0.1)

        th = threading.Thread(target=rec_loop, daemon=True)
        th.start()

        for i, (gx, gy, gyaw) in enumerate(GOALS):
            rospy.loginfo("[Exp] Goal %d: (%.2f, %.2f, %.2f)",
                          i + 1, gx, gy, gyaw)
            start_idx = len(traj)
            path_snapshot = self.send_goal(gx, gy, gyaw)
            ok, dt = self.wait_arrival(gx, gy, GOAL_TIMEOUT)
            segment = traj[start_idx:]
            rospy.loginfo("[Exp] Goal %d ok=%s in %.1f s", i + 1, ok, dt)
            per_goal.append({
                "goal": [gx, gy, gyaw],
                "ok": bool(ok),
                "time_s": dt,
                "actual_path_length_m": path_length([(x, y) for _, x, y in segment]),
                "global_path_length_m": path_length(path_snapshot),
                "global_path": path_snapshot,
                "trajectory_samples": len(segment),
            })

        recording[0] = False
        th.join(timeout=1.0)

        with open(log_path, "w") as f:
            f.write("# t x y\n")
            for (t, x, y) in traj:
                f.write(f"{t:.3f} {x:.3f} {y:.3f}\n")
        with open(json_path, "w") as f:
            json.dump({
                "goals": GOALS,
                "per_goal": per_goal,
                "trajectory_samples": len(traj),
                "actual_path_length_m": path_length([(x, y) for _, x, y in traj]),
                "success_rate": (sum(1 for g in per_goal if g["ok"]) /
                                 float(len(per_goal)) if per_goal else 0.0),
            }, f, indent=2)
        rospy.loginfo("[Exp] wrote %s and %s", log_path, json_path)


def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(points[i][0] - points[i - 1][0],
                          points[i][1] - points[i - 1][1])
               for i in range(1, len(points)))


if __name__ == '__main__':
    try:
        Recorder().run()
    except rospy.ROSInterruptException:
        pass
