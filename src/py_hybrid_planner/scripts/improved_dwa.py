#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Improved DWA with robust collision checking.

Fixes over the original implementation:
- Collision check skips the first `skip_samples` trajectory points instead
  of only index 0, so a robot that has drifted slightly into its own
  inflation zone is not classified as colliding for every (v, w) pair.
- Rotation-only (v=0) trajectories are always admissible if the very
  closest obstacle to the robot is still outside the hard collision radius.
- `stuck` is reported to the upper layer via the returned `info` dict so
  the navigator can trigger a recovery maneuver instead of spinning in
  place forever.
"""

import math
import numpy as np


class ImprovedDWA:
    def __init__(self,
                 robot_radius=0.11,          # turtlebot3 burger
                 hard_radius=0.13,           # hard collision (inflated a bit)
                 max_vel=0.22, min_vel=-0.05,
                 max_omega=2.0, min_omega=-2.0,
                 max_accel=2.5, max_domega=3.2,
                 dt=0.1, predict_time=1.2,
                 v_samples=8, w_samples=21):
        self.robot_radius = robot_radius
        self.hard_radius = hard_radius
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_omega = max_omega
        self.min_omega = min_omega
        self.max_accel = max_accel
        self.max_domega = max_domega
        self.dt = dt
        self.predict_time = predict_time
        self.v_samples = v_samples
        self.w_samples = w_samples
        # Scoring weights (paper 3.3.2)
        self.alpha = 0.2   # heading
        self.beta = 0.5    # safety distance
        self.gamma = 0.3   # velocity

    # ------------------------------------------------------------------
    def motion(self, x, y, theta, v, w, dt):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        return x, y, theta

    def calc_dynamic_window(self, v, w):
        v_min = max(self.min_vel, v - self.max_accel * self.dt)
        v_max = min(self.max_vel, v + self.max_accel * self.dt)
        w_min = max(self.min_omega, w - self.max_domega * self.dt)
        w_max = min(self.max_omega, w + self.max_domega * self.dt)
        return v_min, v_max, w_min, w_max

    def predict_trajectory(self, x, y, theta, v, w):
        traj = [(x, y, theta)]
        t = 0.0
        cx, cy, ct = x, y, theta
        while t < self.predict_time:
            cx, cy, ct = self.motion(cx, cy, ct, v, w, self.dt)
            traj.append((cx, cy, ct))
            t += self.dt
        return traj

    def calc_heading(self, traj, gx, gy):
        ex, ey, et = traj[-1]
        target = math.atan2(gy - ey, gx - ex)
        diff = abs(et - target)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        return 180.0 - math.degrees(diff)

    # ------------------------------------------------------------------
    def _nearby_obs(self, obs_arr, cx, cy, radius=3.0):
        if obs_arr.size == 0:
            return obs_arr
        mask = np.hypot(obs_arr[:, 0] - cx, obs_arr[:, 1] - cy) < radius
        return obs_arr[mask]

    def calc_dist(self, traj, nearby_obs, is_rotation_only):
        """
        Returns (min_dist, collision_flag).
        collision_flag True means this (v, w) is inadmissible.

        - For rotation-only trajectories we do NOT reject based on the
          first few samples (robot hasn't moved yet).
        - Only samples from index `skip` onward are checked against the
          hard collision radius.
        """
        if nearby_obs.size == 0:
            return 2.0, False
        # subsample trajectory for speed but keep tail
        check = traj[::2] if len(traj) > 4 else traj
        # skip the first 2 samples (about 0.2s) so that a robot already
        # inside its own inflation zone is not stuck in "permanent
        # collision"
        skip = 2 if not is_rotation_only else len(check)  # in-place rot never hits geometry
        min_dist = float('inf')
        collision = False
        for i, (x, y, _) in enumerate(check):
            d = float(np.min(np.hypot(nearby_obs[:, 0] - x, nearby_obs[:, 1] - y)))
            if d < min_dist:
                min_dist = d
            if i >= skip and d <= self.hard_radius:
                collision = True
                break
        return min(min_dist, 2.0), collision

    # ------------------------------------------------------------------
    def plan(self, current_x, current_y, current_theta,
             v_curr, w_curr, goal_x, goal_y, obstacle_points, prev_theta):
        """
        Returns (best_v, best_w, info_dict).
        info_dict = {'stuck': bool, 'nearest_obs': (ox,oy) or None, 'reason': str}
        """
        info = {'stuck': False, 'nearest_obs': None, 'reason': ''}

        obs_arr = np.array(obstacle_points) if obstacle_points else np.empty((0, 2))
        nearby = self._nearby_obs(obs_arr, current_x, current_y, radius=3.0)

        # current distance to closest obstacle
        if nearby.size:
            d = np.hypot(nearby[:, 0] - current_x, nearby[:, 1] - current_y)
            idx = int(np.argmin(d))
            cur_min_obs = float(d[idx])
            info['nearest_obs'] = (float(nearby[idx, 0]), float(nearby[idx, 1]))
        else:
            cur_min_obs = float('inf')

        v_min, v_max, w_min, w_max = self.calc_dynamic_window(v_curr, w_curr)
        v_step = max((v_max - v_min) / max(self.v_samples, 1), 0.02)
        w_step = max((w_max - w_min) / max(self.w_samples, 1), 0.05)

        best_v, best_w = 0.0, 0.0
        best_score = -float('inf')
        admissible = 0

        v = v_min
        while v <= v_max + 1e-9:
            w = w_min
            while w <= w_max + 1e-9:
                traj = self.predict_trajectory(current_x, current_y, current_theta, v, w)
                heading = self.calc_heading(traj, goal_x, goal_y)
                dist, collision = self.calc_dist(traj, nearby, is_rotation_only=abs(v) < 1e-3)
                if collision:
                    w += w_step
                    continue
                admissible += 1
                final_theta = traj[-1][2]
                dtheta = abs(final_theta - prev_theta)
                if dtheta > math.pi:
                    dtheta = 2 * math.pi - dtheta
                vel_eval = v * math.cos(dtheta)

                nh = heading / 180.0
                nd = min(dist / 1.0, 1.0)
                nv = abs(vel_eval) / self.max_vel if self.max_vel > 0 else 0.0
                score = self.alpha * nh + self.beta * nd + self.gamma * nv
                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w
                w += w_step
            v += v_step

        if admissible == 0:
            # All forward motion blocked. Signal stuck so navigator can recover.
            info['stuck'] = True
            info['reason'] = 'no admissible candidate'
            # Prefer a slow rotation; direction chosen to face open space
            if info['nearest_obs'] is not None:
                ox, oy = info['nearest_obs']
                bearing = math.atan2(oy - current_y, ox - current_x)
                # rotate AWAY from obstacle bearing
                turn_sign = -1.0 if math.sin(bearing - current_theta) > 0 else 1.0
                return 0.0, turn_sign * 0.6, info
            return 0.0, 0.5, info

        # Even with candidates, mark stuck if we have been squeezed very close
        if cur_min_obs < self.hard_radius:
            info['stuck'] = True
            info['reason'] = 'inside hard radius'
        return best_v, best_w, info
