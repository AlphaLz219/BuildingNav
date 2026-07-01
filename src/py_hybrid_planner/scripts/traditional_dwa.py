#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traditional DWA (baseline for comparison with the paper's improved version).

Classical Fox et al. 1997 formulation:
  cost = alpha * heading + beta * dist + gamma * velocity
  with default weights (alpha=0.2, beta=0.2, gamma=0.2 normalised) and
  WITHOUT the extra delta_theta term that the paper's improved version
  adds (see paper 3.3.1).
"""

import math
import numpy as np


class TraditionalDWA:
    def __init__(self, robot_radius=0.11, hard_radius=0.13,
                 max_vel=0.22, min_vel=0.0,
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
        # classical Fox weights (equal emphasis)
        self.alpha = 0.2
        self.beta = 0.2
        self.gamma = 0.2

    def motion(self, x, y, theta, v, w, dt):
        return (x + v * math.cos(theta) * dt,
                y + v * math.sin(theta) * dt,
                theta + w * dt)

    def calc_dynamic_window(self, v, w):
        return (max(self.min_vel, v - self.max_accel * self.dt),
                min(self.max_vel, v + self.max_accel * self.dt),
                max(self.min_omega, w - self.max_domega * self.dt),
                min(self.max_omega, w + self.max_domega * self.dt))

    def predict_trajectory(self, x, y, theta, v, w):
        traj = [(x, y, theta)]
        t = 0.0
        cx, cy, ct = x, y, theta
        while t < self.predict_time:
            cx, cy, ct = self.motion(cx, cy, ct, v, w, self.dt)
            traj.append((cx, cy, ct))
            t += self.dt
        return traj

    @staticmethod
    def _heading(traj, gx, gy):
        ex, ey, et = traj[-1]
        target = math.atan2(gy - ey, gx - ex)
        d = abs(et - target)
        if d > math.pi:
            d = 2 * math.pi - d
        return 180.0 - math.degrees(d)

    def _dist(self, traj, obs_arr):
        if obs_arr.size == 0:
            return 2.0, False
        min_d = float('inf')
        for i, (x, y, _) in enumerate(traj[::2]):
            d = float(np.min(np.hypot(obs_arr[:, 0] - x, obs_arr[:, 1] - y)))
            if d < min_d:
                min_d = d
            # classical formulation rejects ANY trajectory point inside the
            # collision radius — including index 0 — which is exactly the
            # behaviour that causes the "stuck after collision" failure we
            # keep here on purpose so the comparison is honest.
            if d <= self.hard_radius:
                return min_d, True
        return min(min_d, 2.0), False

    def plan(self, current_x, current_y, current_theta,
             v_curr, w_curr, goal_x, goal_y, obstacle_points, prev_theta=None):
        obs = np.array(obstacle_points) if obstacle_points else np.empty((0, 2))
        if obs.size:
            m = np.hypot(obs[:, 0] - current_x, obs[:, 1] - current_y) < 3.0
            obs = obs[m]

        v_min, v_max, w_min, w_max = self.calc_dynamic_window(v_curr, w_curr)
        v_step = max((v_max - v_min) / max(self.v_samples, 1), 0.02)
        w_step = max((w_max - w_min) / max(self.w_samples, 1), 0.05)

        best_score = -float('inf')
        best = (0.0, 0.0)
        v = v_min
        while v <= v_max + 1e-9:
            w = w_min
            while w <= w_max + 1e-9:
                traj = self.predict_trajectory(current_x, current_y, current_theta, v, w)
                head = self._heading(traj, goal_x, goal_y)
                d, collide = self._dist(traj, obs)
                if collide:
                    w += w_step
                    continue
                nh = head / 180.0
                nd = min(d / 1.0, 1.0)
                nv = v / self.max_vel if self.max_vel > 0 else 0.0
                s = self.alpha * nh + self.beta * nd + self.gamma * nv
                if s > best_score:
                    best_score = s
                    best = (v, w)
                w += w_step
            v += v_step
        # return info dict compatible with the improved DWA
        return best[0], best[1], {'stuck': best_score == -float('inf'), 'nearest_obs': None, 'reason': 'traditional'}
