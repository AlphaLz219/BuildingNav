#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Omnidirectional DWA for the ASK-3 quadruped.

Algorithmic differences vs. the wheeled-robot DWA used by py_hybrid_planner
(see report Section 4 "Algorithm adaptations"):

1. State space is (vx, vy, w) instead of (v, w). The dog can side-step,
   so the velocity sample grid is now 3-D. We keep the grid sparse
   (vx_samples * vy_samples * w_samples ~= 6*5*9) and rely on the dynamic
   window to prune infeasible candidates.

2. Forward-kinematics step uses the body-frame twist projected back into
   the world frame:
       x  += (vx*cos(t) - vy*sin(t)) * dt
       y  += (vx*sin(t) + vy*cos(t)) * dt
       th += w * dt
   (vs. the wheeled equation x += v*cos(t)*dt, y += v*sin(t)*dt that
    assumes a non-holonomic body).

3. Heading score is replaced primarily by an "approach" score. The dog
   can side-step, but when the front corridor is open it should still
   behave like a head-first quadruped: rotate/yaw to align, then walk
   forward through gaps. Lateral motion remains available for close
   obstacle recovery.

4. The "stuck" recovery hint is also different: the recommended escape
   action is a lateral side-step (vy != 0) rather than a pure rotation,
   which is faster on a holonomic platform.

5. Acceleration, braking distance and footprint sizes are tuned for the
   ASK-3. The thesis' braking-safe velocity set Va is implemented as a
   clearance check against speed^2 / (2 * max_decel_xy), and the
   cos(delta theta) smoothness term is applied to the velocity score.
"""

import math
import numpy as np


class OmniDWA:
    def __init__(self,
                 robot_radius=0.23,          # ASK-3 footprint half-diagonal
                 hard_radius=0.13,           # local body-width collision radius
                 footprint_length=0.42,      # compatibility only
                 footprint_width=0.24,       # compatibility only
                 footprint_margin=0.015,
                 max_vx=0.35, min_vx=-0.10,
                 max_vy=0.20, min_vy=-0.20,
                 max_omega=1.0, min_omega=-1.0,
                 max_accel_xy=1.5, max_domega=2.0,
                 max_decel_xy=1.0, brake_margin=0.03,
                 dt=0.1, predict_time=1.4,
                 vx_samples=7, vy_samples=5, w_samples=9,
                 normal_vy_limit=0.06,
                 path_track_weight=0.16,
                 path_corridor_width=0.55):
        self.robot_radius = robot_radius
        self.hard_radius = hard_radius
        self.max_vx = max_vx
        self.min_vx = min_vx
        self.max_vy = max_vy
        self.min_vy = min_vy
        self.max_omega = max_omega
        self.min_omega = min_omega
        self.max_accel_xy = max_accel_xy
        self.max_domega = max_domega
        self.max_decel_xy = max_decel_xy
        self.brake_margin = brake_margin
        self.dt = dt
        self.predict_time = predict_time
        self.vx_samples = vx_samples
        self.vy_samples = vy_samples
        self.w_samples = w_samples
        self.path_track_weight = path_track_weight
        self.path_corridor_width = path_corridor_width

        # Score weights (sum normalised). Clearance remains the largest
        # term following the thesis' DWA analysis, while progress replaces
        # most of the non-holonomic heading term for the quadruped.
        self.alpha_progress = 0.30   # how much closer the trajectory ends up
        self.beta_clearance = 0.36   # min distance to nearby obstacles
        self.gamma_velocity = 0.24   # prefer smooth faster motion
        self.delta_heading = 0.10    # mild bonus for facing along motion
        self.forward_bias = 0.13     # prefer head-first walking when front is clear
        self.side_penalty = 0.14     # discourage crab-walking in open front gaps
        self.yaw_align_weight = 0.04 # small bonus for facing the local target
        self.side_bypass_weight = 0.30
        self.side_turn_penalty = 0.24
        self.side_backward_penalty = 0.08

    # ------------------------------------------------------------------
    def motion(self, x, y, theta, vx, vy, w, dt):
        ct, st = math.cos(theta), math.sin(theta)
        x += (vx * ct - vy * st) * dt
        y += (vx * st + vy * ct) * dt
        theta += w * dt
        return x, y, theta

    def _to_body(self, cx, cy, theta, ox, oy):
        dx = ox - cx
        dy = oy - cy
        ct, st = math.cos(theta), math.sin(theta)
        return dx * ct + dy * st, -dx * st + dy * ct

    def calc_dynamic_window(self, vx, vy, w):
        vx_min = max(self.min_vx, vx - self.max_accel_xy * self.dt)
        vx_max = min(self.max_vx, vx + self.max_accel_xy * self.dt)
        vy_min = max(self.min_vy, vy - self.max_accel_xy * self.dt)
        vy_max = min(self.max_vy, vy + self.max_accel_xy * self.dt)
        w_min = max(self.min_omega, w - self.max_domega * self.dt)
        w_max = min(self.max_omega, w + self.max_domega * self.dt)
        return vx_min, vx_max, vy_min, vy_max, w_min, w_max

    def predict_trajectory(self, x, y, theta, vx, vy, w):
        traj = [(x, y, theta)]
        cx, cy, ct = x, y, theta
        t = 0.0
        while t < self.predict_time:
            cx, cy, ct = self.motion(cx, cy, ct, vx, vy, w, self.dt)
            traj.append((cx, cy, ct))
            t += self.dt
        return traj

    @staticmethod
    def _path_points_window(global_path, start_idx, forward_dist=1.8):
        if not global_path:
            return []
        start_idx = max(0, min(int(start_idx), len(global_path) - 1))
        end_idx = start_idx
        travelled = 0.0
        while end_idx + 1 < len(global_path) and travelled < forward_dist:
            x0, y0 = global_path[end_idx]
            x1, y1 = global_path[end_idx + 1]
            travelled += math.hypot(x1 - x0, y1 - y0)
            end_idx += 1
        return global_path[start_idx:end_idx + 1]

    def _path_corridor_score(self, x, y, path_pts):
        if path_pts is None or path_pts.size == 0:
            return 0.0
        d = float(np.min(np.hypot(path_pts[:, 0] - x, path_pts[:, 1] - y)))
        return max(0.0, 1.0 - d / max(self.path_corridor_width, 1e-3))

    # ------------------------------------------------------------------
    def _nearby_obs(self, obs_arr, cx, cy, radius=3.0):
        if obs_arr.size == 0:
            return obs_arr
        mask = np.hypot(obs_arr[:, 0] - cx, obs_arr[:, 1] - cy) < radius
        return obs_arr[mask]

    def _front_clearance(self, nearby_obs, cx, cy, theta):
        if nearby_obs.size == 0:
            return 2.0
        ct, st = math.cos(theta), math.sin(theta)
        best = float('inf')
        for ox, oy in nearby_obs:
            dx = ox - cx
            dy = oy - cy
            lx = dx * ct + dy * st
            ly = -dx * st + dy * ct
            if lx <= 0.0:
                continue
            if abs(math.atan2(ly, lx)) > math.radians(28.0):
                continue
            d = math.hypot(lx, ly)
            if d < best:
                best = d
        return min(best, 2.0) if best < float('inf') else 2.0

    def calc_dist(self, traj, nearby_obs, is_in_place):
        """
        Returns (min_clearance, collision_flag).

        Skips the first 2 samples so a dog already in its own inflation
        zone is not stuck reporting a permanent collision. Pure in-place
        rotation/skip (very small ||v||) never triggers a geometric
        collision flag (the dog is rotating in place).
        """
        if nearby_obs.size == 0:
            return 2.0, False
        check = traj[::2] if len(traj) > 4 else traj
        skip = 2 if not is_in_place else len(check)
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

    def calc_closing_dist(self, nearby_obs, cx, cy, theta, vx, vy):
        if nearby_obs.size == 0:
            return 2.0
        speed = math.hypot(vx, vy)
        if speed < 0.02:
            return 2.0
        motion_dir = math.atan2(vy, vx)
        cd, sd = math.cos(motion_dir), math.sin(motion_dir)
        best = float('inf')
        for ox, oy in nearby_obs:
            lx, ly = self._to_body(cx, cy, theta, ox, oy)
            along = lx * cd + ly * sd
            lateral = -lx * sd + ly * cd
            if along <= 0.0 or abs(lateral) > self.hard_radius:
                continue
            d = math.hypot(along, lateral)
            if d < best:
                best = d
        return min(best, 2.0) if best < float('inf') else 2.0

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # ------------------------------------------------------------------
    def plan(self, current_x, current_y, current_theta,
             vx_curr, vy_curr, w_curr,
             goal_x, goal_y, obstacle_points,
             prev_theta=0.0,
             global_path=None, path_index=0):
        """
        Returns (best_vx, best_vy, best_w, info).
        info = {'stuck': bool, 'nearest_obs': (ox,oy)|None, 'reason': str,
                'trajectory': [(x,y,theta), ...]}
        """
        info = {'stuck': False, 'nearest_obs': None, 'reason': '',
                'trajectory': []}

        obs_arr = (np.array(obstacle_points, dtype=float)
                   if obstacle_points else np.empty((0, 2)))
        nearby = self._nearby_obs(obs_arr, current_x, current_y, radius=3.0)

        if nearby.size:
            d = np.hypot(nearby[:, 0] - current_x, nearby[:, 1] - current_y)
            idx = int(np.argmin(d))
            cur_min_obs = float(d[idx])
            info['nearest_obs'] = (float(nearby[idx, 0]), float(nearby[idx, 1]))
        else:
            cur_min_obs = float('inf')

        d0_goal = math.hypot(goal_x - current_x, goal_y - current_y)
        if d0_goal < 1e-6:
            return 0.0, 0.0, 0.0, info
        path_window = self._path_points_window(global_path, path_index)
        path_pts = (np.array(path_window, dtype=float)
                    if path_window else np.empty((0, 2)))
        front_clearance = self._front_clearance(
            nearby, current_x, current_y, current_theta)
        front_open = max(0.0, min((front_clearance - 0.35) / 0.45, 1.0))
        target_bearing = math.atan2(goal_y - current_y, goal_x - current_x)
        target_lx, target_ly = self._to_body(
            current_x, current_y, current_theta, goal_x, goal_y)
        target_side_ratio = abs(target_ly) / max(abs(target_lx) + abs(target_ly), 1e-3)
        side_bypass = max(0.0, min((target_side_ratio - 0.45) / 0.35, 1.0))
        side_dir = 1.0 if target_ly >= 0.0 else -1.0

        vx_min, vx_max, vy_min, vy_max, w_min, w_max = \
            self.calc_dynamic_window(vx_curr, vy_curr, w_curr)

        # Build velocity grids
        def grid(lo, hi, n):
            if n <= 1 or hi - lo < 1e-6:
                return np.array([0.5 * (lo + hi)])
            return np.linspace(lo, hi, n)

        vxs = grid(vx_min, vx_max, self.vx_samples)
        vys = grid(vy_min, vy_max, self.vy_samples)
        ws = grid(w_min, w_max, self.w_samples)

        best_vx, best_vy, best_w = 0.0, 0.0, 0.0
        best_traj = []
        best_score = -float('inf')
        admissible = 0
        max_speed = math.hypot(self.max_vx, self.max_vy)

        for vx in vxs:
            for vy in vys:
                speed = math.hypot(vx, vy)
                for w in ws:
                    is_in_place = speed < 0.02
                    traj = self.predict_trajectory(
                        current_x, current_y, current_theta, vx, vy, w)
                    clearance, collision = self.calc_dist(
                        traj, nearby, is_in_place)
                    if collision:
                        continue
                    closing_clearance = self.calc_closing_dist(
                        nearby, current_x, current_y, current_theta, vx, vy)
                    brake_clearance = (self.hard_radius + self.brake_margin +
                                       (speed * speed) /
                                       (2.0 * max(self.max_decel_xy, 1e-3)))
                    if speed > 0.02 and closing_clearance <= brake_clearance:
                        continue
                    admissible += 1
                    ex, ey, et = traj[-1]
                    d_end = math.hypot(goal_x - ex, goal_y - ey)
                    progress = (d0_goal - d_end) / max(d0_goal, 0.5)

                    # heading bonus: motion direction vs. body forward (only
                    # if we are moving)
                    if speed > 1e-3:
                        motion_dir = math.atan2(vy, vx)
                        heading_err = abs(self._wrap(motion_dir))
                        heading_score = 1.0 - heading_err / math.pi
                        lateral_ratio = abs(vy) / max(abs(vx) + abs(vy), 1e-3)
                    else:
                        heading_score = 0.0
                        lateral_ratio = 0.0

                    target_heading_err = abs(self._wrap(target_bearing - et))
                    target_heading_score = 1.0 - target_heading_err / math.pi
                    forward_score = max(vx, 0.0) / max(self.max_vx, 1e-3)
                    head_first_score = forward_score * (1.0 - lateral_ratio)
                    max_abs_vy = max(abs(self.max_vy), abs(self.min_vy), 1e-3)
                    side_speed_score = max(0.0, side_dir * vy) / max_abs_vy
                    turn_cost = abs(w) / max(abs(self.max_omega), 1e-3)
                    backward_cost = max(0.0, -vx) / max(abs(self.min_vx), 1e-3)
                    side_cost = self.side_penalty * front_open * lateral_ratio * (1.0 - side_bypass)

                    delta_theta = abs(self._wrap(et - current_theta))
                    smooth_score = 0.5 * (1.0 + math.cos(delta_theta))

                    nclear = min(clearance / 1.0, 1.0)
                    nspeed = min(speed / max_speed, 1.0) if max_speed > 0 else 0.0
                    nprog = max(min(progress, 1.0), -1.0)
                    mx, my, _ = traj[len(traj) // 2]
                    path_score = (
                        0.4 * self._path_corridor_score(mx, my, path_pts) +
                        0.6 * self._path_corridor_score(ex, ey, path_pts))

                    score = (self.alpha_progress * nprog +
                             self.beta_clearance * nclear +
                             self.gamma_velocity * nspeed * smooth_score +
                             self.path_track_weight * path_score +
                             self.delta_heading * heading_score * (1.0 - 0.7 * side_bypass) +
                             self.forward_bias * front_open * head_first_score * (1.0 - side_bypass) +
                             self.yaw_align_weight * front_open * target_heading_score * (1.0 - side_bypass) +
                             self.side_bypass_weight * side_bypass * side_speed_score -
                             self.side_turn_penalty * side_bypass * turn_cost -
                             self.side_backward_penalty * side_bypass * backward_cost -
                             side_cost)

                    if score > best_score:
                        best_score = score
                        best_vx, best_vy, best_w = vx, vy, w
                        best_traj = traj

        if admissible == 0:
            # No collision-free move; suggest a lateral side-step away from
            # the nearest obstacle. The nav layer can use this as a recovery
            # action (paired with a rotation) rather than blocking entirely.
            info['stuck'] = True
            info['reason'] = 'no admissible candidate'
            if info['nearest_obs'] is not None:
                ox, oy = info['nearest_obs']
                # body-frame bearing to obstacle
                world_bearing = math.atan2(oy - current_y, ox - current_x)
                local_bearing = self._wrap(world_bearing - current_theta)
                # side-step in the OPPOSITE direction
                side_sign = -1.0 if local_bearing > 0 else 1.0
                info['trajectory'] = self.predict_trajectory(
                    current_x, current_y, current_theta, 0.0,
                    0.12 * side_sign, 0.0)
                return 0.0, 0.12 * side_sign, 0.0, info
            info['trajectory'] = self.predict_trajectory(
                current_x, current_y, current_theta, 0.0, 0.0, 0.5)
            return 0.0, 0.0, 0.5, info

        info['trajectory'] = best_traj
        info['score'] = best_score
        return best_vx, best_vy, best_w, info
