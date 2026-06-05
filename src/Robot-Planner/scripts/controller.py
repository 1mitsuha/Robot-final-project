#!/usr/bin/env python3
import sys
import math
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_compat import (
    CompatNode, spin,
    Twist, Float32MultiArray, Odometry,
)
from sensor_msgs.msg import LaserScan
from utils import calc_distance, normalize_angle_deg


def quat_2_euler(ox, oy, oz, ow):
    t3 = +2.0 * (ow * oz + ox * oy)
    t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        if dt <= 0:
            dt = 0.02
        self.integral += error * dt
        I_max = abs(self.output_max - self.output_min) * 0.2
        self.integral = max(min(self.integral, I_max), -I_max)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, self.output_max), self.output_min)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class Controller:
    def __init__(self, node):
        self.node = node
        self.path = []
        self.current_pos = (0, 0)
        self.current_orient = 0
        self.new_path_received = False
        self.active = False

        self.linear_pid = PIDController(kp=0.5, ki=0.02, kd=0.02,
                                        output_min=0.0, output_max=0.80)
        # Angular PID only provides small correction; primary steering is feedforward
        self.angular_pid = PIDController(kp=0.35, ki=0.005, kd=0.03,
                                         output_min=-1.2, output_max=1.2)

        self.sub_odom = node.create_subscriber(Odometry, '/odom', self.get_pos)
        self.sub_path = node.create_subscriber(Float32MultiArray, '/path', self.path_callback)
        self.sub_laser = node.create_subscriber(LaserScan, '/scan', self.laser_callback)
        self.pub = node.create_publisher(Twist, 'cmd_vel', queue_size=10)

        # Laser data
        self.laser_ranges = None
        self.laser_angle_min = 0.0
        self.laser_angle_max = 0.0
        self.laser_angle_inc = 0.0

    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_max = msg.angle_max
        self.laser_angle_inc = msg.angle_increment
        # Replace inf/nan with range_max
        for i in range(len(self.laser_ranges)):
            r = self.laser_ranges[i]
            if math.isinf(r) or math.isnan(r) or r > msg.range_max:
                self.laser_ranges[i] = msg.range_max

    def _min_in_cone(self, center_deg, half_deg):
        """Minimum laser range within [center_deg - half_deg, center_deg + half_deg]."""
        if self.laser_ranges is None:
            return 999.0
        n = len(self.laser_ranges)
        c = math.radians(center_deg)
        h = math.radians(half_deg)
        lo = int((c - h - self.laser_angle_min) / self.laser_angle_inc)
        hi = int((c + h - self.laser_angle_min) / self.laser_angle_inc)
        lo, hi = max(0, min(lo, n - 1)), max(0, min(hi, n - 1))
        if lo > hi:
            lo, hi = hi, lo
        return min(self.laser_ranges[lo:hi + 1]) if lo <= hi else 999.0

    def _find_clear_dir(self):
        """Find direction (degrees from robot front) with most free space."""
        if self.laser_ranges is None:
            return 0.0
        best_dir = 0.0
        best_dist = 0.0
        for deg in range(-150, 151, 10):
            d = self._min_in_cone(deg, 15)
            if d > best_dist:
                best_dist = d
                best_dir = deg
        return best_dir

    def get_pos(self, data):
        self.current_pos = (data.pose.pose.position.x,
                            data.pose.pose.position.y)
        ox = data.pose.pose.orientation.x
        oy = data.pose.pose.orientation.y
        oz = data.pose.pose.orientation.z
        ow = data.pose.pose.orientation.w
        self.current_orient = quat_2_euler(ox, oy, oz, ow) * 180 / math.pi

    def path_callback(self, data):
        path_nodes = []
        published_data = data.data
        for i in range(0, len(published_data), 2):
            if i + 1 < len(published_data):
                path_nodes.append((round(published_data[i], 3),
                                   round(published_data[i + 1], 3)))
        # Interpolate path: insert midpoints on long segments for smoother tracking
        self.path = self._interpolate_path(path_nodes)
        self.new_path_received = True
        self.active = True
        self.node.loginfo("New path: %d waypoints (interpolated from %d)",
                          len(self.path), len(path_nodes))

    def _interpolate_path(self, path, max_seg=0.3):
        """Insert intermediate points to ensure segment length <= max_seg."""
        if len(path) < 2:
            return path
        result = [path[0]]
        for i in range(1, len(path)):
            prev = result[-1]
            curr = path[i]
            dist = calc_distance(prev, curr)
            if dist > max_seg:
                n_extra = int(dist / max_seg)
                for j in range(1, n_extra + 1):
                    t = j / (n_extra + 1)
                    result.append((prev[0] + (curr[0] - prev[0]) * t,
                                   prev[1] + (curr[1] - prev[1]) * t))
            result.append(curr)
        return result

    def _lookahead_point(self, lookahead_dist=0.5):
        """Pure pursuit: find point on path lookahead_dist ahead of robot."""
        if not self.path:
            return self.current_pos

        # Find closest point on path
        ci = 0
        best = float('inf')
        for i, wp in enumerate(self.path):
            d = calc_distance(self.current_pos, wp)
            if d < best:
                best = d
                ci = i

        # Walk forward accumulating distance until lookahead_dist
        accum = 0.0
        prev = self.current_pos
        for i in range(ci, len(self.path)):
            wp = self.path[i]
            seg = calc_distance(prev, wp)
            if accum + seg >= lookahead_dist:
                remaining = lookahead_dist - accum
                ratio = remaining / seg if seg > 0 else 0.0
                return (prev[0] + (wp[0] - prev[0]) * ratio,
                        prev[1] + (wp[1] - prev[1]) * ratio)
            accum += seg
            prev = wp

        return self.path[-1]

    def run(self):
        rate = self.node.create_rate(50)
        self.node.loginfo("Controller ready, waiting for path...")

        prev_pos = self.current_pos
        prev_linear = 0.0
        prev_angular = 0.0
        max_lin_accel = 1.2
        max_ang_accel = 5.0
        last_time = self.node.now()
        stall_count = 0
        recovering = False
        recovery_step = 0

        while CompatNode.ok():
            now = self.node.now()
            dt = self.node.time_diff_sec(now, last_time)
            if dt > 0.2:
                dt = 0.02
            last_time = now

            # New path received — restart tracking
            if self.new_path_received:
                self.new_path_received = False
                prev_pos = self.current_pos
                stall_count = 0
                recovering = False
                recovery_step = 0
                self.linear_pid.reset()
                self.angular_pid.reset()
                self.node.loginfo("Starting new path: %d waypoints", len(self.path))

            # No path yet
            if not self.path or not self.active:
                self.pub.publish(Twist())
                rate.sleep()
                continue

            # Check if near the final goal
            dist_to_goal = calc_distance(self.current_pos, self.path[-1])
            if dist_to_goal < 0.12:
                self.pub.publish(Twist())
                self.active = False
                self.node.loginfo("Goal reached (dist=%.3fm)", dist_to_goal)
                continue

            # ==== Pure Pursuit with feedforward + small PID correction ====
            # Dynamic look-ahead: farther when fast, closer when slow
            dyn_lookahead = 0.5 + 0.3 * (prev_linear / 0.80)
            lookahead = self._lookahead_point(lookahead_dist=dyn_lookahead)
            dx = lookahead[0] - self.current_pos[0]
            dy = lookahead[1] - self.current_pos[1]
            dist_to_lookahead = math.hypot(dx, dy)

            # Angle from robot heading to look-ahead point
            desired_heading = math.degrees(math.atan2(dy, dx))
            heading_error = normalize_angle_deg(desired_heading - self.current_orient)
            alpha_rad = math.radians(heading_error)

            # ---- FEEDFORWARD: pure pursuit curvature formula ----
            # κ = 2*sin(α)/L,  ω = v * κ
            if dist_to_lookahead > 0.02:
                curvature = 2.0 * math.sin(alpha_rad) / dyn_lookahead
                angular_vel_ff = prev_linear * curvature  # use current speed, not target
            else:
                angular_vel_ff = 0.0

            # ---- FEEDBACK: small PID correction for disturbances ----
            angular_pid_out = self.angular_pid.update(heading_error, dt)

            # Blend: at low speed use more PID (need to orient), at high speed use feedforward
            speed_ratio = min(prev_linear / 0.30, 1.0)  # 0→1 as speed rises
            angular_vel_raw = speed_ratio * angular_vel_ff + (1.0 - speed_ratio) * angular_pid_out
            # Always add a small PID term for stability
            angular_vel_raw += 0.15 * angular_pid_out

            # ---- Linear velocity ----
            # Deadband for heading: don't move forward if way off
            if abs(heading_error) > 80.0:
                linear_vel_raw = 0.0
            else:
                linear_vel_raw = self.linear_pid.update(dist_to_goal, dt)
                turn_penalty = math.cos(alpha_rad)
                turn_penalty = max(0.12, turn_penalty)
                linear_vel_raw *= turn_penalty

            # Soft slowdown near goal
            if dist_to_goal < 1.2:
                slow = dist_to_goal / 1.2
                linear_vel_raw = min(linear_vel_raw, 0.50 * slow + 0.04)

            # ---- Stuck detection ----
            moved = calc_distance(self.current_pos, prev_pos)
            if dt > 0.05:
                if moved < 0.008:
                    stall_count += 1
                else:
                    stall_count = max(0, stall_count - 2)
                prev_pos = self.current_pos

            # Recovery: laser-guided escape
            if stall_count > 120 and not recovering:
                self.node.logwarn("Stuck detected! Recovering...")
                recovering = True
                recovery_step = 0
                # Find clearest direction
                self._recovery_clear_dir = self._find_clear_dir()
                self.node.loginfo("Clear direction: %.0f deg", self._recovery_clear_dir)

            if recovering:
                move_cmd = Twist()
                if recovery_step < 80:  # back up 1.6s
                    move_cmd.linear.x = -0.40
                    move_cmd.angular.z = 0.0
                elif recovery_step < 160:  # turn toward clearest direction 1.6s
                    move_cmd.linear.x = 0.0
                    turn_dir = 1.0 if self._recovery_clear_dir >= 0 else -1.0
                    move_cmd.angular.z = 1.8 * turn_dir
                elif recovery_step < 200:  # drive forward 0.8s
                    move_cmd.linear.x = 0.35
                    move_cmd.angular.z = 0.0
                else:
                    recovering = False
                    stall_count = 0
                    prev_pos = self.current_pos
                    self.linear_pid.reset()
                    self.angular_pid.reset()
                    self.node.loginfo("Recovery complete")
                recovery_step += 1
                self.pub.publish(move_cmd)
                prev_linear = 0.0
                prev_angular = 0.0
                rate.sleep()
                continue

            # ---- EMA smoothing (more aggressive for angular) ----
            alpha_lin = 0.40
            alpha_ang = 0.15
            linear_vel = alpha_lin * linear_vel_raw + (1.0 - alpha_lin) * prev_linear
            angular_vel = alpha_ang * angular_vel_raw + (1.0 - alpha_ang) * prev_angular

            # ---- Acceleration limiting ----
            if linear_vel > prev_linear:
                linear_vel = min(linear_vel, prev_linear + max_lin_accel * dt)
            else:
                linear_vel = max(linear_vel, prev_linear - max_lin_accel * 2.5 * dt)

            angular_vel = max(min(angular_vel, prev_angular + max_ang_accel * dt),
                              prev_angular - max_ang_accel * dt)

            prev_linear = linear_vel
            prev_angular = angular_vel

            # ---- Laser safety: emergency brake + side protection ----
            front_min = self._min_in_cone(0, 35)
            left_min = self._min_in_cone(90, 15)
            right_min = self._min_in_cone(-90, 15)

            if front_min < 0.32:
                if linear_vel > 0:
                    linear_vel = 0.0
                # Turn toward more open side
                angular_vel = 1.2 if left_min > right_min else -1.2
                self.linear_pid.reset()

            # Side protection: don't turn into nearby walls
            if left_min < 0.18 and angular_vel > 0.2:
                angular_vel = 0.0
            if right_min < 0.18 and angular_vel < -0.2:
                angular_vel = 0.0

            move_cmd = Twist()
            move_cmd.linear.x = linear_vel
            move_cmd.angular.z = angular_vel
            self.pub.publish(move_cmd)
            rate.sleep()

        # Shutdown
        self.pub.publish(Twist())


if __name__ == '__main__':
    node = CompatNode('controller', anonymous=True)
    ctrl = Controller(node)
    ctrl.run()
