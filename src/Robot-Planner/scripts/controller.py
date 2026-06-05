#!/usr/bin/env python3
import sys
import math
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_compat import (
    CompatNode, spin,
    Twist, Float32MultiArray, Odometry,
)
from utils import calc_distance, normalize_angle_deg


def calculate_angle(p_1, p_2, p_3):
    angle_1 = math.atan2(p_2[1] - p_1[1], p_2[0] - p_1[0])
    angle_2 = math.atan2(p_3[1] - p_2[1], p_3[0] - p_2[0])
    return angle_2 - angle_1


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

        self.linear_pid = PIDController(kp=0.6, ki=0.01, kd=0.02,
                                        output_min=0.0, output_max=0.60)
        self.angular_pid = PIDController(kp=1.2, ki=0.02, kd=0.04,
                                         output_min=-2.0, output_max=2.0)

        self.sub_odom = node.create_subscriber(Odometry, '/odom', self.get_pos)
        self.sub_path = node.create_subscriber(Float32MultiArray, '/path', self.path_callback)
        self.pub = node.create_publisher(Twist, 'cmd_vel', queue_size=10)

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
        self.path = path_nodes
        self.new_path_received = True
        self.active = True
        self.node.loginfo("New path: %d waypoints", len(path_nodes))

    def run(self):
        rate = self.node.create_rate(50)
        self.node.loginfo("Controller ready, waiting for path...")

        point_idx = 0
        prev_pos = self.current_pos
        prev_linear = 0.0
        prev_angular = 0.0
        max_lin_accel = 0.8
        max_ang_accel = 3.0
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

            # New path received — restart tracking from beginning
            if self.new_path_received:
                self.new_path_received = False
                point_idx = 0
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

            # All waypoints reached
            if point_idx >= len(self.path):
                self.pub.publish(Twist())
                self.active = False
                self.node.loginfo("Goal reached")
                continue

            target = self.path[point_idx]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist_to_target = calc_distance(self.current_pos, target)

            # Final waypoint — higher precision
            is_final = (point_idx == len(self.path) - 1)
            wp_threshold = 0.10 if is_final else 0.30
            goal_threshold = 0.08 if is_final else wp_threshold

            if is_final and dist_to_target < goal_threshold:
                self.pub.publish(Twist())
                self.active = False
                self.node.loginfo("Final waypoint reached")
                continue
            elif not is_final and dist_to_target < wp_threshold:
                point_idx += 1
                self.linear_pid.reset()
                self.angular_pid.reset()
                stall_count = 0
                continue

            # Heading computation
            desired_heading = math.degrees(math.atan2(dy, dx))
            heading_error = normalize_angle_deg(desired_heading - self.current_orient)

            # Angular PID
            angular_vel = self.angular_pid.update(heading_error, dt)

            # Stuck detection — no movement for 2 seconds
            moved = calc_distance(self.current_pos, prev_pos)
            if dt > 0.05:
                if moved < 0.01:
                    stall_count += 1
                else:
                    stall_count = 0
                prev_pos = self.current_pos

            # Recovery: if stuck for >2s, back up and reorient
            if stall_count > 100 and not recovering:
                self.node.logwarn("Stuck detected! Recovering...")
                recovering = True
                recovery_step = 0

            if recovering:
                move_cmd = Twist()
                if recovery_step < 25:  # back up for 0.5s
                    move_cmd.linear.x = -0.30
                    move_cmd.angular.z = 0.0
                elif recovery_step < 50:  # rotate for 0.5s
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 1.5
                else:  # done recovering
                    recovering = False
                    stall_count = 0
                    point_idx = max(0, point_idx - 1)  # retry previous waypoint
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

            # Linear velocity — proportional to distance, scaled by heading alignment
            if abs(heading_error) < 45.0:
                turn_penalty = math.cos(math.radians(heading_error))
                turn_penalty = max(0.15, turn_penalty)
                linear_vel = min(dist_to_target * 0.5, self.linear_pid.output_max)
                linear_vel *= turn_penalty
            else:
                linear_vel = 0.0

            # Acceleration limiting
            linear_vel = max(min(linear_vel, prev_linear + max_lin_accel * dt),
                             prev_linear - max_lin_accel * dt)
            angular_vel = max(min(angular_vel, prev_angular + max_ang_accel * dt),
                              prev_angular - max_ang_accel * dt)

            prev_linear = linear_vel
            prev_angular = angular_vel

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
