#!/usr/bin/env python3
import sys
import math
import random
import numpy as np

# Add this directory to path so ros_compat can be imported
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_compat import (
    CompatNode, spin, wait_for_message,
    Point, PoseStamped, OccupancyGrid, Odometry, Path,
    Float32MultiArray, Marker, MarkerArray,
)
from utils import calc_distance


class MapProcessor:
    def __init__(self, node):
        self.node = node
        self.map_data = None
        self.map_info = None
        self.obstacles = []
        node.create_subscriber(OccupancyGrid, '/map', self.map_callback)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.extract_obstacles()
        self._inflate_obstacles(inflation_cells=5)
        self.node.loginfo("Map loaded with resolution %.3f at origin (%.2f, %.2f)" % (
            self.map_info.resolution,
            self.map_info.origin.position.x,
            self.map_info.origin.position.y))

    def _inflate_obstacles(self, inflation_cells=3):
        self.inflated_data = self.map_data.copy()
        h, w = self.inflated_data.shape
        obs_mask = self.map_data > 50
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dy == 0 and dx == 0:
                    continue
                shifted = np.roll(obs_mask, (dy, dx), axis=(0, 1))
                if dy > 0:
                    shifted[:dy, :] = False
                elif dy < 0:
                    shifted[dy:, :] = False
                if dx > 0:
                    shifted[:, :dx] = False
                elif dx < 0:
                    shifted[:, dx:] = False
                self.inflated_data[shifted] = 100

    def extract_obstacles(self):
        self.obstacles = []
        for y in range(self.map_info.height):
            for x in range(self.map_info.width):
                if self.map_data[y][x] > 50:
                    world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                    world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                    self.obstacles.append((world_x, world_y))

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def world_to_map(self, point):
        return self._world_to_grid(point.x, point.y)

    def is_collision(self, p1, p2):
        x1 = p1.x if hasattr(p1, 'x') else p1[0]
        y1 = p1.y if hasattr(p1, 'y') else p1[1]
        x2 = p2.x if hasattr(p2, 'x') else p2[0]
        y2 = p2.y if hasattr(p2, 'y') else p2[1]

        gx1, gy1 = self._world_to_grid(x1, y1)
        gx2, gy2 = self._world_to_grid(x2, y2)

        dx = abs(gx2 - gx1)
        dy = -abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx + dy

        x, y = gx1, gy1
        while True:
            if not (0 <= x < self.map_info.width and 0 <= y < self.map_info.height):
                return True
            if self.inflated_data[y][x] > 50:
                return True
            if x == gx2 and y == gy2:
                break
            e2 = 2 * err
            if e2 >= dy:
                if x == gx2:
                    break
                err += dy
                x += sx
            if e2 <= dx:
                if y == gy2:
                    break
                err += dx
                y += sy

        return False


class RRTStarPlanner:
    def __init__(self, node):
        self.node = node
        self.mp = MapProcessor(node)
        self.start = None
        self.goal = None
        self.nodes = []
        self.robot_pos = Point(x=0.0, y=0.0, z=0.0)

        self.vis_path_pub = node.create_publisher(Path, '/rrt_path', queue_size=10)
        self.ctrl_path_pub = node.create_publisher(Float32MultiArray, '/path', queue_size=10)
        self.tree_pub = node.create_publisher(MarkerArray, '/rrt_tree', queue_size=10)
        # Subscribe to both ROS1 and ROS2 goal topics
        node.create_subscriber(PoseStamped, '/move_base_simple/goal', self.goal_callback)
        node.create_subscriber(PoseStamped, '/goal_pose', self.goal_callback)
        # Track robot position for replanning
        node.create_subscriber(Odometry, '/odom', self.odom_callback)

        node.loginfo("Waiting for /map ...")
        wait_for_message('/map', OccupancyGrid, node=node)
        node.loginfo("Planner initialized, waiting for goal...")

    class Node:
        def __init__(self, point, parent=None):
            self.point = point
            self.parent = parent
            self.cost = 0.0 if parent is None else parent.cost + math.hypot(
                point.x - parent.point.x, point.y - parent.point.y)

    def odom_callback(self, msg):
        self.robot_pos = msg.pose.pose.position

    def goal_callback(self, msg):
        self.goal = msg.pose.position
        if self.mp.map_info is not None:
            # Use robot's actual position as start (not hardcoded origin)
            self.start = Point(x=self.robot_pos.x, y=self.robot_pos.y, z=0.0)
            self.node.loginfo("Planning from (%.2f, %.2f) to (%.2f, %.2f)",
                              self.start.x, self.start.y, self.goal.x, self.goal.y)

            # Check if start or goal is inside an obstacle
            if self.mp.is_collision(self.start, self.start):
                self.node.logerr("Start position is inside an obstacle!")
                return
            if self.mp.is_collision(self.goal, self.goal):
                self.node.logerr("Goal is inside or too close to an obstacle! Pick a different goal.")
                return

            # Single attempt with enough iterations (faster than 3 retries)
            ok = self.plan_path()
            if not ok:
                # Fallback: try once more with reduced inflation
                self.node.logwarn("First attempt failed, retrying with reduced margin...")
                self.mp._inflate_obstacles(inflation_cells=4)
                ok = self.plan_path()
                self.mp._inflate_obstacles(inflation_cells=5)  # restore

    def _nearest_neighbor(self, target):
        tx = target[0] if isinstance(target, tuple) else target.x
        ty = target[1] if isinstance(target, tuple) else target.y
        min_dist = float('inf')
        min_idx = 0
        for i, node in enumerate(self.nodes):
            d = (node.point.x - tx)**2 + (node.point.y - ty)**2
            if d < min_dist:
                min_dist = d
                min_idx = i
        return min_idx

    def _steer(self, from_point, to_target, step_size):
        tx = to_target[0] if isinstance(to_target, tuple) else to_target.x
        ty = to_target[1] if isinstance(to_target, tuple) else to_target.y
        dx = tx - from_point.x
        dy = ty - from_point.y
        dist = math.hypot(dx, dy)
        if dist <= step_size:
            return Point(x=tx, y=ty, z=0.0)
        ratio = step_size / dist
        return Point(x=from_point.x + dx * ratio, y=from_point.y + dy * ratio, z=0.0)

    def _nearby_nodes(self, point, radius):
        indices = []
        for i, node in enumerate(self.nodes):
            d = math.hypot(node.point.x - point.x, node.point.y - point.y)
            if d < radius:
                indices.append(i)
        return indices

    def _smooth_path(self, goal_node):
        waypoints = []
        cur = goal_node
        while cur is not None:
            waypoints.append(cur)
            cur = cur.parent
        waypoints.reverse()

        if len(waypoints) <= 2:
            return goal_node

        max_seg_len = 1.5

        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            best_j = i + 1
            for j in range(len(waypoints) - 1, i, -1):
                seg_len = math.hypot(
                    waypoints[j].point.x - waypoints[i].point.x,
                    waypoints[j].point.y - waypoints[i].point.y)
                if seg_len <= max_seg_len and not self.mp.is_collision(waypoints[i].point, waypoints[j].point):
                    best_j = j
                    break

            waypoints[best_j].parent = waypoints[i]
            waypoints[best_j].cost = waypoints[i].cost + math.hypot(
                waypoints[best_j].point.x - waypoints[i].point.x,
                waypoints[best_j].point.y - waypoints[i].point.y)
            smoothed.append(waypoints[best_j])
            i = best_j

        self.node.loginfo("Path smoothed: %d -> %d waypoints", len(waypoints), len(smoothed))
        return goal_node

    def plan_path(self):
        if self.mp.map_data is None:
            self.node.logerr("No map data available")
            return False

        start_time = self.node.now()
        self.nodes = [self.Node(self.start)]

        max_iter = 12000
        goal_sample_rate = 0.25
        step_size = 0.8
        goal_tolerance = 0.3

        goal_node = None
        best_goal_cost = float('inf')
        first_solution_iter = -1

        # Sampling domain: bounding box of start->goal + generous margin
        sx, sy = self.start.x, self.start.y
        gx, gy = self.goal.x, self.goal.y
        x_min = min(sx, gx) - 4.0
        x_max = max(sx, gx) + 4.0
        y_min = min(sy, gy) - 4.0
        y_max = max(sy, gy) + 4.0

        for i in range(max_iter):
            # ---- Smart adaptive sampling ----
            if i < 40:
                # Bootstrap: dense near actual start position
                sample = (random.uniform(sx - 2, sx + 2),
                          random.uniform(sy - 2, sy + 2))
            elif i < 100:
                # Dense near goal
                sample = (random.uniform(gx - 2, gx + 2),
                          random.uniform(gy - 2, gy + 2))
            elif random.random() < goal_sample_rate:
                sample = (gx, gy)
            else:
                sample = (random.uniform(x_min, x_max), random.uniform(y_min, y_max))

            # Nearest neighbor
            nearest_idx = self._nearest_neighbor(sample)
            nearest = self.nodes[nearest_idx]

            # Steer
            new_point = self._steer(nearest.point, sample, step_size)

            # Collision check
            if self.mp.is_collision(nearest.point, new_point):
                continue

            # Dynamic search radius (RRT*)
            n = len(self.nodes)
            if n > 2:
                search_radius = min(step_size * 2.5, 20.0 * math.sqrt(math.log(n) / max(n, 2)))
            else:
                search_radius = step_size * 2.0

            # Find nearby nodes
            nearby_indices = self._nearby_nodes(new_point, search_radius)

            # Choose best parent
            best_parent = nearest
            best_cost = nearest.cost + math.hypot(
                new_point.x - nearest.point.x, new_point.y - nearest.point.y)
            for idx in nearby_indices:
                candidate = self.nodes[idx]
                edge_cost = math.hypot(
                    new_point.x - candidate.point.x, new_point.y - candidate.point.y)
                if candidate.cost + edge_cost < best_cost:
                    if not self.mp.is_collision(candidate.point, new_point):
                        best_parent = candidate
                        best_cost = candidate.cost + edge_cost

            # Add new node
            new_node = self.Node(new_point, best_parent)
            new_node.cost = best_cost
            self.nodes.append(new_node)

            # Rewire
            for idx in nearby_indices:
                candidate = self.nodes[idx]
                edge_cost = math.hypot(
                    new_point.x - candidate.point.x, new_point.y - candidate.point.y)
                if new_node.cost + edge_cost < candidate.cost:
                    if not self.mp.is_collision(new_point, candidate.point):
                        candidate.parent = new_node
                        candidate.cost = new_node.cost + edge_cost

            # Check goal reachability
            dist_to_goal = math.hypot(new_point.x - gx, new_point.y - gy)
            if dist_to_goal < goal_tolerance:
                if not self.mp.is_collision(new_point, self.goal):
                    total_cost = new_node.cost + dist_to_goal
                    if total_cost < best_goal_cost:
                        best_goal_cost = total_cost
                        goal_node = self.Node(self.goal, new_node)
                        goal_node.cost = total_cost
                        if first_solution_iter < 0:
                            first_solution_iter = i
                            self.node.loginfo("First path found at iteration %d, optimizing...", i)

            # Early stop
            if goal_node is not None and first_solution_iter >= 0:
                if i > first_solution_iter + 300:
                    self.node.loginfo("Early stop at iteration %d", i)
                    break

        elapsed = self.node.time_diff_sec(self.node.now(), start_time)

        if goal_node is not None:
            self._smooth_path(goal_node)
            self.publish_path(goal_node)
            path_length = goal_node.cost
            self.node.loginfo("RRT* done in %.3fs | Path: %.2fm | Tree: %d nodes",
                              elapsed, path_length, len(self.nodes))
            return True
        else:
            self.node.logerr("RRT* failed after %d iters (%.3fs)", max_iter, elapsed)
            return False

    def publish_path(self, goal_node):
        vis_path = Path()
        vis_path.header.frame_id = "map"
        self.node.stamp_header(vis_path.header)

        ctrl_path = Float32MultiArray()
        path_points = []

        current = goal_node
        while current is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position = current.point
            vis_path.poses.append(pose)

            path_points.append((current.point.x, current.point.y))
            current = current.parent

        vis_path.poses.reverse()
        path_points.reverse()

        for point in path_points:
            ctrl_path.data.extend([point[0], point[1]])

        self.vis_path_pub.publish(vis_path)
        self.ctrl_path_pub.publish(ctrl_path)
        self.node.loginfo("Published path: %d waypoints (visual), %d (control)",
                          len(vis_path.poses), len(path_points))

        self.publish_tree()

    def publish_tree(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        self.node.stamp_header(marker.header)
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        for node in self.nodes:
            if node.parent:
                marker.points.append(node.parent.point)
                marker.points.append(node.point)

        marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)


if __name__ == '__main__':
    node = CompatNode('rrt_star_planner')
    planner = RRTStarPlanner(node)
    spin(node)
