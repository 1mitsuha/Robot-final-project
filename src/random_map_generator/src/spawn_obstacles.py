#!/usr/bin/env python3
import sys
import random
import os
import numpy as np
from PIL import Image
import yaml

# Ensure ros_compat can be imported (try local dir first, then Robot-Planner)
_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)
_compat_dir = os.path.join(os.path.dirname(os.path.dirname(_script_dir)),
                           'Robot-Planner', 'scripts')
if _compat_dir not in sys.path:
    sys.path.insert(0, _compat_dir)

from ros_compat import (
    CompatNode, spin,
    Point, Quaternion, Pose,
    call_spawn_service, get_spawn_service_name, get_spawn_service_type,
)

MAP_WIDTH = 20.0
MAP_HEIGHT = 20.0
RESOLUTION = 0.1
OCCUPANCY_THRESHOLD = 0.65


class ObstacleGenerator:
    def __init__(self, node):
        self.node = node
        self.obstacles = []

    def generate_in_gazebo(self, num_obstacles=10):
        service_name = get_spawn_service_name()
        self.node.loginfo("Waiting for %s service...", service_name)
        self.node.wait_for_service(service_name)
        spawn_client = self.node.create_client(
            get_spawn_service_type(), service_name)
        self.node.loginfo("Service available, spawning %d obstacles", num_obstacles)

        spawned = 0
        for i in range(num_obstacles * 3):
            if spawned >= num_obstacles:
                break

            x = random.uniform(-8, 8)
            y = random.uniform(-8, 8)
            size_x = random.uniform(0.5, 3.0)
            size_y = random.uniform(0.5, 3.0)
            size_z = random.uniform(0.5, 2.0)
            half_x = size_x / 2
            half_y = size_y / 2

            near_start = (abs(x) < half_x + 2.0) and (abs(y) < half_y + 2.0)
            if near_start:
                continue

            self.obstacles.append((x, y, size_x, size_y))

            model_name = "obstacle_%d" % spawned
            model_xml = self._create_sdf_model(model_name, size_x, size_y, size_z)
            pose = Pose(position=Point(x=x, y=y, z=size_z / 2),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            try:
                resp = call_spawn_service(
                    client=spawn_client,
                    model_name=model_name,
                    model_xml=model_xml,
                    robot_namespace="",
                    initial_pose=pose,
                    reference_frame="world",
                    node=self.node,
                )

                if resp.success:
                    self.node.loginfo("Spawned %s at (%.2f, %.2f)", model_name, x, y)
                    spawned += 1
                else:
                    self.node.logwarn("Spawn failed: %s", resp.status_message)
            except Exception as e:
                self.node.logerr("Spawn error: %s", str(e))
                break

        self.node.loginfo("Total obstacles spawned: %d", spawned)

    def _create_sdf_model(self, name, size_x, size_y, size_z):
        r = random.random()
        g = random.random()
        b = random.random()
        return (
            '<?xml version="1.0" ?>'
            '<sdf version="1.6">'
            '<model name="' + name + '"><static>true</static><link name="link">'
            '<collision name="collision"><geometry><box>'
            '<size>' + str(size_x) + ' ' + str(size_y) + ' ' + str(size_z) + '</size>'
            '</box></geometry></collision>'
            '<visual name="visual"><geometry><box>'
            '<size>' + str(size_x) + ' ' + str(size_y) + ' ' + str(size_z) + '</size>'
            '</box></geometry>'
            '<material><ambient>' + str(r) + ' ' + str(g) + ' ' + str(b) + ' 1</ambient></material>'
            '</visual></link></model></sdf>'
        )


class PGMMapGenerator:
    def __init__(self):
        self.grid = np.ones((
            int(MAP_HEIGHT / RESOLUTION),
            int(MAP_WIDTH / RESOLUTION)
        ), dtype=np.uint8) * 255

    def world_to_grid(self, x, y):
        grid_x = int((x + MAP_WIDTH / 2) / RESOLUTION)
        grid_y = int((y + MAP_HEIGHT / 2) / RESOLUTION)
        return grid_x, grid_y

    def add_obstacle(self, x, y, size_x, size_y):
        half_x = size_x / 2
        half_y = size_y / 2

        near_start = (abs(x) < half_x + 2.0) and (abs(y) < half_y + 2.0)
        if near_start:
            return

        x_min, y_min = self.world_to_grid(x - half_x, y - half_y)
        x_max, y_max = self.world_to_grid(x + half_x, y + half_y)

        x_min = max(0, x_min)
        x_max = min(self.grid.shape[1], x_max)
        y_min_img = max(0, self.grid.shape[0] - y_max)
        y_max_img = min(self.grid.shape[0], self.grid.shape[0] - y_min)

        if y_min_img > y_max_img:
            y_min_img, y_max_img = y_max_img, y_min_img

        self.grid[y_min_img:y_max_img, x_min:x_max] = 0

    def save(self, pgm_path, yaml_path):
        ros_grid = np.where(self.grid == 0, 0, 100).astype(np.uint8)
        Image.fromarray(ros_grid).save(pgm_path)

        yaml_data = {
            "image": pgm_path,
            "resolution": RESOLUTION,
            "origin": [-MAP_WIDTH / 2, -MAP_HEIGHT / 2, 0.0],
            "occupied_thresh": OCCUPANCY_THRESHOLD,
            "free_thresh": 0.25,
            "negate": 0
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)


if __name__ == "__main__":
    node = CompatNode('spawn_random_obstacles')

    # Declare parameters (ROS2 requires explicit declaration)
    node.declare_parameter('pgm_path', '')
    node.declare_parameter('yaml_path', '')

    ppgm = node.get_parameter('pgm_path')
    pyaml = node.get_parameter('yaml_path')
    node.loginfo("pgm_path=%s, yaml_path=%s", ppgm, pyaml)

    obstacle_gen = ObstacleGenerator(node)
    obstacle_gen.generate_in_gazebo(num_obstacles=15)

    map_gen = PGMMapGenerator()
    for obs in obstacle_gen.obstacles:
        x, y, size_x, size_y = obs
        map_gen.add_obstacle(x, y, size_x, size_y)

    map_gen.save(ppgm, pyaml)
    node.loginfo("PGM map generated with %d obstacles", len(obstacle_gen.obstacles))
    CompatNode.shutdown()
