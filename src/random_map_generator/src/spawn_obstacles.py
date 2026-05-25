#!/usr/bin/env python3
import rospy
import random
import numpy as np
from PIL import Image
import yaml
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

MAP_WIDTH = 20.0
MAP_HEIGHT = 20.0
RESOLUTION = 0.1
OCCUPANCY_THRESHOLD = 0.65


class ObstacleGenerator:
    def __init__(self):
        self.obstacles = []

    def generate_in_gazebo(self, num_obstacles=10):
        rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.loginfo("Service available, spawning %d obstacles", num_obstacles)

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
            model_xml = self._create_sdf_model(size_x, size_y, size_z)
            pose = Pose(position=Point(x, y, size_z / 2),
                        orientation=Quaternion(0, 0, 0, 1))

            try:
                req = SpawnModelRequest()
                req.model_name = model_name
                req.model_xml = model_xml
                req.robot_namespace = ""
                req.initial_pose = pose
                req.reference_frame = "world"
                resp = spawn_model(req)
                if resp.success:
                    rospy.loginfo("Spawned %s at (%.2f, %.2f)", model_name, x, y)
                    spawned += 1
                else:
                    rospy.logwarn("Spawn failed: %s", resp.status_message)
            except Exception as e:
                rospy.logerr("Spawn error: %s", str(e))
                break

        rospy.loginfo("Total obstacles spawned: %d", spawned)

    def _create_sdf_model(self, size_x, size_y, size_z):
        r = random.random()
        g = random.random()
        b = random.random()
        return (
            '<sdf version="1.6">'
            '<model><static>true</static><link name="link">'
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
    rospy.init_node('spawn_random_obstacles')

    ppgm = rospy.get_param('pgm_path')
    pyaml = rospy.get_param('yaml_path')

    obstacle_gen = ObstacleGenerator()
    obstacle_gen.generate_in_gazebo(num_obstacles=15)

    map_gen = PGMMapGenerator()
    for obs in obstacle_gen.obstacles:
        x, y, size_x, size_y = obs
        map_gen.add_obstacle(x, y, size_x, size_y)

    map_gen.save(ppgm, pyaml)
    rospy.loginfo("PGM map generated with %d obstacles", len(obstacle_gen.obstacles))
