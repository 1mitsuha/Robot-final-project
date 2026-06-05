#!/usr/bin/env python3
"""
ROS2 launch file — equivalent of spawn_obstacles.launch for ROS2 Humble+.

Usage:
    ros2 launch random_map_generator spawn_obstacles_ros2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # 1. Gazebo empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    # 2. Spawn obstacles node (delayed 3s)
    spawn_obstacles = Node(
        package='random_map_generator',
        executable='spawn_obstacles.py',
        name='spawn_obstacles_node',
        output='screen',
    )

    from launch.actions import TimerAction
    spawn_obstacles_delayed = TimerAction(
        period=3.0,
        actions=[spawn_obstacles],
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_obstacles_delayed,
    ])
