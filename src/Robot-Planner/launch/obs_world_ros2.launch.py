#!/usr/bin/env python3
"""
ROS2 launch file — equivalent of obs_world.launch for ROS2 Humble+.

Key differences from ROS1:
  - empty_world.launch.py already provides robot_state_publisher + robot spawn
  - ROS2 spawns entity named after TURTLEBOT3_MODEL (e.g. "waffle"), not "turtlebot3_waffle"
  - ROS2 joint_states are on /joint_states directly (no /gazebo prefix remap needed)
  - Map uses nav2_map_server instead of map_server

Usage:
    ros2 launch Robot-Planner obs_world_ros2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
        description='TurtleBot3 model [burger, waffle, waffle_pi]'
    )
    model = LaunchConfiguration('model')

    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    robot_planner_share = get_package_share_directory('Robot-Planner')
    random_map_share = get_package_share_directory('random_map_generator')

    # 1. Gazebo empty world (includes: gzserver, gzclient, robot_state_publisher, spawn_turtlebot3)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    # 2. Spawn obstacles (delayed 15s to let Gazebo start)
    spawn_obstacles_node = Node(
        package='random_map_generator',
        executable='spawn_obstacles.py',
        name='spawn_obstacles_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pgm_path': os.path.join(robot_planner_share, 'maps', 'gazebo_map.pgm'),
            'yaml_path': os.path.join(robot_planner_share, 'maps', 'gazebo_map.yaml'),
            'obstacle_count': 20,
            'x_range': [-10, 10],
            'y_range': [-10, 10],
        }],
    )
    spawn_obstacles_delayed = TimerAction(
        period=15.0,
        actions=[spawn_obstacles_node],
    )

    # 3. Static TF: map -> odom (identity)
    #    Gazebo diff_drive already publishes odom -> base_footprint.
    #    This bridges the gap so the full chain is: map -> odom -> base_footprint -> ...
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # 4. Map publisher — starts immediately, waits internally for PGM+YAML to be ready
    #    Uses our simple publisher instead of nav2_map_server (which needs lifecycle activation)
    map_publisher_node = Node(
        package='Robot-Planner',
        executable='map_publisher.py',
        name='map_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': os.path.join(robot_planner_share, 'maps', 'gazebo_map.yaml'),
        }],
    )

    # 5. Planner (RRT*)
    planner_node = Node(
        package='Robot-Planner',
        executable='planner.py',
        name='planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 6. Controller (PID)
    controller_node = Node(
        package='Robot-Planner',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 7. RViz2 — pre-configured with Map, Path, Tree, RobotModel, LaserScan
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(random_map_share, 'final_ros2.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        model_arg,
        gazebo_launch,
        spawn_obstacles_delayed,
        map_odom_tf,
        map_publisher_node,
        planner_node,
        controller_node,
        rviz_node,
    ])
