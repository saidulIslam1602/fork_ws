#!/usr/bin/env python3
"""
Start kinematics nodes, static TF, Nav2 (localisation + navigation) and RViz.
Assumes Gazebo / simulation clock is already running.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('harmonic_sim')

    # ── Kinematics Nodes ───────────────────────────────────────────
    ik_node = Node(
        package='fork_kinematics',
        executable='inverse_kinematics_node.py',
        name='inverse_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen')

    fk_node = Node(
        package='fork_kinematics',
        executable='forward_kinematics_node.py',
        name='forward_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen')

    # ── Static TF  base_link ➜ lidar_link ──────────────────────────
    static_tf_base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0', '0', '0.2',        # x y z
            '0', '0', '0',          # R P Y
            'base_link', 'lidar_link'],
        output='screen')

    # ── Nav2:  localisation + navigation  ─────────────────────────
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_map    = os.path.join(pkg_share, 'maps',   'my_map_pro.yaml')

    nav2_localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'localization_launch.py')),
        launch_arguments={
            'map': nav2_map,
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
            # ---- initial pose (0, 0, 180°) ----
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_a': '3.14159'
        }.items())

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file':  nav2_params,
            'autostart':    'true'
        }.items())


    return LaunchDescription([
        ik_node,
        fk_node,
        static_tf_base2lidar,
        nav2_localisation,
        nav2_navigation
    ])
