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
            'use_velocity_smoother': 'false',
            'use_smoother':          'false',
        }.items())

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file':  nav2_params,
            'autostart':    'true', 
            'use_velocity_smoother': 'false',
            'use_smoother':          'false',
        }.items())

    # ── RViz  ─────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'rviz2_configs', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([
        nav2_localisation,
        nav2_navigation,
        rviz_node
    ])
