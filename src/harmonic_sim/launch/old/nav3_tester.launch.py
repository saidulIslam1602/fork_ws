#!/usr/bin/env python3
"""
forklift_sim.launch.py

Launch Gazebo with the harmonic world, the custom kinematics nodes,
a base–lidar static transform, the full Nav2 stack (localisation + navigation)
and RViz2.

There are **no hard‑coded absolute paths** in this file: every resource is
resolved at run‑time via `get_package_share_directory`, so the package can be
moved to any workspace or computer without edits.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Package‑share roots
    # -------------------------------------------------------------------------
    pkg_harmonic_sim = get_package_share_directory('harmonic_sim')
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # -------------------------------------------------------------------------
    # Files inside harmonic_sim
    # -------------------------------------------------------------------------
    default_world_path = os.path.join(pkg_harmonic_sim, 'worlds', 'harmonic.world')
    nav2_params_path   = os.path.join(pkg_harmonic_sim, 'config', 'nav2_params.yaml')
    nav2_map_path      = os.path.join(pkg_harmonic_sim, 'maps',   'my_map_pro.yaml')
    rviz_config_path   = os.path.join(pkg_harmonic_sim, 'rviz2_configs', 'navigation.rviz')

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to the Gazebo world file to load'
    )

    # -------------------------------------------------------------------------
    # Gazebo server + client
    # -------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )

    # -------------------------------------------------------------------------
    # Custom kinematics nodes
    # -------------------------------------------------------------------------
    ik_node = Node(
        package='fork_kinematics',
        executable='inverse_kinematics_node.py',
        name='inverse_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fk_node = Node(
        package='fork_kinematics',
        executable='forward_kinematics_node.py',
        name='forward_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Static transform: base_link → lidar_link
    # -------------------------------------------------------------------------
    static_tf_base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        # x  y  z  roll pitch yaw  from‑frame  to‑frame
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Nav2 localisation
    # -------------------------------------------------------------------------
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map':         nav2_map_path,
            'params_file': nav2_params_path,
            'use_sim_time':'true',
            'autostart':   'true'
        }.items()
    )

    # -------------------------------------------------------------------------
    # Nav2 navigation (planner, controller, BT, …)
    # -------------------------------------------------------------------------
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time':'true',
            'autostart':   'true'
        }.items()
    )

    # -------------------------------------------------------------------------
    # RViz2
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # Assemble the LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription([
        world_arg,
        gazebo,
        ik_node,
        fk_node,
        static_tf_base2lidar,
        nav2_localization,
        nav2_navigation,
        rviz_node
    ])
