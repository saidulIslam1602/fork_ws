#!/usr/bin/env python3
"""
Launch Gazebo alone, with a selectable world.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('harmonic_sim')

    default_world = os.path.join(pkg_share, 'worlds', 'harmonic.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to world file to load'
    )

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items())

    return LaunchDescription([
        ik_node,
        fk_node,
        static_tf_base2lidar,
        world_arg, 
        gazebo
        ])
