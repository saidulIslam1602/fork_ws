#!/usr/bin/env python3
"""
Launch Gazebo with the detailed blueprint warehouse world (simplified version).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('harmonic_sim')

    # Use the detailed blueprint warehouse world
    default_world = os.path.join(pkg_share, 'worlds', 'blueprint_warehouse_detailed.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the detailed blueprint warehouse world file'
    )

    # ── Launch Gazebo ─────────────────────────────────────────────
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
        world_arg,
        gazebo
    ]) 