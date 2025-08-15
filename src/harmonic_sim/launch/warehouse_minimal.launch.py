#!/usr/bin/env python3
"""
Minimal launch file for warehouse simulation - just Gazebo with the world
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_harmonic_sim = get_package_share_directory("harmonic_sim")
    
    # World file path
    world_file = os.path.join(pkg_harmonic_sim, "worlds", "blueprint_warehouse_detailed.world")
    
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=world_file,
            description="Full path to the world file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "verbose",
            default_value="true",
            description="Run Gazebo with verbose output",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Run Gazebo with GUI",
        )
    )

    # Initialize Arguments
    world = LaunchConfiguration("world")
    verbose = LaunchConfiguration("verbose")
    gui = LaunchConfiguration("gui")

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world
        ],
        output="screen",
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=LaunchConfiguration("gui")
    )

    return LaunchDescription(declared_arguments + [
        gzserver,
        gzclient,
    ])