#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("harmonic_sim"), "worlds", "complete_warehouse_complex.world"]
            ),
            description="Full path to the world file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo in headless mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    gui = LaunchConfiguration("gui")

    # Get the package directory
    pkg_harmonic_sim = get_package_share_directory("harmonic_sim")
    
    # Forklift model path
    forklift_model_path = os.path.join(pkg_harmonic_sim, "models", "forklift", "model.sdf")
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        condition=UnlessCondition(headless),
        launch_arguments={
            "world": world,
            "verbose": verbose,
            "gui": gui,
        }.items(),
    )

    # Gazebo headless launch
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        condition=IfCondition(headless),
        launch_arguments={
            "world": world,
            "verbose": verbose,
            "gui": "false",
        }.items(),
    )

    # Spawn forklift at a good starting position
    spawn_forklift = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "forklift",
            "-file", forklift_model_path,
            "-x", "0.0",
            "-y", "-100.0",
            "-z", "1.0",
            "-Y", "0.0"
        ],
        output="screen",
    )

    # RViz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("harmonic_sim"), "rviz2_configs", "navigation.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Static transform publisher for map to odom
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    nodes = [
        gazebo,
        gazebo_headless,
        spawn_forklift,
        rviz_node,
        static_tf,
    ]

    return LaunchDescription(declared_arguments + nodes) 