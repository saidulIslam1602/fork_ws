# forklift_sim.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import Node as RosNode  # for static_transform_publisher

def generate_launch_description():


    default_world_path = os.path.join(
        get_package_share_directory('harmonic_sim'),
        'worlds',
        'harmonic.world'  # Use your new file
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Path to the world file'
    )

    # Include the gazebo_ros launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    """
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',                     # <== note the different package
                'gz_sim.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            # forwarded to `gz sim` exactly as typed:
            'gz_args': '-r -v4'             # run & verbose
        }.items())
    """
    ##################################################################
    # Transformes

    ik_node = Node(
        package='fork_kinematics',
        executable='inverse_kinematics_node.py',
        name='inverse_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fk_node = Node(
        package='fork_kinematics',
        executable='forward_kinematics_node.py',  # or 'forward_kinematics_node.py'
        name='forward_kinematics',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    # 2) base_link -> lidar_link
    static_tf_base2lidar = RosNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        parameters=[{'use_sim_time': True}],  # IMPORTANT
        arguments=[
            '0', '0', '0.2',  # x y z
            '0', '0', '0',  # roll pitch yaw
            'base_link', 'lidar_link'
        ],
        output='screen'
    )


    # Start Rviz2  --------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        #arguments=['-d', '/home/ubuntu/ros2_ws/src/harmonic_sim/rviz2_configs/mapping1.rviz'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        ik_node,
        fk_node,
        static_tf_base2lidar,
        rviz_node,
    ])
