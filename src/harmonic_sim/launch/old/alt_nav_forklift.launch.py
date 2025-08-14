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


    # ──‑‑‑‑ Navigation  (Nav2)  ‑‑‑‑──────────────────────────────────────────────────
    # 1) Parameters file for Nav2
    nav2_params = os.path.join(
        get_package_share_directory('harmonic_sim'),  # keep everything in your own pkg
        'config',
        'nav2_params.yaml'  # create / tweak this YAML
    )
    nav2_map = os.path.join(
        get_package_share_directory('harmonic_sim'),  # keep everything in your own pkg
        'maps',
        'my_map_pro.yaml'  # create / tweak this YAML
    )

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')]
        ),
        launch_arguments={
            'map': '/home/ubuntu/ros2_ws/src/harmonic_sim/maps/my_map_pro.yaml',  # <-- rename this key to 'map'
            'use_sim_time': 'true',
            'params_file': '/home/ubuntu/ros2_ws/src/harmonic_sim/config/nav2_params.yaml',
            'autostart': 'true'
        }.items()
    )

    # navigation stack (planner, controller, behaviour tree …)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': '/home/ubuntu/ros2_ws/src/harmonic_sim/config/nav2_params.yaml',
            'autostart': 'true'
        }.items()
    )



    # Start Rviz2  --------------------------------------------------------------

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', '/home/ubuntu/ros2_ws/src/harmonic_sim/rviz2_configs/navigation.rviz'],
        output='screen'
    )


    return LaunchDescription([
        world_arg,
        gazebo,
        ik_node,
        fk_node,
        static_tf_base2lidar,
        rviz_node,
        nav2_localization,
        nav2_navigation
    ])
