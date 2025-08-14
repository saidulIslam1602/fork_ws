#!/usr/bin/env python3
"""
Start map_server + AMCL under a lifecycle manager, then launch the
Nav2 navigation stack (planner / controller / BT navigator) once AMCL
is active.  RViz comes up immediately.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share   = get_package_share_directory('harmonic_sim')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_map    = os.path.join(pkg_share, 'maps',   'my_map_pro.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz2_configs', 'navigation.rviz')

    # ─────────────── map_server ───────────────
    map_server = LifecycleNode(
        package    ='nav2_map_server',
        executable ='map_server',
        name       ='map_server',
        namespace  ='',
        output     ='screen',
        parameters=[nav2_params,
                    {'use_sim_time': True,
                     'yaml_filename': nav2_map}],
    )

    # ─────────────── AMCL ───────────────
    amcl_node = LifecycleNode(
        package    ='nav2_amcl',
        executable ='amcl',
        name       ='amcl',
        namespace  ='',
        output     ='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
    )

    # ─── localisation lifecycle manager ───
    loc_manager = LifecycleNode(
        package    ='nav2_lifecycle_manager',
        namespace  ='', 
        executable ='lifecycle_manager',
        name       ='lifecycle_manager_localization',
        output     ='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart':    True,                 # configure+activate both nodes
            'node_names':   ['map_server', 'amcl']
        }],
    )

    # ─── Nav2 navigation stack (planner / controller / BT) ───
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time':          'true',
            'params_file':           nav2_params,
            'autostart':             'true',
            # no 'map' argument → navigation_launch.py WILL NOT duplicate map_server
            'use_velocity_smoother': 'false',
            'use_smoother':          'false',
        }.items()
    )

    # Start the navigation stack **after AMCL becomes ACTIVE**
    start_nav_evt = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=amcl_node,
            goal_state           ='active',
            entities             =[nav2_navigation],
        )
    )

    # ─────────────── RViz ───────────────
    rviz = Node(
        package    ='rviz2',
        executable ='rviz2',
        name       ='rviz2',
        output     ='screen',
        parameters =[{'use_sim_time': True}],
        arguments  =['-d', rviz_config],
    )

    return LaunchDescription([
        map_server,
        amcl_node,
        loc_manager,
        start_nav_evt,      # << navigation will be spawned later
        rviz,
    ])
