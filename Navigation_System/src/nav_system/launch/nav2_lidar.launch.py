#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Evaluate at launch the value of the launch configuration 'namespace' 
    namespace = LaunchConfiguration('namespace')
    map_path = LaunchConfiguration('map')
    localization = LaunchConfiguration('localization')
    navigation = LaunchConfiguration('navigation')
    x = LaunchConfiguration('loc_x', default=0.0)
    y = LaunchConfiguration('loc_y', default=0.0)
    yaw = LaunchConfiguration('loc_yaw', default=0.0)
    params_dir = os.path.join(get_package_share_directory('nav_system'), 'params')

    # Declares an action to allow users to pass the robot namespace from the 
    # CLI into the launch description as an argument.
    namespace_argument = DeclareLaunchArgument(
        'namespace', 
        default_value='',
        description='Robot namespace')

    map_argument = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(params_dir, 'office_19ex.yaml'),
        description='Path to map\'s YAML file')

#    localization_argument = DeclareLaunchArgument(
#        'localization',
#        default_value=os.path.join(params_dir, 'localization.yaml'),
#        description='Path to YAML AMCL localization configuration file')

    navigation_argument = DeclareLaunchArgument(
        'navigation',
        default_value=os.path.join(params_dir, 'nav2_params.yaml'),
        description='Path to YAML NAV2 navigation configuration file')

    params_file = RewrittenYaml(
            source_file=os.path.join(params_dir, 'localization.yaml'),
            root_key='',
            param_rewrites={
                'x': x,
                'y': y,
                'yaw': yaw
            },
            convert_types=True)

    
    # Declares an action that will launch a node when executed by the launch description.
    # This node is responsible for configuring the RPLidar sensor.    
    lidar_node = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('create3_lidar_slam'),
                'launch/sensors_launch.py')]),
            ),
        ])

    localization_node = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'),
                'launch/localization_launch.py')]),
            launch_arguments={
                'params_file': params_file,
                'map': map_path,
                    }.items()
            ),
        ])

    navigation_node = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'),
                'launch/navigation_launch.py')]),
            launch_arguments={
                'params_file': navigation,
                }.items()
            ),
        ])
    
    # Launches all named actions
    return LaunchDescription([
        namespace_argument,
        map_argument,
        #localization_argument,
        navigation_argument,
        TimerAction(
            period=2.0,
            actions=[lidar_node]
        ),
        TimerAction(
            period=7.0,
            actions=[localization_node]
        ),
        TimerAction(
            period=15.0,
            actions=[navigation_node]
        )
    ])
