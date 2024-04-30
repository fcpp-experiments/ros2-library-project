#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('robot_name', default_value='',
                          description='Robot name'),
    DeclareLaunchArgument('ros_domain', default_value='100',
                          description='ROS_DOMAIN_ID for proxies'),
    DeclareLaunchArgument('backup_storage', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to save past action file.'),
    DeclareLaunchArgument('use_proxies', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to enable ROS2 proxies.'),
    DeclareLaunchArgument('dock_enabled', default_value='0',
                          choices=['0', '1', '2'],
                          description='Whether to return to initial position on successful goal and dock to base station.'),
    DeclareLaunchArgument('sleep_time', default_value='0',
                          description='Time in seconds robots waits after reaching a navigation position'),
    DeclareLaunchArgument('board_proxies', default_value='true',
                          choices=['true', 'false'],
                          description='Whether proxies are used in board or server mode.'),
    DeclareLaunchArgument('capture_camera', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to enable camera capture for computer vision.'),
    DeclareLaunchArgument('capture_server_ip', default_value='192.168.10.50',
                          description='Server\'s IP for computer vision'),
    DeclareLaunchArgument('capture_server_port', default_value='6789',
                          description='port for computer vision'),
    DeclareLaunchArgument('resolution_x', default_value='3264',
                          description='Captured image width'),
    DeclareLaunchArgument('resolution_y', default_value='2448',
                          description='Captured image height'),
    DeclareLaunchArgument('focus_value', default_value='312',
                          description='Focus depth for the camera'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    ros_domain = LaunchConfiguration('ros_domain')
    backup_storage = LaunchConfiguration('backup_storage')
    use_proxies = LaunchConfiguration('use_proxies')
    dock_enabled = LaunchConfiguration('dock_enabled')
    sleep_time = LaunchConfiguration('sleep_time')
    capture_camera = LaunchConfiguration('capture_camera')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    board_proxies = LaunchConfiguration('board_proxies')
    server = LaunchConfiguration('capture_server_ip')
    port = LaunchConfiguration('capture_server_port')
    resolution_x = LaunchConfiguration('resolution_x')
    resolution_y = LaunchConfiguration('resolution_y')
    focus_value = LaunchConfiguration('focus_value')
    inbound_name = 'inbound_proxy'
    outbound_name = 'outbound_proxy'
    zero = LaunchConfiguration('zero', default=0.0)
    if ('ROS_DOMAIN_ID' in os.environ and os.environ['ROS_DOMAIN_ID']):
        inbound_name = inbound_name+'_'+str(os.environ['ROS_DOMAIN_ID'])
        outbound_name = outbound_name+'_'+str(os.environ['ROS_DOMAIN_ID'])

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(''),

        # requires source Robot_Reader
        Node(
            package="robot_reader",
            executable="robot_reader",
            output='screen',
            arguments=[namespace, robot_name, zero, zero, zero]
        ),

        # requires source Robot_Writer
        Node(
            package="robot_writer",
            executable="robot_writer",
            output='screen',
            parameters=[
                {'backup_storage': backup_storage}
                ],
            arguments=[namespace, robot_name, zero, zero, zero]
        ),
        Node(
            package="nav_system",
            executable="navigator",
            output='screen',
            arguments=[namespace],
            parameters=[
                {'x':   x},
                {'y':   y},
                {'yaw': yaw},
                {'dock_enabled': dock_enabled},
                {'sleep_time': sleep_time},
                {'capture_camera': capture_camera}
                ]
        ),
        Node(
            package="camera_capture",
            executable="camera_capture",
            output='screen',
            parameters=[
                {'server' : server},
                {'port' : port},
                {'resolution_x' : resolution_x},
                {'resolution_y' : resolution_y},
                {'focus_value' : focus_value}
            ],
            condition=IfCondition(capture_camera)
        ),
    ])

    proxies = GroupAction([
        # Require source Proxies
        Node(
            name=inbound_name,
            package="inbound_proxy",
            executable="board_inbound_proxy",
            output='screen',
            arguments=[robot_name, ros_domain]
            ),

        # Require source Proxies
        Node(
            name=outbound_name,
            package="outbound_proxy",
            executable="board_outbound_proxy",
            output='screen',
            arguments=[robot_name, ros_domain]
            )
        ],
        condition=IfCondition(use_proxies)
        )

    navigation_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav_system'),
                'launch/nav2_lidar.launch.py')]),
            launch_arguments={'loc_x':x, 'loc_y':y, 'loc_yaw': yaw}.items()
            )


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    ld.add_action(proxies)
    ld.add_action(navigation_node)
    return ld
