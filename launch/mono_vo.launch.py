#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Generates the launch description for the mono_vo node.
    """

    pkg_share = get_package_share_directory('mono_vo')
    
    default_params_file = os.path.join(
        pkg_share,
        'config',
        'params.yaml'
    )

    default_rviz_config_file = os.path.join(pkg_share, 'rviz', 'mono_vo.rviz')

    declared_arguments = []

    # General arguments
    declared_arguments.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS logging level (debug, info, warn, error, fatal).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization if true.'
    ))

    # Input topic arguments
    declared_arguments.append(DeclareLaunchArgument(
        'image_topic',
        default_value='/cam_1/image_raw',
        description='Input topic for the rectified camera image.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'cam_info_topic',
        default_value='/cam_1/camera_info',
        description='Input topic for the camera info.'
    ))

    # Output topic arguments
    declared_arguments.append(DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Output topic for odometry.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/pointcloud',
        description='Output topic for the map point cloud.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'path_topic',
        default_value='/path',
        description='Output topic for the camera trajectory path.'
    ))

    mono_vo_node = Node(
        # General Node configuration
        package='mono_vo',             
        executable='mono_vo',     
        name='mono_vo',           # This name must match the one in your YAML file
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

        # Load parameters from the specified file and set use_sim_time
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],

        # Remap topics based on launch arguments
        remappings=[
            # Subscriptions
            ('/cam_1/image_raw', LaunchConfiguration('image_topic')),
            ('/cam_1/camera_info', LaunchConfiguration('cam_info_topic')),
            # Publications
            ('/odom', LaunchConfiguration('odom_topic')),
            ('/pointcloud', LaunchConfiguration('pointcloud_topic')),
            ('/path', LaunchConfiguration('path_topic')),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_file],
        # Conditionally launch RViz2 based on the 'launch_rviz' argument
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    ld = LaunchDescription()

    for argument in declared_arguments:
        ld.add_action(argument)

    ld.add_action(mono_vo_node)
    ld.add_action(rviz_node)

    return ld