#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_image_topic', default_value='depth/rect'),
        DeclareLaunchArgument('resolution', default_value='0.15'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('sensor_model/max_range', default_value='-1.0'),
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        Node(
            package='octomap_depth_mağğing',
            executable='demaloc',
            output='screen',
            remappings=[('cloud_in', LaunchConfiguration('input_cloud_topic'))],
            parameters=[{'resolution': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'base_frame_id': LaunchConfiguration('base_frame_id'),
                         'compress_map': LaunchConfiguration('compress_map'),
                         'sensor_model/max_range': LaunchConfiguration('sensor_model/max_range'),
                         'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
                         'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
                         'sensor_model/min': LaunchConfiguration('sensor_model/min'),
                         'sensor_model/max': LaunchConfiguration('sensor_model/max')}]
        )
    ])
