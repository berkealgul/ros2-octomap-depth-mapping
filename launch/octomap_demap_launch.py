#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_image_topic', default_value='depth/rect'),
        DeclareLaunchArgument('output_map_topic', default_value='octomap_fullmap'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('resolution', default_value='0.15'),
        DeclareLaunchArgument('camera_model/fx', default_value='0.7'),
        DeclareLaunchArgument('camera_model/fy', default_value='0.4'),
        DeclareLaunchArgument('camera_model/cx', default_value='0.12'),
        DeclareLaunchArgument('camera_model/cy', default_value='0.97'),
        Node(
            package='octomap_depth_mapping',
            executable='octomap_demap',
            output='screen',
            remappings=[('image_in', LaunchConfiguration('input_image_topic')),
                        ('map_out', LaunchConfiguration('output_map_topic'))],
            parameters=[{'resoluti1on': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'base_frame_id': LaunchConfiguration('base_frame_id'),
                         'camera_model/fx': LaunchConfiguration('camera_model/fx'),
                         'camera_model/fy': LaunchConfiguration('camera_model/fy'),
                         'camera_model/cx': LaunchConfiguration('camera_model/cx'),
                         'camera_model/cy': LaunchConfiguration('camera_model/cy')}]
        )
    ])
