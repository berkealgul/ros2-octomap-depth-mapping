#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_image_topic', default_value='depth/rect'),
        DeclareLaunchArgument('input_odom_topic', default_value='odom'),
        DeclareLaunchArgument('output_map_topic', default_value='octomap_fullmap'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('camera_model/fx', default_value='524.0'),
        DeclareLaunchArgument('camera_model/fy', default_value='524.0'),
        DeclareLaunchArgument('camera_model/cx', default_value='316.8'),
        DeclareLaunchArgument('camera_model/cy', default_value='238.5'),
        DeclareLaunchArgument('encoding', default_value='mono16'),
        DeclareLaunchArgument('padding', default_value='1'),
        DeclareLaunchArgument('kernel_size', default_value='1'),
        Node(
            package='demaloc',
            executable='demaloc',
            output='screen',
            remappings=[('image_in', LaunchConfiguration('input_image_topic')),
                        ('odom_in', LaunchConfiguration('input_odom_topic')),
                        ('map_out', LaunchConfiguration('output_map_topic'))],
            parameters=[{'resolution': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'camera_model/fx': LaunchConfiguration('camera_model/fx'),
                         'camera_model/fy': LaunchConfiguration('camera_model/fy'),
                         'camera_model/cx': LaunchConfiguration('camera_model/cx'),
                         'camera_model/cy': LaunchConfiguration('camera_model/cy'),
                         'encoding': LaunchConfiguration('encoding'),
                         'padding': LaunchConfiguration('padding'),
                         'kernel_size': LaunchConfiguration('kernel_size')}]
        )
    ])
