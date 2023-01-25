#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_model/fx', default_value='524.0'),
        DeclareLaunchArgument('sensor_model/fy', default_value='524.0'),
        DeclareLaunchArgument('sensor_model/cx', default_value='316.8'),
        DeclareLaunchArgument('sensor_model/cy', default_value='238.5'),
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        DeclareLaunchArgument('encoding', default_value='mono16'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('padding', default_value='1'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('filename', default_value=''),
        DeclareLaunchArgument('save_on_shutdown', default_value='false'),
        Node(
            package='octomap_depth_mapping',
            executable='octomap_depth_mapping',
            name='octodm_node',
            output='screen',
            remappings=[('image_in', 'depth/rect'),
                        ('pose_in', 'pose'),
                        ('map_out', 'octomap_fullmap')],
            parameters=[{'resolution': LaunchConfiguration('resolution'),
                         'frame_id': LaunchConfiguration('frame_id'),
                         'sensor_model/fx': LaunchConfiguration('sensor_model/fx'),
                         'sensor_model/fy': LaunchConfiguration('sensor_model/fy'),
                         'sensor_model/cx': LaunchConfiguration('sensor_model/cx'),
                         'sensor_model/cy': LaunchConfiguration('sensor_model/cy'),
                         'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
                         'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
                         'sensor_model/min': LaunchConfiguration('sensor_model/min'),
                         'sensor_model/max': LaunchConfiguration('sensor_model/max'),
                         'encoding': LaunchConfiguration('encoding'),
                         'padding': LaunchConfiguration('padding'),
                         'filename': LaunchConfiguration('filename'),
                         'save_on_shutdown': LaunchConfiguration('save_on_shutdown'),
                         'width': LaunchConfiguration('width'),
                         'height': LaunchConfiguration('height')
                        }]
        )
    ])
