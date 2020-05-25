"""Launch Gaze Correlation"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    config_gaze_correlation = LaunchConfiguration('config_gaze_correlation', default=os.path.join(get_package_share_directory(
        'gaze_correlation'), 'config', 'gaze_correlation.yaml'))
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory(
        'gaze_correlation'), 'config', 'rviz2.rviz'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_gaze_correlation',
            default_value=config_gaze_correlation,
            description='Path to config for gaze_correlation'),
        DeclareLaunchArgument(
            'config_rviz2',
            default_value=config_rviz2,
            description='Path to config for RViz2'),

        Node(
            package='gaze_correlation',
            node_executable='gaze_correlation',
            node_name='gaze_correlation',
            node_namespace='',
            output='screen',
            parameters=[config_gaze_correlation],
            remappings=[('gaze', 'gaze'),
                        ('geometric_primitives', 'gpf/geometric_primitives'),
                        ('object_of_interest', 'gaze_correlation/object_of_interest'),
                        ('point_of_gaze', 'ecard/gaze_correlation/point_of_gaze'),
                        ('visualisation_markers', 'gaze_correlation/visualisation_markers')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('gaze_correlation'), 'launch',
                              'rviz2.launch.py')]),
            launch_arguments=[('config_rviz2', config_rviz2)]
        ),
    ])
