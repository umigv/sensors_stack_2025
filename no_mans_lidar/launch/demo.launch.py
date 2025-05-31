from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
            package='pointcloud_to_grid',
            executable='grid_trajectory',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("topic")},
                {'position_x': -20.0},
                {'position_y': 0.0},
                {'verbose1': False},
                {'verbose2': False},
                {'cell_size': 0.05}, #might have to change this to 0.1
                {'length_x': 3.8},
                {'length_y': 7.75},
                {'mapi_topic_name': 'intensity_grid'},
                {'maph_topic_name': 'height_grid'},
                {'search_length': 4.0},
                {'search_range_deg': 80.0},
                {'search_resolution_deg': 0.5}, 
                {'search_start_mid_deg': -180.0}, 
            ]
        )

    ])
