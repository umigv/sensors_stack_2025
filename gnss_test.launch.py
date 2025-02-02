# import relevant libraries
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        # set gnss_fusion to enabled
        DeclareLaunchArgument(
            'gnss_fusion_enabled', 
            default_value='true', 
            description='Enable GNSS Fusion'
        ),
        
        # GNSS fusion node (using ZED ROS 2 Wrapper with GNSS support)
        Node(
            package='zed_ros2_wrapper',
            executable='zed_node',
            name='zed',
            namespace='',
            output='screen',
            parameters=[{
                'gnss_fusion.gnss_fusion_enabled': LaunchConfiguration('gnss_fusion_enabled'),
                # Add other parameters you need for ZED camera configuration
            }],
            remappings=[
                ('/zed/odom', '/odom'),  # Remap ZED odometry topic
                ('/zed/gnss/fix', '/gps/fix')  # Map GNSS data to the appropriate topic
            ],
        ),
        
        # GPS node (ublox GPS connected to /dev/ttyACM0)
        Node(
            package='ublox_gps',
            executable='ublox_driver',
            name='ublox_gps',
            namespace='',
            output='screen',
            parameters=[{
                'device': '/dev/ttyACM0',  # GPS device
                'frame_id': 'gps',  # Frame of reference for GPS
            }],
            remappings=[
                ('/gps/fix', '/gps/fix')  # Make sure the NavSatFix message is available on this topic
            ],
        ),
        
        # Service for converting Lat/Lon to robot map coordinates (optional)
        Node(
            package='zed_ros2_wrapper',
            executable='gnss_to_map_node',
            name='gnss_to_map',
            namespace='',
            output='screen',
            parameters=[{
                'gnss_fusion.gnss_fusion_enabled': LaunchConfiguration('gnss_fusion_enabled')
            }],
            remappings=[
                ('/gnss_to_map/latlon', '/gnss_latlon'),  # Lat/Lon to map conversion
            ],
        ),
        
        # Log the GNSS fusion status
        LogInfo(
            condition=launch.conditions.IfCondition(LaunchConfiguration('gnss_fusion_enabled')),
            msg="GNSS Fusion is enabled. Lat/Lon will be fused with visual odometry."
        ),
        
        # Log if GNSS Fusion is not enabled
        LogInfo(
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gnss_fusion_enabled').perform() == 'false'),
            msg="GNSS Fusion is disabled. Only visual odometry will be used for localization."
        ),
    ])
