import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for the GNSS and ZED Fusion Launch File
        DeclareLaunchArgument(
            'gps_device', 
            default_value='/dev/ttyACM0',  # Default GPS device path
            description='GPS device path'
        ),

        DeclareLaunchArgument(
            'gnss_fusion_enabled', 
            default_value='true',  # Default to enabling GNSS fusion
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
            }],
            remappings=[
                ('/zed/odom', '/odom'),  # Remap ZED odometry topic
                ('/zed/gnss/fix', '/gps/fix')  # Map GNSS data to the appropriate topic
            ],
        ),
        
        # GPS node (ublox GPS with configurable device path)
        Node(
            package='ublox_gps',
            executable='ublox_driver',
            name='ublox_gps',
            namespace='',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('gps_device'),  # Use the configurable GPS device path
                'frame_id': 'gps',  # Frame of reference for GPS
            }],
            remappings=[
                ('/gps/fix', '/gps/fix')  # Make sure the NavSatFix message is available on this topic
            ],
        ),
        
        # Service for converting Lat/Lon to robot map coordinates
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
            condition=launch.conditions.IfCondition(LaunchConfiguration('gnss_fusion_enabled')),
            msg="GNSS Fusion is disabled. Only visual odometry will be used for localization."
        ),
    ])
