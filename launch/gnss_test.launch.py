''' 
    Using the ZED ROS 2 Wrapper that can subscribe to a NavSatFix topic and 
    fuse GNSS data information with Positional Tracking information 
    to obtain a precise robot localization referred to Earth coordinates. 
    To enable GNSS fusion set the parameter gnss_fusion.gnss_fusion_enabled to true. 
    The services toLL and fromLL can be used to convert Latitude/Longitude coordinates to robot map coordinates. 
 
Nodes:
    ublox GPS from /dev/ttyACM0 (needs to publish to sensor_msgs/NavSatFix)
    zed visual odom \odom 

When running launch script, specify the GPS device path via the command line (default path is set to: '/dev/ttyACM0')

ros2 launch package gnss_test.launch.py gps_device:=<DEVICE_PATH> 
''' 

# import relevant libraries
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import WaitForMessage

def generate_launch_description():
    return LaunchDescription([
        
        # Wait for the GPS data to be available
        WaitForMessage(
            topic='/gps/fix',
            msg_type='sensor_msgs/NavSatFix',
            timeout=10.0,  # Timeout in seconds
        ),
        
        # Make GPS device path configurable
        DeclareLaunchArgument(
            'gps_device', 
            default_value='/dev/ttyACM0',  # Default value if not specified
            description='GPS device path'
        ),

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
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gnss_fusion_enabled').perform() == 'false'),
            msg="GNSS Fusion is disabled. Only visual odometry will be used for localization."
        ),
    ])
