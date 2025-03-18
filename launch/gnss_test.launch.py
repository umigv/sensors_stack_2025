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

        # Velodyne Driver node: use velodyne package to get laserscan
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver',
            namespace='',
            output='screen',
            parameters=[{
                'frame_id': 'velodyne',
                'device_ip': '192.168.1.202',
                'scan_angle_min': -2.0,
                'scan_angle_max': 2.0,
                'scan_frequency': 10.0,
            }],
            remappings=[
                ('/velodyne_points', '/velodyne/points')  # default topic for PointCloud2 data
            ],
        ),
        
        # Velodyne PointCloud to LaserScan conversion
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace='',
            output='screen',
            parameters=[{
                'frame_id': 'velodyne',  # The frame of reference
                'scan_height': 1.5,       # Height of the laser scan, adjust as needed
                'max_range': 35.0,        # Maximum range of the LaserScan
                'min_range': 1.5,         # Minimum range of the LaserScan
                'angle_min': -3.14,       # Start angle for the scan
                'angle_max': 3.14,        # End angle for the scan
            }],
            remappings=[
                ('/velodyne/points', '/velodyne_points'),  # PointCloud2 topic from Velodyne driver
                ('/scan', '/velodyne/scan')  # LaserScan topic
            ],
        ),

        # PointCloud2 to OccupancyGrid conversion
        Node(
            package='pointcloud_to_occupancy_grid',
            executable='pointcloud_to_occupancy_grid_node',
            name='pointcloud_to_occupancy_grid',
            namespace='',
            output='screen',
            parameters=[{
                'frame_id': 'velodyne',
                'resolution': 0.05,  # Resolution of the occupancy grid in meters
                'size_x': 76,  # Size of the grid in meters
                'size_y': 155,  # Size of the grid in meters
                'max_range': 35.0,  # Maximum range
                'min_range': 1.5,  # Minimum range 
            }],
            remappings=[
                ('/velodyne_points', '/velodyne/points'),  # Subscribe to the Velodyne PointCloud2 data
                ('/occupancy_grid', '/map')  # Publish OccupancyGrid to the /map topic
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

        # Launch the slam_toolbox node for SLAM mapping
        Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[{
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': None,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',  # Assuming the LiDAR scan data is being published to '/scan'
            'use_map_saver': True,
            'mode': 'mapping',
            'map_resolution': 0.05,  # 5cm per grid cell
            'map_size': [3.8, 7.75],  # 3.8m x 7.75m (760.05m x 1550.05m in the map grid)
            'max_laser_range': 12.0,  # Match ZED 2i’s reliable range
            'map_update_interval': 0.066,  # Match ZED 2i’s 15 Hz rate
            'minimum_time_interval': 0.5,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,  # if 0 never publishes odometry
            'transform_timeout': 0.2,
            'tf_buffer_duration': 10.0,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 10.0,
            'use_scan_matching': True,
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,
            'loop_match_minimum_chain_size': 10,
            'debug_logging': False,
        }],
        remappings=[
            ('/scan', '/scan')  # LiDAR scan data is typically published to the '/scan' topic
        ],
    ),
])
