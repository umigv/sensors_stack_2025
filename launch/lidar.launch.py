import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
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
                'output_qos_reliability': 'reliable', # set QoS to reliable
            }],
            remappings=[
                ('/velodyne/points', '/velodyne_points'),  # PointCloud2 topic from Velodyne driver
                ('/scan', '/velodyne/scan')  # LaserScan topic
            ],
        ),

        # PointCloud2 to OccupancyGrid conversion
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # or 'async_slam_toolbox_node' for online async
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'slam_params_file': '/path/to/your/slam_config.yaml',  # Optional, see below
                'use_sim_time': False,
                'resolution': 0.05,
                'max_laser_range': 35.0,
                'minimum_time_interval': 0.5,  # reduce to improve update rate
            }],
            remappings=[
                ('scan', '/velodyne/scan'),
                ('/map', '/map')
            ],
        ),
    ])
