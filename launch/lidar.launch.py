import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
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
    ])
