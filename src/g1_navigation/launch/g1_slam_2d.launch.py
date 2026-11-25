import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    livox_dir = get_package_share_directory('livox_ros_driver2')
    fast_lio_dir = get_package_share_directory('fast_lio')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    g1_nav_dir = get_package_share_directory('g1_navigation')

    slam_params_file = os.path.join(g1_nav_dir, 'config', 'slam_2d.yaml')

    return LaunchDescription([
        # 1. Driver LiDAR Livox Mid-360
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360_launch.py')
            )
        ),

        # 2. SLAM 3D FAST-LIO
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
            ),
            launch_arguments={'config_file': 'mid360.yaml'}.items()
        ),

        # 3. Redressement TF2 (inclinaison tête G1 -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '-0.45',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', 'body', '--child-frame-id', 'base_link']
        ),

        # 4. Projection 3D -> LaserScan 2D (/scan)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': 0.15,
                'max_height': 1.50,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.30,
                'range_max': 15.0,
                'use_inf': True
            }]
        ),

        # 5. Cartographie 2D SLAM Toolbox (/map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'slam_params_file': slam_params_file}.items()
        )
    ])