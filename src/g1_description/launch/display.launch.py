import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('g1_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'g1_23dof.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'g1.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
    	# Place le bassin (pelvis) à 0.765 m au-dessus du sol (world)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_pelvis',
            arguments=['0', '0', '0.765', '0', '0', '0', 'world', 'pelvis']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
