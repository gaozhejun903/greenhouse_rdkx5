import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_pkg')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'lidar.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'lidar.rviz')

    return LaunchDescription([
        # 1. static TF odom → base_link (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # 2. robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_path]),
            }],
        ),

        # 3. lidar driver
        Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'lidar_params.yaml')],
        ),

        # 4. rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
