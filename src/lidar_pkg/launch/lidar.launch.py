import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_pkg')
    params_file = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),

        Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
