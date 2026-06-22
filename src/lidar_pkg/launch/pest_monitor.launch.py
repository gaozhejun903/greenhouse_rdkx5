import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_pkg')
    pest_params = os.path.join(pkg_dir, 'config', 'pest_monitor_params.yaml')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2 with pest markers'),

        # ========== Pest Monitor Node ==========
        Node(
            package='lidar_pkg',
            executable='pest_monitor',
            name='pest_monitor',
            output='screen',
            parameters=[pest_params],
        ),

        # ========== RViz2 (optional) ==========
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'slam.rviz')],
            output='screen',
        ),
    ])
