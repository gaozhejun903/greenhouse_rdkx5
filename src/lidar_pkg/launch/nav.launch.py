import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_pkg')

    # Paths
    urdf_path = os.path.join(pkg_dir, 'urdf', 'lidar.urdf')
    lidar_params = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')
    base_params = os.path.join(pkg_dir, 'config', 'base_params.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    pest_params = os.path.join(pkg_dir, 'config', 'pest_monitor_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam.rviz')

    # Launch arguments
    use_sim_time = False
    autostart = True
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_keyboard = LaunchConfiguration('use_keyboard', default='false')
    map_file = LaunchConfiguration('map_file',
        default=os.path.expanduser('~/rdkx5_ws/maps/my_map.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('use_keyboard', default_value='false',
                              description='Launch keyboard control for manual override'),
        DeclareLaunchArgument('map_file',
                              default_value=os.path.expanduser('~/rdkx5_ws/maps/my_map.yaml'),
                              description='Full path to map yaml file'),

        # ========== 1. Robot State Publisher (URDF) ==========
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_path]),
                'use_sim_time': use_sim_time,
            }],
        ),

        # ========== 2. LiDAR Driver ==========
        Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[lidar_params],
        ),

        # ========== 3. Base Driver (STM32 chassis) ==========
        Node(
            package='lidar_pkg',
            executable='base_driver',
            name='base_driver',
            output='screen',
            parameters=[base_params],
        ),

        # ========== 3.5 Pest Monitor (病虫害定位监控) ==========
        Node(
            package='lidar_pkg',
            executable='pest_monitor',
            name='pest_monitor',
            output='screen',
            parameters=[pest_params],
        ),

        # ========== 4. Map Server ==========
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params,
                {'yaml_filename': map_file}],
        ),

        # ========== 5. AMCL (Localization) ==========
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),

        # ========== 6. Planner Server ==========
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # ========== 7. Controller Server ==========
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # ========== 8. Behavior Server ==========
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # ========== 9. BT Navigator ==========
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        # ========== 10. Lifecycle Manager ==========
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),

        # ========== 11. Keyboard Control (可选, 手动接管) ==========
        Node(
            package='lidar_pkg',
            executable='keyboard_control',
            name='keyboard_control',
            output='screen',
            condition=IfCondition(use_keyboard),
        ),

        # ========== 12. RViz2 ==========
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
