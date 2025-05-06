import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition   # <-- önemli

def generate_launch_description():
    prefix_address  = get_package_share_directory('tortoisebot_slam')
    config_directory = os.path.join(prefix_address, 'config')
    slam_config      = 'slam.lua'

    # Launch argümanları
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    resolution   = LaunchConfiguration('resolution')
    pub_period   = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('exploration',  default_value='true'),

        DeclareLaunchArgument('resolution',           default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec',   default_value='1.0'),

        DeclareLaunchArgument('configuration_directory',
                              default_value=config_directory),
        DeclareLaunchArgument('slam_configuration_basename',
                              default_value=slam_config),
        DeclareLaunchArgument('localization_configuration_basename',
                              default_value=slam_config),

        # ► SLAM (exploration == "true")
        Node(
            package='cartographer_ros',
            condition=IfCondition(exploration),
            executable='cartographer_node',
            name='as21_cartographer_node',
            arguments=[
                '-configuration_directory', config_directory,
                '-configuration_basename',  slam_config
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ► Yalnızca hazır harita için (localization modu)
        Node(
            package='cartographer_ros',
            condition=UnlessCondition(exploration),
            executable='cartographer_node',
            name='as21_cartographer_node',
            arguments=[
                '-configuration_directory', config_directory,
                '-configuration_basename',  slam_config
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ► Occupancy grid SLAM çıktısı (exploration == "true")
        Node(
            package='cartographer_ros',
            condition=IfCondition(exploration),
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=[
                '-resolution',         resolution,
                '-publish_period_sec', pub_period
            ]
        ),
    ])

