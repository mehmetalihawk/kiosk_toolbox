#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 1) Nav2 bringup script dizini
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    # 2) Tortoisebot_navigation paketi kökü
    tortoisebot_nav_dir = get_package_share_directory('tortoisebot_navigation')

    # 3) <== PARAMETRE YML DOSYASI  (senin verdiğin tam path)  ==>
    default_params_path = LaunchConfiguration(
        'params_file',
        default='/home/ali/sllidar/src/tortoisebot/tortoisebot_navigation/config/nav2_params_robot.yaml')

    # 4) Harita dosyası (gerekirse değiştir)
    default_map_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('tortoisebot_bringup'),
            'maps',
            'room2.yaml'))

    # 5) SimTime argümanı
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # --- Nav2’nin ana launch’ını içe al ---
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map':        default_map_path,
            'use_sim_time': use_sim_time,
            'params_file':  default_params_path
        }.items(),
    )

    # --- LaunchDescription çıktısı ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to the nav2 parameters file to use'),

        navigation_launch,
    ])

