import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ros2_slam_auto_navigation'
    urdf_package_name = 'ali_rob'

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')

    # RViz + Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(urdf_package_name),
                'launch',
                'rviz_.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Twist Mux
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # RPLIDAR
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m12_launch.py'
            )
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': '256000',
            'frame_id': 'lidar'
        }.items()
    )

    # Motor Sürücü
    differential_drive_node = Node(
        package='zlac8015d_serial',
        executable='zlac_run',
        name='differential_drive',
        output='screen',
        respawn=True,
        respawn_delay=1.0
    )

    # Laser Filter
    laser_filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/ali/kiosk_toolbox/src/laser_filters/examples/angular_filter_example.launch.py'
        )
    )

    # Cartographer (SLAM)
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('tortoisebot_slam'),
                'launch',
                'cartographer.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'exploration': exploration
        }.items()
    )

    # BNO055 IMU
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('bno055'),
                'launch',
                'bno055.launch.py'
            )
        ])
    )

    # EKF (robot_localization)
    ekf_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('exploration',  default_value='true'),
        rsp,
        twist_mux,
        rplidar_node,
        laser_filters_launch,
        differential_drive_node,
        bno055_launch,
        ekf_node,
        cartographer_launch
    ])

