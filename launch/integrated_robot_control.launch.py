import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    pico_port = LaunchConfiguration('pico_port', default='/dev/ttyACM0')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/rplidar')
    rplidar_baudrate = LaunchConfiguration('rplidar_baudrate', default=256000)
    imu_port = LaunchConfiguration('imu_port', default='/dev/imu_usb')
    imu_baudrate = LaunchConfiguration('imu_baudrate', default=115200)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pico_port',
            default_value=pico_port,
            description='Serial port for communication with microcontroller'
        ),
        DeclareLaunchArgument(
            'rplidar_port',
            default_value=rplidar_port,
            description='Port for rplidar sensor'
        ),
        DeclareLaunchArgument(
            'rplidar_baudrate',
            default_value=rplidar_baudrate,
            description='Baud rate for rplidar sensor'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value=imu_port,
            description='Serial port for IMU'
        ),
        DeclareLaunchArgument(
            'imu_baudrate',
            default_value=imu_baudrate,
            description='Baud rate for IMU'
        ),
       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/lidar.launch.py']
            ),
            launch_arguments={'serial_port': rplidar_port, 'serial_baudrate': rplidar_baudrate}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/imu.launch.py']
            ),
         
            launch_arguments={'serial_port': imu_port, 'serial_baudrate': imu_baudrate}.items()
        ),
        Node(
            package='integrated_robot_control', executable='control_node',
            name='control_node', 
            output='screen',
            parameters=[{'pico_port': pico_port}]
        ),
        # static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.12', '0', '0.28', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        # static transform for imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.12', '0', '0.18', '0', '0', '0', 'base_link', 'imu'],
            output='screen'
        ),
        # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )
        
    ])
