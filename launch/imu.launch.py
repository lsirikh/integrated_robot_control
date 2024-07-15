from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_port = LaunchConfiguration('serial_port', default='/dev/imu_usb')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Port for imu sensor'
        ),
        Node(
            name='imu_composition',
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            output='screen',
            parameters=[{
                'port': '/dev/imu_usb',
                'serial_baudrate': 9600,
                'frame_id': 'imu',
                'inverted': False,
                'angle_compensate': True,
                }],
        )
    ])