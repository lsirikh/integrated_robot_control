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
    serial_baudrate = LaunchConfiguration('serial_baudrate', default=9600)
    frame_id = LaunchConfiguration('frame_id', default='imu_link')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Port for imu sensor'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected imu'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of imu'),

        Node(
            name='imu_node',
            package='cmp10a_imu',
            executable='cmp10a_imu_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                }],
        )
    ])