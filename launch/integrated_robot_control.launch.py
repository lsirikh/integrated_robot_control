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
    imu_baudrate = LaunchConfiguration('imu_baudrate', default=9600)
    joy_config = LaunchConfiguration('joy_config', default='ps4')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    config_filepath = LaunchConfiguration('config_filepath', default=[
        LaunchConfiguration('config_dir'), '/', joy_config, '.config.yaml'
    ])

    # EKF 설정 파일 경로
    #ekf_config = os.path.join(get_package_share_directory('integrated_robot_control'), 'config', 'ekf.yaml')
    config_dir = os.path.join(get_package_share_directory('integrated_robot_control'), 'config')

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
        DeclareLaunchArgument(
            'joy_config',
            default_value=joy_config,
            description='Configuration for joystick'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value=joy_dev,
            description='Joystick device'
        ),
        DeclareLaunchArgument(
            'config_dir',
            default_value=config_dir,
            description='Configuration directory'
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
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            output='screen'
        ),
        Node(
            package='integrated_robot_control', executable='control_node',
            name='control_node', 
            output='screen',
            parameters=[{'pico_port': pico_port}]
        ),
        Node(
            package='integrated_robot_control', executable='data_report_node',
            name='data_report_node', 
            output='screen',
        ),
        Node(
            package='ekf_robot_control',
            executable='main_node',
            name='ekf_main_node',
            output='screen'
        ),
        Node(
            package='ekf_robot_control',
            executable='set_measurement_covariance_node',
            name='ekf_set_measurement_covariance_node',
            output='screen'
        ),
        # 새로 추가된 robot_control_profile 노드
        # Node(
        #     package='integrated_robot_control', executable='robot_control_profile',
        #     name='robot_control_profile_node',
        #     output='screen'
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.064', '0', '0.120', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )
        
    ])
