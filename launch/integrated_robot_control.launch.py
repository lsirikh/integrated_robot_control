import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pico_port = LaunchConfiguration('pico_port', default='/dev/ttyACM0')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/rplidar')
    rplidar_baudrate = LaunchConfiguration('rplidar_baudrate', default=256000)
    imu_port = LaunchConfiguration('imu_port', default='/dev/imu_usb')
    imu_baudrate = LaunchConfiguration('imu_baudrate', default=115200)

    # EKF 설정 파일 경로
    config_dir = os.path.join(get_package_share_directory('integrated_robot_control'), 'config')
    ekf_config = os.path.join(config_dir, 'ekf.yaml')
    amcl_config = os.path.join(config_dir, 'amcl_params.yaml')

    # 맵 파일 경로
    map_file = os.path.join('/home/ubuntu/robot_ws', 'map_1729274679.yaml')

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
            package='integrated_robot_control', executable='sensor_sync_node',
            name='sensor_sync_node', 
            output='screen',
        ),
        # To change this node to service node - robot_control_profile Node
        # Node(
        #     package='integrated_robot_control', executable='robot_control_profile',
        #     name='robot_control_profile_node',
        #     output='screen'
        # ),

        # # ekf_robot_control pkg node
        # Uncertainty was too high to apply this package
        # Node(
        #     package='ekf_robot_control',
        #     executable='main_node',
        #     name='ekf_main_node',
        #     output='screen'
        # ),
        # Node(
        #     package='ekf_robot_control',
        #     executable='set_measurement_covariance_node',
        #     name='ekf_set_measurement_covariance_node',
        #     output='screen'
        # ),

     

        # robot_localization EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
      
        # nav2_amcl node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')],
        ),

        # initialpose 퍼블리싱 노드 추가
        Node(
            package='integrated_robot_control',
            executable='initialpose_node',
            name='initialpose_node',
            output='screen',
        ),

        # # imu_recorder_node 퍼블리싱 노드 추가
        # Node(
        #     package='integrated_robot_control',
        #     executable='imu_recorder_node',
        #     name='imu_recorder_node',
        #     output='screen',
        # ),
        
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
