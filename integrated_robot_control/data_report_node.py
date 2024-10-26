import os
import csv
from datetime import datetime
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time



def quaternion_to_yaw(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

class DataReportNode(Node):
    def __init__(self):
        super().__init__('data_report_node')

        self.get_logger().info('Start Data Report Node initialized')
         # 구독자 생성: 오도메트리 메시지
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        #self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        #tf_recorder
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_data = None
        self.imu_data = None
        self.ekf_data = None

        # Variables to store previous values for acceleration calculation
        self.prev_odom = None
        self.prev_imu = None
        self.prev_ekf = None
        self.prev_time = None

        # Position and velocity estimates for IMU
        self.imu_position_x = 0.0
        self.imu_position_y = 0.0
        self.imu_velocity_x = 0.0
       
        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/data_report_node_{timestamp_str}.csv')
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')

        # self.count = 1
        # self.point_index = 0  # 데이터 포인트 인덱스 초기화

        # CSV 파일 경로의 디렉터리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)

        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                
                writer.writerow([
                                    'timestamp', 
                                    'odom_position_x', 'odom_position_y', 'odom_orientation_yaw',
                                    'odom_linear_velocity_x', 'odom_linear_accel_x', 
                                    'odom_angular_velocity_z', 'odom_angular_accel_z',

                                    'imu_position_x', 'imu_position_y', 'imu_orientation_yaw',
                                    'imu_linear_velocity_x', 'imu_linear_acceleration_x',
                                    'imu_angular_velocity_z', 'imu_linear_acceleration_z',

                                    'ekf_position_x', 'ekf_position_y', 'ekf_orientation_yaw',
                                    'ekf_linear_velocity_x', 'ekf_linear_acceleration_x',
                                    'ekf_angular_velocity_z', 'ekf_linear_acceleration_z',
                                    'map_x', 'map_y'
                                ])
        # timer callback based csv logs storing process
        self.timer = self.create_timer(0.5, self.timer_callback)

      
    def odom_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        
        current_time = self.get_clock().now().to_msg()
        
        # Calculate linear and angular acceleration
        if self.prev_odom and self.prev_time:
            delta_time = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) / 1e9
            linear_accel_x = (linear_x - self.prev_odom['linear_x']) / delta_time
            angular_accel_z = (angular_z - self.prev_odom['angular_z']) / delta_time
        else:
            linear_accel_x = 0.0
            angular_accel_z = 0.0

        self.odom_data = {
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y,
            'orientation_yaw': yaw,
            'linear_velocity_x': linear_x,
            'linear_accel_x': linear_accel_x,
            'angular_velocity_z': angular_z,
            'angular_accel_z': angular_accel_z
        }

        # Update previous data
        self.prev_odom = {'linear_x': linear_x, 'angular_z': angular_z}
        self.prev_time = current_time

    def imu_callback(self, msg):
        yaw = quaternion_to_yaw(msg.orientation)
        angular_velocity_z = msg.angular_velocity.z
        linear_accel_x = msg.linear_acceleration.x
        current_time = self.get_clock().now().to_msg()
        
        # Calculate position and velocity estimates
        if self.prev_time:
            delta_time = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) / 1e9
            
            # Update velocity by integrating acceleration in the x direction (since y direction velocity is zero in differential drive)
            self.imu_velocity_x += linear_accel_x * delta_time
            
            # Update positions based on the velocity and yaw
            self.imu_position_x += self.imu_velocity_x * delta_time * math.cos(yaw)
            self.imu_position_y += self.imu_velocity_x * delta_time * math.sin(yaw)

        self.imu_data = {
            'position_x': self.imu_position_x,
            'position_y': self.imu_position_y,
            'orientation_yaw': yaw,
            'linear_velocity_x': self.imu_velocity_x,
            'linear_acceleration_x': linear_accel_x,
            'angular_velocity_z': angular_velocity_z,
            'linear_acceleration_z': msg.linear_acceleration.z
        }

        # Update previous time
        self.prev_time = current_time


    def ekf_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        current_time = self.get_clock().now().to_msg()
        
        # Calculate linear acceleration and angular acceleration similarly to odom
        if self.prev_ekf and self.prev_time:
            delta_time = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) / 1e9
            linear_accel_x = (linear_x - self.prev_ekf['linear_x']) / delta_time
            angular_accel_z = (angular_z - self.prev_ekf['angular_z']) / delta_time
        else:
            linear_accel_x = 0.0
            angular_accel_z = 0.0

        self.ekf_data = {
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y,
            'yaw': yaw,
            'linear_velocity_x': linear_x,
            'linear_acceleration_x': linear_accel_x,
            'angular_velocity_z': angular_z,
            'linear_acceleration_z': angular_accel_z
        }

        # Update previous EKF data
        self.prev_ekf = {'linear_x': linear_x, 'angular_z': angular_z}
        self.prev_time = current_time

    # When timer callback is called, this method runs.
    def timer_callback(self):
        self.save_data_to_csv()

    def save_data_to_csv(self):
        if self.imu_data is None:
            self.get_logger().warn('Imu data is not available yet')
            return

        if self.odom_data is None:
            self.get_logger().warn('Odometry data is not available yet')
            return

        if self.ekf_data is None:
            self.get_logger().warn('Extended Kalman Filter data is not available yet')
            return

        if self.tf_listener is None:
            self.get_logger().warn('Extended Kalman Filter data is not available yet')
            return


        try:
            # Get the latest transform between map and base_link
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            # translation 값을 개별적으로 추출
            translation_x = trans.transform.translation.x
            translation_y = trans.transform.translation.y

            # 각 데이터의 값이 None이 아닌지 확인 후 float으로 변환
            odom_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.odom_data.values()]
            imu_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.imu_data.values()]
            ekf_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.ekf_data.values()]


            with open(self.csv_file_path, mode='a') as file:
                writer = csv.writer(file)
                writer.writerow([
                    # no, x, y,
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                    *["{:.4f}".format(val) for val in odom_values],
                    *["{:.4f}".format(val) for val in imu_values],
                    *["{:.4f}".format(val) for val in ekf_values],
                    f'{translation_x:.4f}', f'{translation_y:.4f}',
                    #lidar_data_str
                ])

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        data_report_node = DataReportNode()
        rclpy.spin(data_report_node)
    except KeyboardInterrupt:
        data_report_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        data_report_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
