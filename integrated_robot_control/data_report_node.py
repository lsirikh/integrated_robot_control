import os
import csv
from datetime import datetime
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

class DataReportNode(Node):
    def __init__(self):
        super().__init__('data_report_node')

        self.get_logger().info('Start Data Report Node initialized')
         # 구독자 생성: 오도메트리 메시지
        self.odom_sub = self.create_subscription(Odometry, '/odom/data', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/fused', self.ekf_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.odom_data = None
        self.imu_data = None
        self.ekf_data = None
        self.lidar_data = None

        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/data_report_node_{timestamp_str}.csv')
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')

        # CSV 파일 경로의 디렉터리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)

        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                # writer.writerow(['timestamp', 'x', 'y', 'theta', 'v', 'w', 'x_corrected', 'y_corrected', 'theta_corrected', 'v_corrected', 'w_corrected', 'lidar_ranges'])
                writer.writerow([
                                    'timestamp', 
                                    'odom_position_x', 'odom_position_y', 'odom_position_z',
                                    'odom_orientation_x', 'odom_orientation_y', 'odom_orientation_z', 'odom_orientation_w',
                                    'odom_linear_x', 'odom_linear_y', 'odom_linear_z',
                                    'odom_angular_x', 'odom_angular_y', 'odom_angular_z',

                                    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
                                    'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
                                    'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',

                                    'ekf_position_x', 'ekf_position_y', 'ekf_position_z',
                                    'ekf_orientation_x', 'ekf_orientation_y', 'ekf_orientation_z', 'ekf_orientation_w',
                                    'ekf_linear_x', 'ekf_linear_y', 'ekf_linear_z',
                                    'ekf_angular_x', 'ekf_angular_y', 'ekf_angular_z',

                                    'lidar_data'
                                ])
        self.timer = self.create_timer(0.125, self.timer_callback)

    def odom_callback(self, msg):
        self.odom_data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]

    def imu_callback(self, msg):
        self.imu_data = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

    def ekf_callback(self, msg):
        self.ekf_data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]

    # # 콜백 함수: LiDAR 메시지 처리
    # def lidar_callback(self, msg):
    #     angle_min = msg.angle_min
    #     angle_increment = msg.angle_increment
    #     self.lidar_data = ','.join([f'[{angle_min + i * angle_increment:.2f},{dist:.2f}]' if np.isfinite(dist) else '[{angle_min + i * angle_increment:.2f},inf]' for i, dist in enumerate(msg.ranges)])


    def lidar_callback(self, msg):
        # # 모든 속성과 값을 출력
        # for attribute in dir(msg):
        #     if not attribute.startswith('_'):  # 내부 속성은 제외
        #         attribute_value = getattr(msg, attribute)
        #         self.get_logger().info(f'{attribute}: {attribute_value}')
        #self.get_logger().info(f'lidar_callback in data_report_node')
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        self.lidar_data = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            angle_deg = angle * 180 / math.pi  # 라디안에서 각도로 변환
            self.lidar_data.append((angle_deg, distance))
        

    def timer_callback(self):
        if self.lidar_data is None:
            self.get_logger().info('Lidar data is not available yet')
            return
        
        if self.imu_data is None:
            self.get_logger().info('Imu data is not available yet')
            return

        if self.odom_data is None:
            self.get_logger().info('Odometry data is not available yet')
            return

        if self.ekf_data is None:
            self.get_logger().info('Extended Kalman Filter data is not available yet')
            return

        lidar_data_str = ','.join([f'[{angle:.2f},{distance:.2f}]' for angle, distance in self.lidar_data])

        # with open(self.csv_file_path, mode='a') as file:
        #     writer = csv.writer(file)
        #     writer.writerow([
        #         datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        #         # f'[{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}', f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}]',
        #         f'{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}',
        #         f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}', f'{self.odom_data[5]:.4f}', f'{self.odom_data[6]:.4f}',
        #         f'{self.odom_data[7]:.4f}', f'{self.odom_data[8]:.4f}', f'{self.odom_data[9]:.4f}',
        #         f'{self.odom_data[10]:.4f}', f'{self.odom_data[11]:.4f}', f'{self.odom_data[12]:.4f}',

        #          f'{self.ekf_data[0]:.4f}', f'{self.ekf_data[1]:.4f}', f'{self.ekf_data[2]:.4f}',
        #         f'{self.ekf_data[3]:.4f}', f'{self.ekf_data[4]:.4f}', f'{self.ekf_data[5]:.4f}', f'{self.ekf_data[6]:.4f}',
        #         f'{self.ekf_data[7]:.4f}', f'{self.ekf_data[8]:.4f}', f'{self.ekf_data[9]:.4f}',
        #         f'{self.ekf_data[10]:.4f}', f'{self.ekf_data[11]:.4f}', f'{self.ekf_data[12]:.4f}'
        #         # , f'[{lidar_data_str}]'
        #     ])
        with open(self.csv_file_path, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                *[f'{val:.4f}' for val in self.odom_data],
                *[f'{val:.4f}' for val in self.imu_data],
                *[f'{val:.4f}' for val in self.ekf_data],
                lidar_data_str
            ])

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
