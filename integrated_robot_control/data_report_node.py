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
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, 'odom/filtered', self.ekf_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        self.odom_data = None
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
                                    # 'x', 'y', 'theta', 'v', 'w',
                                    'position_x', 'position_y', 'position_z',
                                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                                    'linear_x', 'linear_y', 'linear_z',
                                    'angular_x', 'angular_y', 'angular_z',

                                    'correct_position_x', 'correct_position_y', 'correct_position_z',
                                    'correct_orientation_x', 'correct_orientation_y', 'correct_orientation_z', 'correct_orientation_w',
                                    'correct_linear_x', 'correct_linear_y', 'correct_linear_z',
                                    'correct_angular_x', 'correct_angular_y', 'correct_angular_z',
                                ])
        self.timer = self.create_timer(0.125, self.timer_callback)

    def odom_callback(self, msg):
        #self.get_logger().info(f'odom_callback in data_report_node')

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
        if self.ekf_data is None or self.lidar_data is None:
            self.get_logger().info('Lidar data or Imu data is not available yet')
            return
        
        if self.odom_data is None:
            self.get_logger().info('Odometry data is not available yet')
            return

        lidar_data_str = ','.join([f'[{angle:.2f},{distance:.2f}]' for angle, distance in self.lidar_data])

        with open(self.csv_file_path, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                # f'[{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}', f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}]',
                f'{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}',
                f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}', f'{self.odom_data[5]:.4f}', f'{self.odom_data[6]:.4f}',
                f'{self.odom_data[7]:.4f}', f'{self.odom_data[8]:.4f}', f'{self.odom_data[9]:.4f}',
                f'{self.odom_data[10]:.4f}', f'{self.odom_data[11]:.4f}', f'{self.odom_data[12]:.4f}',

                 f'{self.ekf_data[0]:.4f}', f'{self.ekf_data[1]:.4f}', f'{self.ekf_data[2]:.4f}',
                f'{self.ekf_data[3]:.4f}', f'{self.ekf_data[4]:.4f}', f'{self.ekf_data[5]:.4f}', f'{self.ekf_data[6]:.4f}',
                f'{self.ekf_data[7]:.4f}', f'{self.ekf_data[8]:.4f}', f'{self.ekf_data[9]:.4f}',
                f'{self.ekf_data[10]:.4f}', f'{self.ekf_data[11]:.4f}', f'{self.ekf_data[12]:.4f}'
                # , f'[{lidar_data_str}]'
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
