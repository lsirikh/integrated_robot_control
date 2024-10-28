import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Imu
from datetime import datetime
import csv
import time

class ImuDataRecorder(Node):
    def __init__(self):
        super().__init__('imu_data_recorder')
        
        # Subscriber to /imu/data_raw topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        # CSV 파일 설정
        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/imu_data_{timestamp_str}.csv')
        self.create_csv_file()
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')
        
        # 50Hz로 데이터를 기록하도록 타이머 설정 (1/50 초 간격)
        self.timer_period = 1.0 / 50.0
        self.timer = self.create_timer(self.timer_period, self.save_data)
        
        self.imu_data = None

    def create_csv_file(self):
        # CSV 파일을 만들고 라벨을 기록
        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)  # 디렉토리 생성
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'timestamp',
                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
                ])
            self.get_logger().info(f'CSV file {self.csv_file_path} created with labels.')

    def imu_callback(self, msg):
        # 최신 IMU 데이터를 저장
        self.imu_data = {
            'timestamp': time.time(),
            'orientation_x': msg.orientation.x,
            'orientation_y': msg.orientation.y,
            'orientation_z': msg.orientation.z,
            'orientation_w': msg.orientation.w,
            'angular_velocity_x': msg.angular_velocity.x,
            'angular_velocity_y': msg.angular_velocity.y,
            'angular_velocity_z': msg.angular_velocity.z,
            'linear_acceleration_x': msg.linear_acceleration.x,
            'linear_acceleration_y': msg.linear_acceleration.y,
            'linear_acceleration_z': msg.linear_acceleration.z
        }

    def save_data(self):
        # IMU 데이터가 존재할 때만 CSV에 저장
        if self.imu_data:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    self.imu_data['timestamp'],
                    self.imu_data['orientation_x'], self.imu_data['orientation_y'], self.imu_data['orientation_z'], self.imu_data['orientation_w'],
                    self.imu_data['angular_velocity_x'], self.imu_data['angular_velocity_y'], self.imu_data['angular_velocity_z'],
                    self.imu_data['linear_acceleration_x'], self.imu_data['linear_acceleration_y'], self.imu_data['linear_acceleration_z']
                ])


def main(args=None):
    rclpy.init(args=args)
    imu_recorder_node = ImuDataRecorder()
    rclpy.spin(imu_recorder_node)
    imu_recorder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()