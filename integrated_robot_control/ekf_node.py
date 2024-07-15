import os
import csv
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion  # Quaternion 클래스 임포트

# 함수: 쿼터니언을 오일러 각도로 변환
def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)  # 첫 번째 오일러 각도 (롤)
    t1 = +1.0 - 2.0 * (x * x + y * y)  # 두 번째 오일러 각도 (롤)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)  # 세 번째 오일러 각도 (피치)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arctan2(t2, np.sqrt(1.0 - t2 * t2))
    t3 = +2.0 * (w * z + x * y)  # 네 번째 오일러 각도 (요)
    t4 = +1.0 - 2.0 * (y * y + z * z)  # 다섯 번째 오일러 각도 (요)
    yaw = np.arctan2(t3, t4)
    return roll, pitch, yaw

# 함수: 오일러 각도를 쿼터니언으로 변환
def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

# 클래스: EKF 노드
class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # 노드 초기화 로그
        self.get_logger().info('Start EKF Node initialized')

        # 구독자 생성: 오도메트리 메시지
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # 구독자 생성: IMU 메시지
        self.imu_sub = self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)

        # 퍼블리셔 생성: 필터링된 오도메트리 메시지
        self.ekf_odom_pub = self.create_publisher(Odometry, 'odom/filtered', 10)

        # 상태 벡터와 공분산 행렬 초기화
        self.x = np.zeros(5)
        self.P = np.eye(5)

        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/ekf_node_data_{timestamp_str}.csv')
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')
        
        # CSV 파일 경로의 디렉터리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)

        # CSV 파일이 존재하지 않으면 새로 생성
        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['timestamp', 'x', 'y', 'theta', 'v', 'w', 'x_corrected', 'y_corrected', 'theta_corrected', 'v_corrected', 'w_corrected'])

        self.get_logger().info("Finish EKF Node initialized")

    # 콜백 함수: 오도메트리 메시지 처리
    def odom_callback(self, msg):
        self.get_logger().info("Odometry message received")

        dt = 0.04  # 시간 간격
        x, y = float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)
        _, _, theta = quaternion_to_euler(msg.pose.pose.orientation)
        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)

        # 상태 전이 행렬
        F = np.array([
            [1, 0, -v * dt * np.sin(theta), dt * np.cos(theta), 0],
            [0, 1, v * dt * np.cos(theta), dt * np.sin(theta), 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + np.eye(5) * 0.01

        # 관측 벡터와 관측 행렬
        z = np.array([x, y, theta, v, w])
        H = np.eye(5)
        R = np.eye(5) * 0.1
        ys = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ ys
        self.P = (np.eye(5) - K @ H) @ self.P

        # 수정된 오도메트리 퍼블리시
        self.publish_corrected_odometry()

        # CSV 파일에 데이터 기록
        with open(self.csv_file_path, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],  # 소수점 3자리까지 표기
                f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}', f'{v:.4f}', f'{w:.4f}',  # 원본 데이터
                f'{float(self.x[0]):.4f}', f'{float(self.x[1]):.4f}', f'{float(self.x[2]):.4f}',  # 보정된 데이터
                f'{float(self.x[3]):.4f}', f'{float(self.x[4]):.4f}'
            ])

    # 콜백 함수: IMU 메시지 처리
    def imu_callback(self, msg):
        self.get_logger().info("IMU message received")
        _, _, theta = quaternion_to_euler(msg.orientation)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        self.x[2] = theta
        self.x[3] += ax * 0.04
        self.x[4] += ay * 0.04

    # 함수: 수정된 오도메트리 퍼블리시
    def publish_corrected_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(self.x[0])
        odom_msg.pose.pose.position.y = float(self.x[1])
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=float(self.x[0]),  # float으로 변환
            y=float(self.x[1]),  # float으로 변환
            z=float(self.x[2]),  # float으로 변환
            w=1.0
        )
        odom_msg.twist.twist.linear.x = float(self.x[3])  # float으로 변환
        odom_msg.twist.twist.angular.z = float(self.x[4])  # float으로 변환
        self.ekf_odom_pub.publish(odom_msg)

# 메인 함수: 노드 실행
def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
