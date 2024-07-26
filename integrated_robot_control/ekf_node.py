import os
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import transforms3d.euler

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    q = transforms3d.euler.euler2quat(roll, pitch, yaw)
    quaternion = Quaternion()
    quaternion.x = q[1]
    quaternion.y = q[2]
    quaternion.z = q[3]
    quaternion.w = q[0]
    return quaternion

def euler_from_quaternion(quaternion):
    euler = transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    return euler

def radians_to_degrees(radians):
    return radians * (180.0 / np.pi)

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        self.get_logger().info('EKF 노드 초기화 시작')

        # 파라미터를 YAML에서 로드
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 65.0),
                ('sensor_timeout', 0.1),
                ('two_d_mode', True),
                ('publish_tf', True),
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('base_link_frame', 'base_link'),
                ('world_frame', 'odom'),
                ('odom0', '/odom'),
                ('imu0', '/imu/data_raw')
            ]
        )

        # 오도메트리와 IMU 데이터를 구독
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom0').get_parameter_value().string_value, self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.get_parameter('imu0').get_parameter_value().string_value, self.imu_callback, 10)

        # 필터링된 오도메트리 데이터를 퍼블리시
        self.ekf_odom_pub = self.create_publisher(Odometry, 'odom/filtered', 10)

        # 상태 벡터 초기화: [x, y, roll, pitch, yaw]
        self.mu = np.zeros(5)
        # 상태 공분산 행렬 초기화
        self.Sigma = np.eye(5)

        # 프로세스 노이즈 공분산 행렬
        self.R = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])**2
        # 오도메트리 관측 노이즈 공분산 행렬
        self.Q_odom = np.diag([0.05, 0.05, 0.01])**2
        # IMU 관측 노이즈 공분산 행렬
        self.Q_imu = np.diag([0.01, 0.01, 0.01])**2

        self.initialized = False

        self.get_logger().info('EKF 노드 초기화 완료')

    # 예측 단계 (Prediction Step)
    def predict(self, u, dt):
        # 상태 벡터의 각 요소 추출
        theta = self.mu[4]

        # 상태 전이 행렬 G 계산
        G = np.array([
            [1, 0, 0, 0, -u[0] * np.sin(theta) * dt],
            [0, 1, 0, 0, u[0] * np.cos(theta) * dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # 상태 벡터 예측
        self.mu[0] += u[0] * np.cos(theta) * dt
        self.mu[1] += u[0] * np.sin(theta) * dt
        self.mu[4] += u[1] * dt

        # 상태 공분산 행렬 예측
        self.Sigma = G @ self.Sigma @ G.T + self.R

    # 갱신 단계 (Update Step)
    def update(self, z, H, Q):
        # 칼만 이득 계산
        K = self.Sigma @ H.T @ np.linalg.inv(H @ self.Sigma @ H.T + Q)
        # 상태 벡터 갱신
        self.mu = self.mu + K @ (z - H @ self.mu)
        # 상태 공분산 행렬 갱신
        self.Sigma = (np.eye(len(self.mu)) - K @ H) @ self.Sigma

    # 오도메트리 콜백 함수
    def odom_callback(self, msg):
        if not self.initialized:
            return

        # 주기 계산
        dt = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value

        # 오도메트리로부터 x, y, yaw 정보 추출
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = euler_from_quaternion(msg.pose.pose.orientation)[2]

        # 원래 오도메트리 데이터 로그 출력
        self.get_logger().info(f"Original Odometry - x: {x:.3f}, y: {y:.3f}, yaw: {radians_to_degrees(yaw):.2f}°")

        # 측정 벡터 z 구성
        z_odom = np.array([x, y, yaw])

        # 오도메트리 관측 행렬 H 구성
        H_odom = np.zeros((3, 5))
        H_odom[0, 0] = 1
        H_odom[1, 1] = 1
        H_odom[2, 4] = 1

        # 갱신 단계 수행
        self.update(z_odom, H_odom, self.Q_odom)

        # 수정된 오도메트리 퍼블리시
        self.publish_corrected_odometry()

    # IMU 콜백 함수
    def imu_callback(self, msg):
        if not self.initialized:
            self.mu[4] = euler_from_quaternion(msg.orientation)[2]
            self.initialized = True
            return

        # IMU로부터 roll, pitch, yaw 정보 추출
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)

        # 원래 IMU 데이터 로그 출력
        self.get_logger().info(f"Original IMU - Roll: {radians_to_degrees(roll):.2f}°, Pitch: {radians_to_degrees(pitch):.2f}°, Yaw: {radians_to_degrees(yaw):.2f}°")

        # 주기 계산
        dt = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value

        # 측정 벡터 z 구성
        z_imu = np.array([roll, pitch, yaw])

        # IMU 관측 행렬 H 구성
        H_imu = np.zeros((3, 5))
        H_imu[0, 2] = 1
        H_imu[1, 3] = 1
        H_imu[2, 4] = 1

        # 갱신 단계 수행
        self.update(z_imu, H_imu, self.Q_imu)

    # 수정된 오도메트리 퍼블리시 함수
    def publish_corrected_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        odom_msg.child_frame_id = self.get_parameter('base_link_frame').get_parameter_value().string_value
        odom_msg.pose.pose.position.x = float(self.mu[0])
        odom_msg.pose.pose.position.y = float(self.mu[1])
        odom_msg.pose.pose.position.z = 0.0

        roll, pitch, yaw = float(self.mu[2]), float(self.mu[3]), float(self.mu[4])
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        odom_msg.pose.pose.orientation = quaternion
        self.ekf_odom_pub.publish(odom_msg)

        self.get_logger().info(f'Corrected Odometry - Position: ({self.mu[0]:.3f}, {self.mu[1]:.3f}, 0.0), Orientation: (Roll: {radians_to_degrees(roll):.2f}°, Pitch: {radians_to_degrees(pitch):.2f}°, Yaw: {radians_to_degrees(yaw):.2f}°)')

def main(args=None):
    rclpy.init(args=args)

    try:
        ekf_node = EKFNode()
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        ekf_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        ekf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

