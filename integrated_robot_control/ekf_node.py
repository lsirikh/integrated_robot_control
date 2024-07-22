

import os
import yaml
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion  # Quaternion 클래스 임포트
import transforms3d.euler


# # 함수: 쿼터니언을 오일러 각도로 변환
# def quaternion_to_euler(q):
#     x, y, z, w = q.x, q.y, q.z, q.w
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll = np.arctan2(t0, t1)
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch = np.arcsin(t2)
#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw = np.arctan2(t3, t4)
#     return roll, pitch, yaw

# # 함수: 오일러 각도를 쿼터니언으로 변환
# def quaternion_from_euler(roll, pitch, yaw):
#     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
#     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
#     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
#     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
#     return [qx, qy, qz, qw]

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    q = transforms3d.euler.euler2quat(roll, pitch, yaw)
    quaternion = Quaternion()
    quaternion.x = q[1]
    quaternion.y = q[2]
    quaternion.z = q[3]
    quaternion.w = q[0]
    return quaternion

def euler_from_quaternion(quaternion):
    # quaternion -> euler
    euler = transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    return euler

# 클래스: EKF 노드
class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # 노드 초기화 로그
        self.get_logger().info('Start EKF Node initialized')

        #|x,     y,      z,     |
        #|roll,  pitch,  yaw,   |
        #|vx,    vy,     vz,    |
        #|vroll, vpitch, vyaw,  |
        #|ax,    ay,     az     |

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
                ('odom0_config', [False, False, False, 
                                  False, False, False,
                                  True, False, False, 
                                  False, False, True,
                                  False, False, False]),
                ('imu0', '/imu/data_raw'),
                ('imu0_config', [False, False, False, 
                                 True, True, True,
                                 False, False, False, 
                                 True, True, True,
                                 True, True, True]),
                ('imu0_differential', False),
                ('imu0_remove_gravitational_acceleration', True)
            ]
        )

        # 구독자 생성
        self.odom0_config = self.get_parameter('odom0_config').get_parameter_value().bool_array_value
        self.imu0_config = self.get_parameter('imu0_config').get_parameter_value().bool_array_value

        # Example usage in callbacks
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom0').get_parameter_value().string_value, self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.get_parameter('imu0').get_parameter_value().string_value, self.imu_callback, 10)

        # 퍼블리셔 생성: 필터링된 오도메트리 메시지
        self.ekf_odom_pub = self.create_publisher(Odometry, 'odom/filtered', 10)

        # 상태 벡터와 공분산 행렬 초기화
        self.x = np.zeros(15)
        self.P = np.eye(15)

        self.initialized = False  # 초기화 여부 플래그

        self.get_logger().info("Finish EKF Node initialized")


    # 콜백 함수: 오도메트리 메시지 처리
    def odom_callback(self, msg):
        if not self.initialized:
            return

        dt = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value
        vx = float(msg.twist.twist.linear.x)
        vyaw = float(msg.twist.twist.angular.z)

        z = np.array([0, 0, 0,
                    0, 0, 0,
                    vx, 0, 0,
                    0, 0, vyaw,
                    0, 0, 0])
        indices = [i for i, val in enumerate(self.odom0_config) if val]
        H = np.zeros((len(indices), 15))
        H[:, indices] = np.eye(len(indices))

        z = z[indices]

        if np.all(self.x == 0):
            self.x[6] = vx
            self.x[14] = vyaw
            return

        F = np.eye(15)
        F[0, 6] = dt * np.cos(self.x[8])
        F[1, 6] = dt * np.sin(self.x[8])
        F[0, 8] = -vx * dt * np.sin(self.x[8])
        F[1, 8] = vx * dt * np.cos(self.x[8])
        F[8, 14] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + np.eye(15) * 0.01

        H = H[:, :15]
        R = np.eye(len(indices)) * 0.1
        ys = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ ys
        self.P = (np.eye(15) - K @ H) @ self.P

        self.publish_corrected_odometry()


    # 콜백 함수: IMU 메시지 처리
    def imu_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        vroll = float(msg.angular_velocity.x)
        vpitch = float(msg.angular_velocity.y)
        vyaw = float(msg.angular_velocity.z)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        z = np.array([0, 0, 0,
                      roll, pitch, yaw,
                      0, 0, 0,
                      vroll, vpitch, vyaw,
                      ax, ay, az])
        indices = [i for i, val in enumerate(self.imu0_config) if val]
        H = np.zeros((len(indices), 15))
        H[:, indices] = np.eye(len(indices))

        z = z[indices]

        if not self.initialized:
            self.x[3] = roll
            self.x[4] = pitch
            self.x[8] = yaw
            self.initialized = True

        self.x[9] = vroll
        self.x[10] = vpitch
        self.x[11] = vyaw
        self.x[12] = ax
        self.x[13] = ay
        self.x[14] = az

        # EKF 업데이트
        R = np.eye(len(indices)) * 0.1
        ys = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ ys
        self.P = (np.eye(15) - K @ H) @ self.P 

    
    # 함수: 수정된 오도메트리 퍼블리시
    def publish_corrected_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        odom_msg.child_frame_id = self.get_parameter('base_link_frame').get_parameter_value().string_value
        odom_msg.pose.pose.position.x = float(self.x[0])
        odom_msg.pose.pose.position.y = float(self.x[1])
        odom_msg.pose.pose.position.z = 0.0

        roll, pitch, yaw = 0.0, 0.0, float(self.x[8])
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        odom_msg.pose.pose.orientation = quaternion        

        odom_msg.twist.twist.linear.x = float(self.x[6])  # float으로 변환
        odom_msg.twist.twist.linear.y = float(self.x[7])  # v_y
        odom_msg.twist.twist.angular.z = float(self.x[14])  # float으로 변환
        self.ekf_odom_pub.publish(odom_msg)

        # 쿼터니언을 오일러 각도로 변환하여 로그로 출력
        # corrected_roll, corrected_pitch, corrected_yaw = euler_from_quaternion(quaternion)
        # self.get_logger().info(f'Corrected Odometry - Position: ({self.x[0]:.2f}, {self.x[1]:.2f}, 0.0), '
        #                        f'Orientation: (Roll: {corrected_roll:.2f}, Pitch: {corrected_pitch:.2f}, Yaw: {corrected_yaw:.2f}), '
        #                        f'Velocity: (Linear X: {self.x[6]:.2f}, Linear Y: {self.x[7]:.2f}, Angular Z: {self.x[14]:.2f})')

# 메인 함수: 노드 실행
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
