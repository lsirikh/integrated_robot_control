#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist


class RobotControlProfileNode(Node):
    def __init__(self):
        super().__init__('robot_control_profile_node')

        # 오도메트리 구독 및 속도 명령 퍼블리셔 설정
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/fused', self.odom_callback, 10)
        # self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.is_fused_started = False

        # 센서 데이터 수신 여부 플래그
        self.odom_received = False
        self.imu_received = False
        self.lidar_received = False

        # 제어 파라미터 초기화
        self.linear_speed = 0.25  # m/s
        self.angular_speed = math.radians(25)  # rad/s
        self.reduced_speed_factor = 0.8  # 속도를 1/3로 줄이는 비율
        self.reduced_speed_factor_radian = 0.8  # 속도를 1/2로 줄이는 비율

        # 방향 보정을 위한 추가 파라미터
        self.kp = 3.0  # 비례 상수 (적절히 조정 필요)
        self.max_angular_speed = math.radians(20)  # 최대 각속도 (rad/s)

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # 움직임 시퀀스 설정
        self.movement_sequence = [
            {'action': 'forward', 'speed': 0.10, 'value': 1.0},
            {'action': 'right_turn', 'speed': 30, 'value': math.radians(90)},
            {'action': 'forward', 'speed': 0.10, 'value': 0.2},
            {'action': 'right_turn', 'speed': 30, 'value': math.radians(90)},
            {'action': 'forward', 'speed': 0.10, 'value': 1.0},
            # {'action': 'left_turn', 'speed': 10, 'value': math.radians(89.78)},
            # {'action': 'forward', 'speed': 0.10, 'value': 0.2},
            # {'action': 'left_turn', 'speed': 10, 'value': math.radians(89.78)},
            # {'action': 'forward', 'speed': 0.10, 'value': 1.0},
        ]
        self.current_step = 0
        self.reached_goal = False

        # 주기적으로 명령을 발행하는 타이머 설정 (0.1초마다 호출)
        self.control_timer = self.create_timer(0.01, self.control_loop)

    def odom_callback(self, msg):
        """오도메트리 콜백에서 현재 위치와 방향 업데이트"""
        self.odom_received = True  # 오도메트리 데이터가 수신됨
        if not self.is_fused_started:
            self.is_fused_started = True
            # 시작 위치와 방향 저장
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.start_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        # 로그 추가: 오도메트리 데이터 수신 여부 확인
        self.get_logger().info(f"Fused_odom data: x={msg.pose.pose.position.x:0.4f}, y={msg.pose.pose.position.y:0.4f}, yaw={self.quaternion_to_yaw(msg.pose.pose.orientation):0.4f}")

    def imu_callback(self, msg):
        """IMU 콜백"""
        self.imu_received = True  # IMU 데이터가 수신됨

    def lidar_callback(self, msg):
        """LiDAR 콜백"""
        self.lidar_received = True  # LiDAR 데이터가 수신됨

    def quaternion_to_yaw(self, q):
        """쿼터니언에서 yaw 값을 계산"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_distance(self, x1, y1, x2, y2):
        """두 점 사이의 거리를 계산"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def normalize_angle(self, angle):
        """각도를 -π에서 π 사이로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_forward(self, target_speed, target_distance= 0.1):
        """로봇을 지정된 거리만큼 전진시키는 함수"""
        self.linear_speed = target_speed
        distance_travelled = self.calculate_distance(self.start_x, self.start_y, self.current_x, self.current_y)

        # 현재 값과 목표 값을 로그로 출력
        self.get_logger().info(f"Moving forward: Target distance={target_distance:.2f}, Current distance={distance_travelled:.2f}")

        twist = Twist()
        if distance_travelled < target_distance:
            # 속도 감소 로직
            if distance_travelled > target_distance * 0.8:
                twist.linear.x = self.linear_speed * self.reduced_speed_factor
            else:
                twist.linear.x = self.linear_speed

            # 방향 보정: 목표 yaw과 현재 yaw의 오차 계산
            yaw_error = self.normalize_angle(self.start_yaw - self.current_yaw)
            angular_correction = self.kp * yaw_error

            # 각속도 제한
            angular_correction = max(-self.max_angular_speed, min(self.max_angular_speed, angular_correction))
            twist.angular.z = angular_correction

            self.get_logger().info(f"Targeted linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
            self.cmd_vel_publisher.publish(twist)
            return False  # 아직 이동 중
        else:
            # 이동 완료
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f"Forward movement of {target_distance}m completed.")
            return True  # 이동 완료

    def turn_right(self, target_speed, target_angle):
        """로봇을 지정된 각도만큼 우회전시키는 함수"""
        self.angular_speed = math.radians(target_speed)  # rad/s
        angle_travelled = self.normalize_angle(self.current_yaw - self.start_yaw)

        # 현재 각도와 목표 각도를 로그로 출력
        self.get_logger().info(
            f"Turning right: Target angle={math.degrees(target_angle):.2f}, Current angle={math.degrees(angle_travelled):.2f}")

        twist = Twist()
        if abs(angle_travelled) < abs(target_angle):
            # 속도 감소 로직
            if abs(angle_travelled) > abs(target_angle) * 0.8:
                twist.angular.z = -self.angular_speed * self.reduced_speed_factor_radian
            else:
                twist.angular.z = -self.angular_speed
            self.cmd_vel_publisher.publish(twist)
            return False  # 아직 회전 중
        else:
            # 회전 완료
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.start_yaw = self.current_yaw
            self.get_logger().info(
                f"Right turn of {math.degrees(target_angle)} degrees completed.")
            return True  # 회전 완료

    def turn_left(self, target_speed, target_angle):
        """로봇을 지정된 각도만큼 좌회전시키는 함수"""
        self.angular_speed = math.radians(target_speed)  # rad/s
        angle_travelled = self.normalize_angle(self.current_yaw - self.start_yaw)

        # 현재 각도와 목표 각도를 로그로 출력
        self.get_logger().info(
            f"Turning left: Target angle={math.degrees(target_angle):.2f}, Current angle={math.degrees(angle_travelled):.2f}")

        twist = Twist()
        if abs(angle_travelled) < abs(target_angle):
            # 속도 감소 로직
            if abs(angle_travelled) > abs(target_angle) * 0.8:
                twist.angular.z = self.angular_speed * self.reduced_speed_factor_radian
            else:
                twist.angular.z = self.angular_speed
            self.cmd_vel_publisher.publish(twist)
            return False  # 아직 회전 중
        else:
            # 회전 완료
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.start_yaw = self.current_yaw
            self.get_logger().info(
                f"Left turn of {math.degrees(target_angle)} degrees completed.")
            return True  # 회전 완료

    def control_loop(self):
        """모든 데이터가 수신되면 움직이기 시작"""
        if not (self.odom_received and self.imu_received and self.lidar_received):
            self.get_logger().info("Waiting for all sensor data...")
            return
        
        if self.reached_goal:
            self.get_logger().info("Goal reached. Stopping the robot.")
            self.stop_robot()
            return

        if self.current_step >= len(self.movement_sequence):
            self.reached_goal = True
            return

        current_action = self.movement_sequence[self.current_step]
        action = current_action['action']
        speed = current_action['speed']
        value = current_action['value']

        if action == 'forward':
            completed = self.move_forward(speed, value)
        elif action == 'right_turn':
            completed = self.turn_right(speed, value)
        elif action == 'left_turn':
            completed = self.turn_left(speed, value)
        else:
            self.get_logger().warn(f"Unknown action: {action}")
            completed = True  # 알 수 없는 액션은 건너뜀

        if completed:
            self.current_step += 1  # 다음 단계로 이동

    def stop_robot(self):
        """로봇을 정지시킵니다."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    try:
        robot_control_profile_node = RobotControlProfileNode()
        rclpy.spin(robot_control_profile_node)
    except KeyboardInterrupt:
        robot_control_profile_node.get_logger().info(
            'Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        robot_control_profile_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
