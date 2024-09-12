#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RobotControlProfileNode(Node):
    def __init__(self):
        super().__init__('robot_control_profile_node')
        
        # 오도메트리 구독 및 속도 명령 퍼블리셔 설정
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/fused', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.is_fused_started = False

        # 제어 파라미터 초기화
        self.current_step = 0
        self.linear_speed = 0.25  # m/s
        self.angular_speed = math.radians(25)  # rad/s
        self.reduced_speed_factor = 1 / 3  # 속도를 1/3로 줄이는 비율
        self.target_distance = [1.0, 0.4, 1.0]  # 각 단계별 목표 거리
        self.target_angle = math.radians(90)  # 90도 회전
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.reached_goal = False

        # 주기적으로 명령을 발행하는 타이머 설정 (0.1초마다 호출)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """오도메트리 콜백에서 현재 위치와 방향 업데이트"""
        if not self.is_fused_started:
            self.is_fused_started = True

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def quaternion_to_yaw(self, q):
        """쿼터니언에서 yaw 값을 계산"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_distance(self, x1, y1, x2, y2):
        """두 점 사이의 거리를 계산"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def control_loop(self):
        """is_fused_started check"""
        if not self.is_fused_started:
            return

        """로봇이 지정된 프로파일로 움직이도록 제어"""
        if self.reached_goal:
            self.get_logger().info("Goal reached. Stopping the robot.")
            self.stop_robot()
            return

        twist = Twist()

        if self.current_step == 0:  # 전진 1.0m
            distance_travelled = self.calculate_distance(self.start_x, self.start_y, self.current_x, self.current_y)
            if distance_travelled < self.target_distance[0]:
                if distance_travelled > self.target_distance[0] * 0.8:
                    twist.linear.x = self.linear_speed * self.reduced_speed_factor  # 속도 줄이기
                else:
                    twist.linear.x = self.linear_speed
            else:
                twist.linear.x = 0.0
                self.current_step += 1
                self.start_yaw = self.current_yaw
                self.get_logger().info("First forward completed. Turning 90 degrees.")

        elif self.current_step == 1:  # 오른쪽 90도 회전
            angle_travelled = abs(self.current_yaw - self.start_yaw)
            if angle_travelled < self.target_angle:
                if angle_travelled > self.target_angle * 0.8:
                    twist.angular.z = -self.angular_speed * self.reduced_speed_factor  # 속도 줄이기
                else:
                    twist.angular.z = -self.angular_speed
            else:
                twist.angular.z = 0.0
                self.current_step += 1
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.get_logger().info("First turn completed. Moving forward again.")

        elif self.current_step == 2:  # 전진 0.4m
            distance_travelled = self.calculate_distance(self.start_x, self.start_y, self.current_x, self.current_y)
            if distance_travelled < self.target_distance[1]:
                if distance_travelled > self.target_distance[1] * 0.8:
                    twist.linear.x = self.linear_speed * self.reduced_speed_factor  # 속도 줄이기
                else:
                    twist.linear.x = self.linear_speed
            else:
                twist.linear.x = 0.0
                self.current_step += 1
                self.start_yaw = self.current_yaw
                self.get_logger().info("Second forward completed. Turning 90 degrees again.")

        elif self.current_step == 3:  # 다시 오른쪽 90도 회전
            angle_travelled = abs(self.current_yaw - self.start_yaw)
            if angle_travelled < self.target_angle:
                if angle_travelled > self.target_angle * 0.8:
                    twist.angular.z = -self.angular_speed * self.reduced_speed_factor  # 속도 줄이기
                else:
                    twist.angular.z = -self.angular_speed
            else:
                twist.angular.z = 0.0
                self.current_step += 1
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.get_logger().info("Second turn completed. Moving forward to final destination.")

        elif self.current_step == 4:  # 마지막 전진 1.0m
            distance_travelled = self.calculate_distance(self.start_x, self.start_y, self.current_x, self.current_y)
            if distance_travelled < self.target_distance[2]:
                if distance_travelled > self.target_distance[2] * 0.8:
                    twist.linear.x = self.linear_speed * self.reduced_speed_factor  # 속도 줄이기
                else:
                    twist.linear.x = self.linear_speed
            else:
                twist.linear.x = 0.0
                self.reached_goal = True
                self.get_logger().info("Final forward completed. Reached goal.")

        # 생성한 Twist 메시지를 퍼블리시
        self.cmd_vel_publisher.publish(twist)

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
        robot_control_profile_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        robot_control_profile_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
