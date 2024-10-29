#!/usr/bin/env python3

from dataclasses import dataclass
import os
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from tf2_ros import TransformBroadcaster
import transforms3d.euler

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu  # 추가: IMU 데이터를 받기 위함
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile

import binascii

@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_speed: float
    right_speed: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float
    

def calculate_crc(data: str) -> int:
    """Calculate CRC-16-CCITT for the given data string."""
    crc = 0xFFFF
    for byte in data.encode('utf-8'):
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
    return crc & 0xFFFF


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('rpi_robot_node')
        self.get_logger().info(f'Raspberry pi pico was declared!')
        self.declare_parameter('pico_port', '/dev/ttyACM0')

        timestamp = self.get_clock().now().to_msg()
        qos_profile = QoSProfile(depth=10)
        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', qos_profile)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)

        time.sleep(0.2)
        self.port = self.get_parameter('pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(self.port, baudrate=9600, timeout=1)
        self.get_logger().info(f'Using serial port {self.ser.name}')
        self.twist = Twist()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.linear_acceleration = 0.015
        self.angular_acceleration = 0.10

        # IMU 데이터를 위한 변수 초기화
        self.current_roll = 0.0
        self.current_pitch = 0.0

        # set timer
        self.pub_period = 0.03  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        # self.pose_period = 10
        # self.pose_timer = self.create_timer(self.pose_period, self.pose_callback)
        
        # tf
        self.tf_broadcaster = TransformBroadcaster(self)

        # serial input Hz calculate
        self.total_data_count = 0      # 전체 데이터 개수
        self.valid_data_count = 0      # 정상 데이터 개수
        self.last_time = None
        self.avg_hz_sum = 0
        self.avg_hz_count = 0
        self.last_log_time = time.time()  # 마지막으로 평균을 출력한 시간

        # 누적 성공 비율 계산을 위한 변수
        self.cumulative_total_data_count = 0   # 누적된 총 데이터 개수
        self.cumulative_valid_data_count = 0   # 누적된 정상 데이터 개수

    
    def imu_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        self.current_roll = roll
        self.current_pitch = pitch

    def pose_callback(self):
        timestamp = self.get_clock().now().to_msg()
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = timestamp
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0

        self.pose_publisher.publish(initial_pose)

    def pub_callback(self):
        #self.velocity_adjust_manager()

        robot_state = self.send_command(self.twist.linear.x, self.twist.angular.z)
        if robot_state is None:
            return

        timestamp = self.get_clock().now().to_msg()
         # tf_transformations 사용
        q = quaternion_from_euler(0, 0, robot_state.theta)
        robot_orientation = Quaternion()
        robot_orientation.x = q[0]
        robot_orientation.y = q[1]
        robot_orientation.z = q[2]
        robot_orientation.w = q[3]
        
        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = robot_state.x_pos
        odom_msg.pose.pose.position.y = robot_state.y_pos
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = robot_state.v
        odom_msg.twist.twist.angular.z = robot_state.w
        self.odom_publisher.publish(odom_msg)

        # # transforms
        # t = TransformStamped()
        # t.header.stamp = timestamp
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = robot_state.x_pos
        # t.transform.translation.y = robot_state.y_pos
        # t.transform.translation.z = 0.0
        # t.transform.rotation = robot_orientation

        # # broadcast and publish
        # self.tf_broadcaster.sendTransform(t)

    def send_command(self, linear: float, angular: float) -> SerialStatus:
        command = f'{linear:.3f},{angular:.3f}/'.encode('UTF-8')
        self.ser.reset_input_buffer()
        self.ser.write(command)
        
        start_time = time.time()
        while self.ser.in_waiting == 0:
            if time.time() - start_time > 0.05:
                self.get_logger().warn('Timeout waiting for data')
                self.total_data_count += 1  # 수신 실패도 전체 데이터로 계산
                self.cumulative_total_data_count += 1  # 누적 총 데이터에도 포함
                return None
            time.sleep(0.02)

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8').strip()
        self.total_data_count += 1  # 수신된 데이터 수에 포함
        self.cumulative_total_data_count += 1  # 누적 총 데이터에도 포함

        try:
            # '/'로 입력값과 데이터 나누기
            if '/' in res:
                input_part, received_data = res.split('/', 1)
            else:
                self.get_logger().warn(f'Invalid format, missing "/": "{res}"')
                return None

            # CRC 분리 및 데이터 유효성 검사
            if ',' in received_data:
                data_part, received_crc = received_data.rsplit(',', 1)
                received_crc = int(received_crc.strip(), 16)  # 16진수로 CRC 변환
            else:
                self.get_logger().warn(f'Invalid format, missing CRC: "{received_data}"')
                return None
            
            # self.get_logger().info(f'[{input_part}]/[{data_part}],[{received_crc}]')

            # CRC Calculate
            calculated_crc = calculate_crc(data_part)
            if received_crc != calculated_crc:
                self.get_logger().error(f'CRC mismatch: expected {calculated_crc:04X}, got {received_crc:04X}')
                return None

            raw_list = data_part.strip().split(',')
            if len(raw_list) != 7:  # 필드 수 일치 확인
                self.get_logger().error(f'Unexpected number of elements in data: {values_list}')
                return None
            
            #At this line get the message successfully.
            values_list = [float(value) for value in raw_list]
            self.valid_data_count += 1  # 정상 데이터 개수 증가
            self.cumulative_valid_data_count += 1  # 누적 정상 데이터에도 포함

            # === 정상 데이터 수신 시 주파수 계산 ===
            current_time = time.time()
            if self.last_time is not None:
                period = current_time - self.last_time
                hz = 1.0 / period if period > 0 else 0
                self.avg_hz_sum += hz
                self.avg_hz_count += 1

            self.last_time = current_time

            # 1초마다 평균 Hz 및 정상/전체 비율 출력
            if current_time - self.last_log_time >= 1.0:
                if self.avg_hz_count > 0:
                    avg_hz = self.avg_hz_sum / self.avg_hz_count
                    success_ratio = (self.valid_data_count / self.total_data_count) * 100
                    cumulative_success_ratio = (self.cumulative_valid_data_count / self.cumulative_total_data_count) * 100  # 누적 성공 비율
                    self.get_logger().info(
                        f"[INFO] Average Hz: {avg_hz:.2f} Hz, Success Rate: {self.valid_data_count}/{self.total_data_count} ({success_ratio:.2f}%)[{cumulative_success_ratio:.2f}%]"
                    )
                    # 평균 Hz와 성공 비율 초기화
                    self.avg_hz_sum = 0
                    self.avg_hz_count = 0
                    self.valid_data_count = 0
                    self.total_data_count = 0
                self.last_log_time = current_time
            # =====================================
        
            return SerialStatus(*values_list)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Parsing error: "{res}" for {e}')
            return None

    def twist_callback(self, twist: Twist):
            self.twist = twist
            # self.get_logger().info(f'[Receive twist] {twist.linear.x:.3f}, {twist.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_control_node = RobotControlNode()
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        robot_control_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        robot_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
