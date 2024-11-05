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
import struct
import binascii

MESSAGE_FORMAT = '<HfffffffHH'  # 리틀 엔디언, 구조체 포맷
CONTROL_MESSAGE_FORMAT = '<HffHH'  # '<'는 리틀 엔디언을 의미합니다.
MESSAGE_SIZE = struct.calcsize(MESSAGE_FORMAT)

Linear = 0.8
Angular = 0.2

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
    

# CRC 계산 함수 수정
def calculate_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


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
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
        self.get_logger().info(f'Using serial port {self.ser.name}')
        self.ser.write(b"$")
        self.get_logger().info(f'Pico data was initialized!...')
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

    def receive_message(self) -> SerialStatus:
        start_time = time.time()
        buffer = bytearray()
        message_found = False

        while True:
            if self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))

                # 메시지의 시작(ctx)와 끝(ext) 위치를 찾음
                while len(buffer) >= 2:
                    # 시작 표시자(ctx)를 찾음
                    ctx_index = buffer.find(b'\x34\x12')  # 0x1234를 리틀 엔디언으로 표현
                    if ctx_index == -1:
                        # 시작 표시자가 없으면 버퍼의 데이터를 버림
                        buffer = bytearray()
                        break
                    else:
                        # 시작 표시자 이후의 데이터를 처리
                        if len(buffer) >= ctx_index + MESSAGE_SIZE:
                            # 충분한 데이터가 버퍼에 있음
                            potential_message = buffer[ctx_index:ctx_index + MESSAGE_SIZE]
                            # 종료 표시자(ext)를 확인
                            ext_value = potential_message[-2:]
                            if ext_value != b'\x78\x56':  # 0x5678을 리틀 엔디언으로 표현
                                # 종료 표시자가 맞지 않으면 시작 위치 이후부터 다시 찾음
                                buffer = buffer[ctx_index + 2:]
                                continue
                            else:
                                # 메시지 발견
                                message_found = True
                                break
                        else:
                            # 데이터가 충분하지 않으면 추가 수신을 기다림
                            break

            # 메시지를 찾았으면 파싱 시작
            if message_found:
                break

            # 타임아웃 처리
            if time.time() - start_time > 0.5:  # 적절한 타임아웃 시간 설정
                self.get_logger().warn('Timeout waiting for message')
                return None
            time.sleep(0.001)  # 짧은 시간 대기

        if not message_found:
            self.get_logger().warn('Failed to find a valid message')
            return None

        # 메시지 파싱
        try:
            msg_tuple = struct.unpack(MESSAGE_FORMAT, potential_message)
        except struct.error as e:
            self.get_logger().warn(f'Failed to unpack message: {e}')
            return None

        ctx = msg_tuple[0]
        l_speed = msg_tuple[1]
        r_speed = msg_tuple[2]
        x_pos = msg_tuple[3]
        y_pos = msg_tuple[4]
        theta = msg_tuple[5]
        v = msg_tuple[6]
        w = msg_tuple[7]
        crc_received = msg_tuple[8]
        ext = msg_tuple[9]

        # ctx와 ext 검증 (이미 버퍼에서 확인했으므로 생략 가능하지만 추가 검증)
        if ctx != 0x1234 or ext != 0x5678:
            self.get_logger().warn(f'Invalid start or end marker after unpacking: ctx={ctx:04X}, ext={ext:04X}')
            return None

        # CRC 검증
        data_for_crc = potential_message[2:-4]  # l_speed부터 w까지의 데이터
        crc_calculated = calculate_crc(data_for_crc)
        if crc_received != crc_calculated:
            self.get_logger().warn(f'CRC mismatch: expected {crc_calculated:04X}, got {crc_received:04X}')
            return None

        # 버퍼에서 사용한 데이터 제거
        buffer = buffer[ctx_index + MESSAGE_SIZE:]
        return SerialStatus(
            left_speed=l_speed,
            right_speed=r_speed,
            x_pos=x_pos,
            y_pos=y_pos,
            theta=theta,
            v=v,
            w=w
        )

    def send_command(self, linear: float, angular: float) -> SerialStatus:

        # ControlMessage 생성
        ctx = 0x1234
        ext = 0x5678

        # 데이터 필드를 바이너리로 패킹하기 전에 CRC를 계산할 데이터 준비
        data_for_crc = struct.pack('<ff', linear, angular)
        crc = calculate_crc(data_for_crc)

        # 전체 메시지를 패킹
        message = struct.pack(CONTROL_MESSAGE_FORMAT, ctx, linear, angular, crc, ext)

        # 메시지 전송
        self.ser.reset_input_buffer()
        self.ser.write(message)
        self.ser.flush()

        # 메시지 수신
        robot_state = self.receive_message()
        if robot_state is None:
            self.get_logger().warn('Failed to receive valid robot state')
            return None

        # 추가 처리 (예: 로그 출력 등)
        return robot_state

    def twist_callback(self, twist: Twist):
            self.twist = twist


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
