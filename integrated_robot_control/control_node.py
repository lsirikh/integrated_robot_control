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
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu  # 추가: IMU 데이터를 받기 위함
from tf_transformations import quaternion_from_euler, euler_from_quaternion



@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_ref_speed: float
    right_ref_speed: float
    left_speed: float
    right_speed: float
    left_effort: float
    right_effort: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float
    tick_l: int
    tick_r: int


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('rpi_robot_node')
        self.get_logger().debug(f'Raspberry pi pico was declared!')
        self.declare_parameter('pico_port', '/dev/ttyACM0')

        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/robot/odom', 10)


        time.sleep(0.2)
        self.port = self.get_parameter('pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
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
        # tf
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def imu_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        self.current_roll = roll
        self.current_pitch = pitch

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

    def velocity_adjust_manager(self):
        # Adjust linear velocity
        if self.twist.linear.x == 0:
            if abs(self.linear_velocity) > self.linear_acceleration:
                if self.linear_velocity > 0:
                    self.linear_velocity -= self.linear_acceleration
                else:
                    self.linear_velocity += self.linear_acceleration
            else:
                self.linear_velocity = 0
        elif abs(self.twist.linear.x - self.linear_velocity) > self.linear_acceleration:
            if self.twist.linear.x > self.linear_velocity:
                self.linear_velocity += self.linear_acceleration
            else:
                self.linear_velocity -= self.linear_acceleration
        else:
            self.linear_velocity = self.twist.linear.x

        # Adjust angular velocity
        if self.twist.angular.z == 0:
            if abs(self.angular_velocity) > self.angular_acceleration:
                if self.angular_velocity > 0:
                    self.angular_velocity -= self.angular_acceleration
                else:
                    self.angular_velocity += self.angular_acceleration
            else:
                self.angular_velocity = 0
        elif abs(self.twist.angular.z - self.angular_velocity) > self.angular_acceleration:
            if self.twist.angular.z > self.angular_velocity:
                self.angular_velocity += self.angular_acceleration
            else:
                self.angular_velocity -= self.angular_acceleration
        else:
            self.angular_velocity = self.twist.angular.z

        # Check for large sudden changes
        if abs(self.linear_velocity - self.prev_linear_velocity) > self.linear_acceleration * 5:
            self.linear_velocity = self.prev_linear_velocity + self.linear_acceleration * 5 if self.linear_velocity > self.prev_linear_velocity else self.prev_linear_velocity - self.linear_acceleration * 5
        if abs(self.angular_velocity - self.prev_angular_velocity) > self.angular_acceleration * 5:
            self.angular_velocity = self.prev_angular_velocity + self.angular_acceleration * 5 if self.angular_velocity > self.prev_angular_velocity else self.prev_angular_velocity - self.angular_acceleration * 5

        self.prev_linear_velocity = self.linear_velocity
        self.prev_angular_velocity = self.angular_velocity

    def log_robot_state(self, robot_state):
        self.get_logger().info(f'Robot State:')
        self.get_logger().info(f'  Left Ref Speed: {robot_state.left_ref_speed}')
        self.get_logger().info(f'  Right Ref Speed: {robot_state.right_ref_speed}')
        self.get_logger().info(f'  Left Speed: {robot_state.left_speed}')
        self.get_logger().info(f'  Right Speed: {robot_state.right_speed}')
        self.get_logger().info(f'  Left Effort: {robot_state.left_effort}')
        self.get_logger().info(f'  Right Effort: {robot_state.right_effort}')
        self.get_logger().info(f'  X Position: {robot_state.x_pos}')
        self.get_logger().info(f'  Y Position: {robot_state.y_pos}')
        self.get_logger().info(f'  Theta: {robot_state.theta}')
        self.get_logger().info(f'  Linear Velocity: {robot_state.v}')
        self.get_logger().info(f'  Angular Velocity: {robot_state.w}')

    def send_command(self, linear: float, angular: float) -> SerialStatus:
        # self.get_logger().info(f'[Send command] {linear}, {angular}')
        command = f'{linear:.3f},{angular:.3f}/'.encode('UTF-8')
        self.get_logger().debug(f'Sending command: "{command}"')
        self.ser.write(command)
        while self.ser.in_waiting == 0:
            time.sleep(0.01)  # 짧은 시간 동안 대기

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8')

        self.get_logger().info(f'data: "{res}", bytes: {len(res)}')
        if (len(res) < 83) or (len(res) > (83 + 30)):
            #self.get_logger().warn(f'Bad data: "{res}"')
            return None

        try:
            raw_list = res.strip().split('/')[1].split(',')
            values_list = [float(value) for value in raw_list]
            csv_line = ','.join([str(value) for value in values_list])
            self.get_logger().info(f'correct data ==> {csv_line}')

            #self.get_logger().info(f'data: "{res}", bytes: {len(res)}')
            if len(values_list) != 13:  # Ensure the list has the expected number of elements
                self.get_logger().error(f'Unexpected number of elements in data: {values_list}')
                return None
        except ValueError as e:
            # self.get_logger().warn(f'Bad data: "{res}"')
            return None
        except IndexError as e:
            return None

        return SerialStatus(*values_list)
    
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
