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
    # quaternion -> euler
    euler = transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
    return euler

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


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('rpi_robot_node')
        self.get_logger().debug(f'Raspberry pi pico was declared!')
        self.declare_parameter('pico_port', '/dev/ttyACM0')

        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        time.sleep(0.2)
        self.port = self.get_parameter('pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
        self.get_logger().info(f'Using serial port {self.ser.name}')
        self.twist = Twist()
        # set timer
        self.pub_period = 0.04  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        # tf
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def pub_callback(self):
        robot_state = self.send_command(self.twist.linear.x, self.twist.angular.z)
        if robot_state is None:
            return

        robot_orientation = quaternion_from_euler(0, 0, robot_state.theta)
        timestamp = self.get_clock().now().to_msg()
        
        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = robot_state.x_pos
        t.transform.translation.y = robot_state.y_pos
        t.transform.translation.z = 0.0325
        t.transform.rotation = robot_orientation
        
        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = robot_state.x_pos
        odom_msg.pose.pose.position.y = robot_state.y_pos
        odom_msg.pose.pose.position.z = 0.325
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = robot_state.v
        odom_msg.twist.twist.angular.z = robot_state.w

        # Log the odometry message
        # self.log_odometry(odom_msg)
        # self.log_robot_state(robot_state)

        # broadcast and publish
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)


    def log_odometry(self, odom_msg):
        self.get_logger().info(f'Odom Message:')
        self.get_logger().info(f'  Header:')
        self.get_logger().info(f'    Stamp: {odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec}')
        self.get_logger().info(f'    Frame ID: {odom_msg.header.frame_id}')
        self.get_logger().info(f'  Child Frame ID: {odom_msg.child_frame_id}')
        self.get_logger().info(f'  Pose:')
        self.get_logger().info(f'    Position:')
        self.get_logger().info(f'      x: {odom_msg.pose.pose.position.x}')
        self.get_logger().info(f'      y: {odom_msg.pose.pose.position.y}')
        self.get_logger().info(f'      z: {odom_msg.pose.pose.position.z}')
        self.get_logger().info(f'    Orientation:')
        self.get_logger().info(f'      x: {odom_msg.pose.pose.orientation.x}')
        self.get_logger().info(f'      y: {odom_msg.pose.pose.orientation.y}')
        self.get_logger().info(f'      z: {odom_msg.pose.pose.orientation.z}')
        self.get_logger().info(f'      w: {odom_msg.pose.pose.orientation.w}')
        self.get_logger().info(f'  Twist:')
        self.get_logger().info(f'    Linear:')
        self.get_logger().info(f'      x: {odom_msg.twist.twist.linear.x}')
        self.get_logger().info(f'      y: {odom_msg.twist.twist.linear.y}')
        self.get_logger().info(f'      z: {odom_msg.twist.twist.linear.z}')
        self.get_logger().info(f'    Angular:')
        self.get_logger().info(f'      x: {odom_msg.twist.twist.angular.x}')
        self.get_logger().info(f'      y: {odom_msg.twist.twist.angular.y}')
        self.get_logger().info(f'      z: {odom_msg.twist.twist.angular.z}')

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
        self.get_logger().debug(f'Data to send: {linear}, {angular}')
        command = f'{linear:.3f},{angular:.3f}/'.encode('UTF-8')
        self.get_logger().debug(f'Sending command: "{command}"')
        self.ser.write(command)
        # while self.ser.in_waiting == 0:
        #     pass

        # Wait for data with a timeout
        timeout = 1.0  # 1 second timeout
        start_time = time.time()
        while self.ser.in_waiting == 0:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for response from serial port')
                return None
            time.sleep(0.01)  # Sleep briefly to prevent busy waiting

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8')

        if (len(res) < 79) or (len(res) > (79 + 13)):
            #self.get_logger().warn(f'Bad data: "{res}"')
            return None
        # else:
        #     self.get_logger().info(f'data: "{res}", bytes: {len(res)}')


        try:
            raw_list = res.strip().split('/')[1].split(',')
            values_list = [float(value) for value in raw_list]
            if len(values_list) != 11:  # Ensure the list has the expected number of elements
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
        # self.get_logger().info(f'[Twist received] {twist.linear.x:.3f}, {twist.angular.z:.3f}')

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
