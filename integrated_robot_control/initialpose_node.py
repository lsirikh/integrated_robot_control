#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 5.0  # Publish every 5 seconds
        self.timer = self.create_timer(timer_period, self.publish_initialpose)

    def publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0] * 36  # Set covariance to a zero-filled list of length 36

        self.publisher_.publish(msg)
        self.get_logger().info("Published initial pose.")

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
