import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/corrected_odom', 10)

        self.current_imu_data = None
        self.current_odom_data = None

    def imu_callback(self, msg):
        self.current_imu_data = msg
        self.correct_odometry()

    def odom_callback(self, msg):
        self.current_odom_data = msg
        self.correct_odometry()

    def correct_odometry(self):
        if self.current_imu_data and self.current_odom_data:
            corrected_odom = Odometry()
            corrected_odom.header = self.current_odom_data.header
            corrected_odom.child_frame_id = self.current_odom_data.child_frame_id

            # Use IMU orientation directly for corrected odometry
            corrected_odom.pose.pose.orientation = self.current_imu_data.orientation

            # Copy over the position from the original odometry
            corrected_odom.pose.pose.position = self.current_odom_data.pose.pose.position

            # You can add more sophisticated correction logic here

            self.odom_publisher.publish(corrected_odom)
            self.get_logger().info('Published corrected odometry')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
