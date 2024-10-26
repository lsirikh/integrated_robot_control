import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')

        # 센서 데이터 구독자 생성
        self.odom_sub = Subscriber(self, Odometry, '/odom')
        self.imu_sub = Subscriber(self, Imu, '/imu/data_raw')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')

        # ApproximateTimeSynchronizer: 대략적인 타임스탬프 동기화
        self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub, self.scan_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info(f'sensor_sync_node was registred!')

    def sync_callback(self, odom_msg, imu_msg, scan_msg):
        # 동기화된 데이터를 처리하는 콜백 함수
        self.get_logger().info(f'Synced odom: {odom_msg.header.stamp}, imu: {imu_msg.header.stamp}, scan: {scan_msg.header.stamp}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
