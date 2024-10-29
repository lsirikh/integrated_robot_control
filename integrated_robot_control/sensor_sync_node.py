import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')

        # 센서 데이터 구독자 생성
        self.odom_sub = Subscriber(self, Odometry, '/odom')
        self.imu_sub = Subscriber(self, Imu, '/imu/data_raw')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')

        # 동기화된 데이터 Publisher 생성
        self.synced_odom_pub = self.create_publisher(Odometry, '/sync/odom', 10)
        self.synced_imu_pub = self.create_publisher(Imu, '/sync/imu', 10)
        self.synced_scan_pub = self.create_publisher(LaserScan, '/sync/scan', 10)

        # IMU 초기 오프셋 값을 저장할 변수
        self.initial_yaw = None

        # ApproximateTimeSynchronizer: 대략적인 타임스탬프 동기화
        self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub, self.scan_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('sensor_sync_node was registered!')

    def sync_callback(self, odom_msg, imu_msg, scan_msg):
        # IMU 메시지에서 yaw 값 보정
        adjusted_imu_msg = self.adjust_imu_orientation(imu_msg)

        # 동기화된 데이터를 새 토픽에 각각 publish
        self.synced_odom_pub.publish(odom_msg)
        self.synced_imu_pub.publish(adjusted_imu_msg)
        self.synced_scan_pub.publish(scan_msg)

    def adjust_imu_orientation(self, imu_msg):
        # 쿼터니언을 이더 오일러 각으로 변환하여 yaw 값 추출
        orientation_q = imu_msg.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # IMU 초기 yaw 값을 설정
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info(f'Initial IMU yaw set to {self.initial_yaw:.2f} radians')

        # 초기 yaw 값에 따른 오프셋 적용
        adjusted_yaw = yaw - self.initial_yaw

        # -π에서 π 범위로 조정
        if adjusted_yaw > 3.14159:
            adjusted_yaw -= 2 * 3.14159
        elif adjusted_yaw < -3.14159:
            adjusted_yaw += 2 * 3.14159

        # 보정된 yaw를 쿼터니언으로 변환하여 IMU 메시지 업데이트
        adjusted_orientation_q = quaternion_from_euler(0, 0, adjusted_yaw)
        imu_msg.orientation.x = adjusted_orientation_q[0]
        imu_msg.orientation.y = adjusted_orientation_q[1]
        imu_msg.orientation.z = adjusted_orientation_q[2]
        imu_msg.orientation.w = adjusted_orientation_q[3]

        return imu_msg

def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
