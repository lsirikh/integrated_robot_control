import os
import csv
from datetime import datetime
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time



def quaternion_to_yaw(msg):
    """
    ROS 메시지의 쿼터니언을 yaw 각도로 변환하는 함수입니다.

    매개변수:
        msg: Odometry 또는 Imu 메시지와 같이 orientation 정보를 포함하는 ROS 메시지

    반환값:
        yaw: 라디안 단위의 yaw 각도
    """
    # 쿼터니언 구성 요소 추출
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    # yaw (z축 회전) 계산
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return yaw

class DataReportNode(Node):
    def __init__(self):
        super().__init__('data_report_node')

        self.get_logger().info('Start Data Report Node initialized')
         # 구독자 생성: 오도메트리 메시지
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        #self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        #self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        #tf_recorder
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #robot initial coordinate with pose
        self.robot_origin = None
        self.latest_odom_pose = None  # 최신 오도메트리 포즈
        self.lidar_global_points = None  # 변환된 레이저 포인트 저장

        self.odom_data = None
        self.imu_data = None
        self.ekf_data = None
        self.lidar_data = None

        # O 버튼의 이전 상태를 추적하기 위한 변수
        self.previous_button_state = 0

        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/data_report_node_{timestamp_str}.csv')
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')

        self.count = 1
        self.point_index = 0  # 데이터 포인트 인덱스 초기화

        # CSV 파일 경로의 디렉터리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)

        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                # writer.writerow(['timestamp', 'x', 'y', 'theta', 'v', 'w', 'x_corrected', 'y_corrected', 'theta_corrected', 'v_corrected', 'w_corrected', 'lidar_ranges'])
                writer.writerow([
                #                    'no', 'x', 'y',
                                    'timestamp', 
                                    'odom_position_x', 'odom_position_y', 'odom_position_z',
                                    'odom_orientation_x', 'odom_orientation_y', 'odom_orientation_z', 'odom_orientation_w',
                                    'odom_linear_x', 'odom_linear_y', 'odom_linear_z',
                                    'odom_angular_x', 'odom_angular_y', 'odom_angular_z',

                                    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
                                    'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
                                    'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',

                                    'ekf_position_x', 'ekf_position_y', 'ekf_position_z',
                                    'ekf_orientation_x', 'ekf_orientation_y', 'ekf_orientation_z', 'ekf_orientation_w',
                                    'ekf_linear_x', 'ekf_linear_y', 'ekf_linear_z',
                                    'ekf_angular_x', 'ekf_angular_y', 'ekf_angular_z',
                                    'robot_x', 'robot_y', 'robot_yaw',
                                    #'lidar_data'
                                    'true_x', 'true_y'
                                ])
        # timer callback based csv logs storing process
        self.timer = self.create_timer(0.5, self.timer_callback)

        # planned-route 
        self.points = [
            [1, 30, 20], [2, 30, 30], [3, 30, 40], [4, 30, 50], [5, 30, 60], [6, 30, 70], [7, 30, 80], [8, 30, 90], [9, 30, 100], [10, 30, 110],
            [11, 30, 120], [12, 30, 130], [13, 30, 140], [14, 30, 150], [15, 30, 160], [16, 30, 170], [17, 30, 180], [18, 30, 190], [19, 30, 200], [20, 30, 210],
            [21, 30, 220], [22, 30, 230], [23, 30, 240], [24, 30, 250], [25, 40, 250], [26, 50, 250], [27, 50, 240], [28, 50, 230], [29, 50, 220], [30, 50, 210],
            [31, 50, 200], [32, 50, 190], [33, 50, 180], [34, 50, 170], [35, 50, 160], [36, 50, 150], [37, 50, 140], [38, 50, 130], [39, 50, 120], [40, 50, 110],
            [41, 50, 100], [42, 50, 90], [43, 50, 80], [44, 50, 70], [45, 50, 60], [46, 50, 50], [47, 50, 40], [48, 60, 40], [49, 70, 40], [50, 70, 50],
            [51, 70, 60], [52, 70, 70], [53, 70, 80], [54, 70, 90], [55, 70, 100], [56, 70, 110], [57, 70, 120], [58, 70, 130], [59, 70, 140], [60, 70, 150],
            [61, 70, 160], [62, 70, 170], [63, 70, 180], [64, 70, 190], [65, 70, 200], [66, 70, 210], [67, 70, 220], [68, 70, 230], [69, 70, 240], [70, 70, 250],
            [71, 80, 250], [72, 90, 250], [73, 90, 240], [74, 90, 230], [75, 90, 220], [76, 90, 210], [77, 90, 200], [78, 90, 190], [79, 90, 180], [80, 90, 170],
            [81, 90, 160], [82, 90, 150], [83, 90, 140], [84, 90, 130], [85, 90, 120], [86, 90, 110], [87, 90, 100], [88, 90, 90], [89, 90, 80], [90, 90, 70],
            [91, 90, 60], [92, 90, 50], [93, 90, 40], [94, 100, 40], [95, 110, 40], [96, 110, 50], [97, 110, 60], [98, 110, 70], [99, 110, 80], [100, 110, 90],
            [101, 110, 100], [102, 110, 110], [103, 110, 120], [104, 110, 130], [105, 110, 140], [106, 110, 150], [107, 110, 160], [108, 110, 170], [109, 110, 180], [110, 110, 190],
            [111, 110, 200], [112, 110, 210], [113, 110, 220], [114, 110, 230], [115, 110, 240], [116, 110, 250], [117, 120, 250], [118, 130, 250], [119, 130, 240], [120, 130, 230],
            [121, 130, 220], [122, 130, 210], [123, 130, 200], [124, 130, 190], [125, 130, 180], [126, 130, 170], [127, 130, 160], [128, 130, 150], [129, 130, 140], [130, 130, 130],
            [131, 130, 120], [132, 130, 110], [133, 130, 100], [134, 130, 90], [135, 130, 80], [136, 130, 70], [137, 130, 60], [138, 130, 50], [139, 130, 40], [140, 140, 40],
            [141, 150, 40], [142, 150, 50], [143, 150, 60], [144, 150, 70], [145, 150, 80], [146, 150, 90], [147, 150, 100], [148, 150, 110], [149, 150, 120], [150, 150, 130],
            [151, 150, 140], [152, 150, 150], [153, 150, 160], [154, 150, 170], [155, 150, 180], [156, 150, 190], [157, 150, 200], [158, 150, 210], [159, 150, 220], [160, 150, 230],
            [161, 150, 240], [162, 150, 250], [163, 160, 250], [164, 170, 250]
        ]

    def odom_callback(self, msg):
        self.odom_data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]

    def imu_callback(self, msg):
        self.imu_data = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]

    def ekf_callback(self, msg):
        """Odometry 콜백 함수"""
        # Odometry로부터 위치와 방향 추출
        yaw = quaternion_to_yaw(msg)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 로봇의 초기 위치를 설정
        if self.robot_origin is None:
            self.robot_origin = [x, y, yaw]

        # Odometry 데이터를 초기 위치를 기준으로 보정
        x -= self.robot_origin[0]
        y -= self.robot_origin[1]
        yaw -= self.robot_origin[2]

        # yaw를 -π ~ π 범위로 조정
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

        # 최신 Odometry 포즈 저장
        self.latest_odom_pose = [x, y, yaw]

        self.ekf_data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]

    # def lidar_callback(self, msg):
    #     # # 모든 속성과 값을 출력
    #     # for attribute in dir(msg):
    #     #     if not attribute.startswith('_'):  # 내부 속성은 제외
    #     #         attribute_value = getattr(msg, attribute)
    #     #         self.get_logger().info(f'{attribute}: {attribute_value}')
    #     #self.get_logger().info(f'lidar_callback in data_report_node')

    #     angle_min = msg.angle_min
    #     angle_increment = msg.angle_increment
    #     ranges = msg.ranges

    #     self.lidar_data = []
    #     for i, distance in enumerate(ranges):
    #         angle = angle_min + i * angle_increment
    #         angle_deg = angle * 180 / math.pi  # 라디안에서 각도로 변환
    #         self.lidar_data.append((angle_deg, distance))

    def lidar_callback(self, msg):
        # 최신 오도메트리 데이터가 있는지 확인
        if self.latest_odom_pose is None:
            self.get_logger().warn('Odometry data is not available yet')
            return

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        x_robot, y_robot, yaw = self.latest_odom_pose
        self.lidar_global_points = []

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment

            # 유효한 거리 데이터인지 확인
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                continue

            # 로봇 프레임에서의 레이저 포인트 좌표 계산
            x_laser = distance * math.cos(angle)
            y_laser = distance * math.sin(angle)

            # 전역 좌표계로 변환
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            x_global = cos_yaw * x_laser - sin_yaw * y_laser + x_robot
            y_global = sin_yaw * x_laser + cos_yaw * y_laser + y_robot

            # 변환된 좌표를 리스트에 추가
            self.lidar_global_points.append((x_global, y_global))
    
 
    # When timer callback is called, this method runs.
    def timer_callback(self):
        self.save_data_to_csv()

    def save_data_to_csv(self):
        if self.imu_data is None:
            self.get_logger().warn('Imu data is not available yet')
            return

        if self.odom_data is None:
            self.get_logger().warn('Odometry data is not available yet')
            return

        if self.ekf_data is None:
            self.get_logger().warn('Extended Kalman Filter data is not available yet')
            return

        if self.tf_listener is None:
            self.get_logger().warn('Extended Kalman Filter data is not available yet')
            return


        # lidar_data_str = ','.join([f'[{angle:.2f},{distance:.2f}]' for angle, distance in self.lidar_data])
        #lidar_data_str = ';'.join([f'({x:.4f},{y:.4f})' for x, y in self.lidar_global_points])

        # with open(self.csv_file_path, mode='a') as file:
        #     writer = csv.writer(file)
        #     writer.writerow([
        #         datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        #         # f'[{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}', f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}]',
        #         f'{self.odom_data[0]:.4f}', f'{self.odom_data[1]:.4f}', f'{self.odom_data[2]:.4f}',
        #         f'{self.odom_data[3]:.4f}', f'{self.odom_data[4]:.4f}', f'{self.odom_data[5]:.4f}', f'{self.odom_data[6]:.4f}',
        #         f'{self.odom_data[7]:.4f}', f'{self.odom_data[8]:.4f}', f'{self.odom_data[9]:.4f}',
        #         f'{self.odom_data[10]:.4f}', f'{self.odom_data[11]:.4f}', f'{self.odom_data[12]:.4f}',

        #          f'{self.ekf_data[0]:.4f}', f'{self.ekf_data[1]:.4f}', f'{self.ekf_data[2]:.4f}',
        #         f'{self.ekf_data[3]:.4f}', f'{self.ekf_data[4]:.4f}', f'{self.ekf_data[5]:.4f}', f'{self.ekf_data[6]:.4f}',
        #         f'{self.ekf_data[7]:.4f}', f'{self.ekf_data[8]:.4f}', f'{self.ekf_data[9]:.4f}',
        #         f'{self.ekf_data[10]:.4f}', f'{self.ekf_data[11]:.4f}', f'{self.ekf_data[12]:.4f}'
        #         # , f'[{lidar_data_str}]'
        #     ])

        try:
            # Get the latest transform between map and base_link
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            # translation 값을 개별적으로 추출
            translation = trans.transform.translation
            translation_x = translation.x
            translation_y = translation.y
            translation_z = translation.z

            with open(self.csv_file_path, mode='a') as file:
                writer = csv.writer(file)
                writer.writerow([
                    # no, x, y,
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                    *[f'{val:.4f}' for val in self.odom_data],
                    *[f'{val:.4f}' for val in self.imu_data],
                    *[f'{val:.4f}' for val in self.ekf_data],
                    *[f'{val:.4f}' for val in self.latest_odom_pose],
                    f'{translation_x:.4f}', f'{translation_y:.4f}',
                    #lidar_data_str
                ])

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

        


def main(args=None):
    rclpy.init(args=args)
    try:
        data_report_node = DataReportNode()
        rclpy.spin(data_report_node)
    except KeyboardInterrupt:
        data_report_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        data_report_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
