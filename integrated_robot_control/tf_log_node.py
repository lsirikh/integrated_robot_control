import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

class TFRecorder(Node):
    def __init__(self):
        super().__init__('tf_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행

    def timer_callback(self):
        try:
            # Get the latest transform between map and base_link
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            # Extract the x, y, z translation
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # Print the x, y position
            self.get_logger().info(f'base_link position in map: x = {x:.3f}, y = {y:.3f}, z = {z:.3f}')

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    tf_recorder = TFRecorder()
    rclpy.spin(tf_recorder)
    tf_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
