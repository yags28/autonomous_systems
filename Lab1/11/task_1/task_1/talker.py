import rclpy
from rclpy.node import Node
from task_2_interfaces.msg import JointData
from std_msgs.msg import Float32
import time


class talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(Float32, 'my_first_topic', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        msg = Float32()
        msg.data = float(elapsed)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Node active for {elapsed:.2f} seconds')


def main(args=None):
    rclpy.init(args=args)
    node = talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
