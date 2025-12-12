import rclpy
from rclpy.node import Node
from task_2_interfaces.msg import JointData
from geometry_msgs.msg import Point32

class talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(JointData, 'joint_topic', 10)
        timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.seconds = 0.0

    def timer_callback(self):
        msg = JointData()
        msg.center = Point32(x=1.0, y=2.0, z=3.0)  # some valid point
        msg.vel = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: center=({msg.center.x}, {msg.center.y}, {msg.center.z}), vel={msg.vel}')

def main(args=None):
    rclpy.init(args=args)
    node = talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
