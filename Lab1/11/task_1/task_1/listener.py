import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        # Create a subscriber to receive the original value
        self.subscription = self.create_subscription(
            Float32,
            'my_first_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Calculate the doubled value from the message data
        doubled_data = msg.data * 2
        
        # Log the result to the terminal
        self.get_logger().info(f'Heard: "{msg.data:.2f}", Doubled: "{doubled_data:.2f}"')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



