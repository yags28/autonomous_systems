import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState


class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            JointState, 'joint_state_service', self.handle_joint_service)
        self.get_logger().info("Service is ready to receive requests.")

    def handle_joint_service(self, request, response):
        # Use the correct field names: x, y, and z from the .srv file
        total = request.x + request.y + request.z

        # Set the correct response field: valid from the .srv file
        response.valid = total >= 0

        # Update the logger to use the correct names as well
        self.get_logger().info(
            f'Incoming request\nx: {request.x}, y: {request.y}, z: {request.z}\n'
            f'Sum: {total}, Sending response: {response.valid}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()
    rclpy.spin(service_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()