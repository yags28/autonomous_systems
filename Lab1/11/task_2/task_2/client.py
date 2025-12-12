import rclpy
from rclpy.node import Node
from task_2_interfaces.srv import JointState
import sys

class client(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(JointState, 'joint_state_service')
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        
        self.req = JointState.Request()

    def send_request(self, x, y, z):
        self.req.x = x
        self.req.y = y
        self.req.z = z
        
        # Call the service asynchronously
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        # Return the result from the future
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    # Check if the correct number of command-line arguments was given
    if len(sys.argv) != 4:
        print("Usage: ros2 run task_2 client <x> <y> <z>")
        return

    # Create the client node
    client_node = client()

    try:
        # Convert the command-line arguments from strings to floats
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
        # Send the request with the custom arguments
        response = client_node.send_request(x, y, z)
        
        if response:
            client_node.get_logger().info(
                f'Request sent: x={x}, y={y}, z={z}\n'
                f'Response received: valid={response.valid}'
            )
        else:
            client_node.get_logger().error('Service call failed with no response.')
            
    except ValueError:
        client_node.get_logger().error('Invalid arguments. Please provide three numbers.')
    except Exception as e:
        client_node.get_logger().error(f'Service call failed: {e}')
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()